# Copyright 2025 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import asyncio
import difflib
import heapq
import subprocess
import sys
import time

from textual.markup import escape  # To remove potential errors in textual terminal
# sipnner
from textual.timer import Timer
import rclpy
import os
from typing import List, Optional
import threading

class StreamToTextual:
    """
    Class used to redirect the stdout/stderr streams in the textual terminal
    """

    def __init__(self, app, stream_name: str = "stdout"):
        self.app = app
        self.real_stream = getattr(sys, stream_name)

    def write(self, data: str):
        if not data:
            return

        if data.strip():
            # Ensure update happens on the app thread
            # self.app.call_from_thread(self.app.append_log_text, data)
            self.app.call_from_thread(self.app.add_line, data)

    def flush(self):
        self.real_stream.flush()


class SpinnerHook:
    """
    Single entrant spinner controller for console.
    - Starts the spinner on the LLM request.
    - Stops the spinner when LLM request is over.
    """

    def __init__(self, spinner_status):
        self.spinner_status = spinner_status

    def on_request_start(self, text="Querying LLM..."):
        self.spinner_status.start(text)

    def on_request_end(self):
        self.spinner_status.stop()

def attach_ros_logger_to_console(console, node):
    """
    Function that remove ROS node overlaping prints in the terminal
    """

    logger = node.get_logger()

    def info_hook(msg, *args, **kwargs):
        console.call_from_thread(console.logger.log_msg, f"<gray>[ROS] [INFO] {msg}</gray>")

    def warn_hook(msg, *args, **kwargs):
        console.call_from_thread(console.logger.log_msg, f"<gray>[ROS] [WARN] {msg}</gray>")

    def error_hook(msg, *args, **kwargs):
        console.call_from_thread(console.logger.log_msg, f"<gray>[ROS] [ERROR] {msg}</gray>")

    logger.info = info_hook
    logger.warning = warn_hook
    logger.error = error_hook

def common_prefix(strings: str) -> str:
    if not strings:
        return ""

    common_prefix = strings[0]
    commands = strings[0]

    for i in range(1, len(strings)):
        commands += f"    {strings[i]}"

        tmp = ""
        n = min(len(common_prefix), len(strings[i]))
        j = 0

        while j < n:
            if common_prefix[j] != strings[i][j]:
                break
            tmp += common_prefix[j]

            j += 1

        if j < n:
            common_prefix = tmp

    return common_prefix, commands

async def run_streaming_cmd_async(
    console, args: list[str], max_duration: float = 60, max_lines: int = 1000, echo: bool = True, tool_name=""
) -> str:
    # Unpack the command
    cmd, *cmd_args = args

    # Create the subprocess
    process = await asyncio.create_subprocess_exec(
        cmd,
        *cmd_args,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )

    assert process.stdout is not None

    start_time = time.monotonic()
    line_count = 0

    try:
        # Subprocess main loop. Read line by line
        async for raw_line in process.stdout:
            line = raw_line.decode(errors="ignore").rstrip("\n")

            # Print the line
            if echo:
                line_processed = escape(line)
                console.add_line(line_processed)
       # Count the line
            line_count += 1
            if max_lines is not None and line_count >= max_lines:
                console.logger.log_tool(f"[tool]Stopping:[/tool] Reached max_lines = {max_lines}", tool_name=tool_name)
                console.set_stream_task(None)
                process.terminate()
                break

            # Check duration
            if max_duration and (time.monotonic() - start_time) >= max_duration:
                console.logger.log_tool(
                    f"[tool]Stopping:[/tool] Exceeded max_duration = {max_duration}s", tool_name=tool_name
                )
                console.set_stream_task(None)
                process.terminate()
                break

    except asyncio.CancelledError:
        # Task was cancelled → stop the subprocess
        console.logger.log_tool("[tool]Cancellation received:[/tool] terminating subprocess...", tool_name=tool_name)
        process.terminate()
        raise
    # Not necessary, textual terminal get the keyboard input
    except KeyboardInterrupt:
        # Ctrl+C pressed → stop subprocess
        console.logger.log_tool("[tool]Ctrl+C received:[/tool] terminating subprocess...", tool_name=tool_name)
    finally:
        try:
            await asyncio.wait_for(process.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            console.logger.log_tool("Subprocess didn't exit in time → killing it.", tool_name=tool_name, error=True)
            process.kill()
            await process.wait()

    return "Process stopped due to Ctrl+C"


def execute_subprocess(console, tool_name, base_args, max_duration, max_lines):
    stream_task = None

    def _launcher() -> None:
        nonlocal stream_task
        # This always runs in the Textual event-loop thread
        loop = asyncio.get_running_loop()
        stream_task = loop.create_task(
            run_streaming_cmd_async(
                console,
                base_args,
                max_duration=max_duration,
                max_lines=max_lines,
                tool_name=tool_name,  # tool_header_str
            )
        )

        def _on_done(task: asyncio.Task) -> None:
            if task.cancelled():
                # Normal path → don't log as an error
                # If you want a message, call UI methods directly here,
                # not via console.write (see Fix 2)
                return

            try:
                task.result()
            except Exception as e:
                console.logger.log_msg(f"Echo task error: {e!r}\n", error=True)
                # result["output"] = False
                return

        stream_task.add_done_callback(_on_done)

    try:
        # Are we already in the Textual event loop thread?
        asyncio.get_running_loop()
    except RuntimeError:
        # No loop here → probably ROS thread. Bounce into Textual thread.
        # `console.app` is your Textual App instance.
        console.app.call_from_thread(_launcher)
    else:
        # We *are* in the loop → just launch directly.
        _launcher()

    # Store the task in the console to be able to cancel it later
    console.set_stream_task(stream_task)
    console.logger.log_tool("[tool]Subprocess created![tool]", tool_name=tool_name)


def run_oneshot_cmd(args: list[str]) -> str:
    try:
        return subprocess.check_output(args, stderr=subprocess.STDOUT, text=True)

    except subprocess.CalledProcessError as e:
        raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")


def suggest_string(console, tool_name, string_name, input_string, real_string_list):
    ret = None

    def _similarity(a: str, b: str) -> float:
        """Return a similarity score between 0 and 1."""
        return difflib.SequenceMatcher(None, a, b).ratio()

    def _get_suggestions(real_string_list_comp: list[str], string_comp: str) -> tuple[str, list[str]]:
        """
        Function used to search for the most "similar" string in a list.

        Used in ROS2 cli commands to used the "simmilar" in case
        the queried topic does not exists.

        Example:
        real_string_list_comp = [
            "/parameter_events",
            "/rosout",
            "/turtle1/cmd_vel",
            "/turtle1/color_sensor",
            "/turtle1/pose",
        ]
        string_comp = "trtle1"

        @return
            str: the most "similar" string
            list[str] a sorted list by a similitud value
        """

        topic_list_pq = []
        n = len(string_comp)

        for string in real_string_list_comp:
            m = len(string)
            # Calculate the similitud
            score = _similarity(string_comp, string)
            # Give more value for the nearest size comparations.
            score -= abs(n - m) * 0.005
            # Max-heap (via negative score)
            heapq.heappush(topic_list_pq, (-score, string))

        # Pop ordered list
        ret_list: list[str] = []
        _, most_topic_similar = heapq.heappop(topic_list_pq)

        ret_list.append(most_topic_similar)

        while topic_list_pq:
            _, topic = heapq.heappop(topic_list_pq)
            ret_list.append(topic)

        return most_topic_similar, ret_list

    if input_string not in real_string_list:
        # console.add_line(f"{tool_header_str} {string_name}: \"{input_string}\" does not exists")
        console.logger.log_tool(f'{string_name}: "{input_string}" does not exists', tool_name=tool_name)

        # Get the suggestions list sorted by similitud value
        _, topic_sim_list = _get_suggestions(real_string_list, input_string)

        # Open the ModalScreen
        console.open_radiolist(topic_sim_list, f"{tool_name}")

        # Wait for the user to select and item in the
        # RadioList ModalScreen
        console.suggestion_index_changed.wait()

        # Check if the user cancelled the suggestion
        if console.suggestion_index >= 0:
            ret = topic_sim_list[console.suggestion_index]

        # Reset suggestion index
        console.suggestion_index = -1
        console.suggestion_index_changed.clear()

    return ret
