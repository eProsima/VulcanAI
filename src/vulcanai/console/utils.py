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
from textual.markup import escape # To remove potential errors in textual terminal
from textual.timer import Timer

color_tool = "#EB921E"

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

        # optional: still write to real stdout/stderr
        self.real_stream.write(data)
        self.real_stream.flush()

        if data.strip():
            # Ensure update happens on the app thread
            #self.app.call_from_thread(self.app.append_log_text, data)
            self.app.call_from_thread(self.app.add_line_dq, data)
            self.app.render_log()

    def flush(self):
        self.real_stream.flush()

class SpinnerHook:
    """
    Single entrant spinner controller for console.
    - Starts the spinner on the LLM request.
    - Stops the spinner when LLM request is over.
    """

    def __init__(self, console):

        self.console = console

        # Spinner states
        self.spinner_timer: Timer | None = None
        self.spinner_frames = ["⠋","⠙","⠹","⠸","⠼","⠴","⠦","⠧","⠇","⠏"]
        self.spinner_frame_index = 0
        self.spinner_line_index: int | None = None

    def on_request_start(self, text: str = "Querying LLM...") -> None:
        """
        Create the spinner line at the end of the log and start updating it.
        """

        self.color = "#0d87c0"
        self.update_color = "#15B606"
        self.text = text

        # Check if it is already running
        if self.spinner_timer is not None:
            return

        # Initialized the class variables
        self.console._log(f"[{self.color}]{text}[/{self.color}]")
        self.spinner_line_index = len(self.console.log_lines_dq) - 1
        self.spinner_frame_index = 0

        # Update every 0.1s
        self.spinner_timer = self.console.set_interval(0.1, self.update_spinner)
        # Update the terminal
        self.console.render_log()

    def update_spinner(self) -> None:
        """
        Timer callback. Rotate the spinner frame on the stored last log line.
        """

        # Check if the spinner is not running
        if self.spinner_line_index is None:
            return

        frame = self.spinner_frames[self.spinner_frame_index]
        self.spinner_frame_index = (self.spinner_frame_index + 1) % len(self.spinner_frames)

        # Update that specific line only
        self.console.log_lines_dq[self.spinner_line_index] = \
            f"[{self.update_color}]{frame}[/{self.update_color}] " + \
            f"[{self.color}]{self.text}[/{self.color}]"

        # Update the terminal
        self.console.render_log()

    def on_request_end(self) -> None:
        """
        Stop the spinner.
        Optional, replace the line with final_text.
        """

        # Check if the spinner is running
        if self.spinner_timer is not None:
            self.spinner_timer.stop()
            self.spinner_timer = None

        # Update the spinner message line
        if self.spinner_line_index is not None:
            self.console.log_lines_dq.pop()
            self.spinner_line_index = None
            self.console.render_log()


def attach_ros_logger_to_console(console, node):
    """
    Function that remove ROS node overlaping prints in the terminal
    """

    logger = node.get_logger()

    def info_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[INFO] {msg}[/gray]")

    def warn_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[WARN] {msg}[/gray]")

    def error_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[ERROR] {msg}[/gray]")

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

async def run_streaming_cmd_async(console, args: list[str],
        max_duration: float = 60,
        max_lines: int = 1000,
        echo: bool = True,
        print_header="") -> str:


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
                console._log(line_processed)#, subprocess_flag=True)

            # Count the line
            line_count += 1
            if max_lines is not None and line_count >= max_lines:
                console._log(f"{print_header} " + \
                    f"[yellow]Stopping: [bold]reached max_lines = {max_lines}[/bold][/yellow]"
                )
                console.set_stream_task(None)
                process.terminate()
                break

            # Check duration
            if max_duration and (time.monotonic() - start_time) >= max_duration:
                console._log(f"{print_header} " + \
                    f"[yellow]Stopping: [bold]exceeded max_duration = {max_duration}s[/bold] [/yellow]"
                )
                console.set_stream_task(None)
                process.terminate()
                break


    except asyncio.CancelledError:
        # Task was cancelled → stop the subprocess
        console._log(f"{print_header} [yellow][bold]Cancellation received:[/bold] terminating subprocess...[/yellow]")
        process.terminate()
        raise
    # Not necessary, textual terminal get the keyboard input
    except KeyboardInterrupt:
        # Ctrl+C pressed → stop subprocess
        console._log(f"{print_header} [yellow][bold]Ctrl+C received:[/bold] terminating subprocess...[/yellow]")
        process.terminate()

    finally:
        try:
            await asyncio.wait_for(process.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            console._log(f"{print_header} Subprocess didn't exit in time → killing it.", log_color=0)
            process.kill()
            await process.wait()

    return "Process stopped due to Ctrl+C"


def execute_subprocess(console, tool_name, base_args, max_duration, max_lines):

    stream_task = None
    tool_header_str = f"[bold {color_tool}]\[TOOL [italic]{tool_name}[/italic]][/bold {color_tool}]"

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
                print_header=tool_header_str
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
                console._log(f"Echo task error: {e!r}\n", log_color=0)
                #result["output"] = False
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


    console.set_stream_task(stream_task)
    console._log(f"{tool_header_str} [yellow][bold]Subprocess created![/bold][/yellow]")

def run_oneshot_cmd(args: list[str]) -> str:
    try:
        return subprocess.check_output(
            args,
            stderr=subprocess.STDOUT,
            text=True
        )

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
            _ , topic = heapq.heappop(topic_list_pq)
            ret_list.append(topic)

        return most_topic_similar, ret_list

    if input_string not in real_string_list:

        tool_header_str = f"[bold {color_tool}]\[TOOL [italic]{tool_name}[/italic]][/bold {color_tool}]"

        console._log(f"{tool_header_str} {string_name}: \"{input_string}\" does not exists")

        # Get the suggestions list sorted by similitud value
        _, topic_sim_list = _get_suggestions(real_string_list, input_string)

        # Open the ModalScreen
        console.open_radiolist(topic_sim_list, f" {tool_name}")

        # Wait for the user to select and item in the
        # RadioList ModalScreen
        while console.suggestion_index == -1:
            time.sleep(0.01)

        # Check if the user cancelled the suggestion
        if console.suggestion_index >= 0:
            ret = topic_sim_list[console.suggestion_index]

        # Reset suggestion index
        console.suggestion_index = -1

    return ret

