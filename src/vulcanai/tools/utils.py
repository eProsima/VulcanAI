# Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
import threading
import time

from textual.markup import escape  # To remove potential errors in textual terminal


async def run_streaming_cmd_async(
    console, args: list[str], max_duration: float = 60, max_lines: int = 1000, echo: bool = True, tool_name=""
) -> str:
    # Unpack the command
    cmd, *cmd_args = args

    captured_lines: list[str] = []
    process = None
    try:
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

        # Subprocess main loop. Read line by line
        async for raw_line in process.stdout:
            line = raw_line.decode(errors="ignore").rstrip("\n")

            # Print the line
            if echo:
                if args[:3] == ["ros2", "topic", "echo"] and line:
                    msg = line.strip()
                    if msg == "---":
                        continue
                    msg = msg.strip("'\"")
                    line = f"[ROS] [INFO] I heard: [{msg}]"

                captured_lines.append(line)
                line_processed = escape(line)
                if hasattr(console, "add_subprocess_line"):
                    console.add_subprocess_line(line_processed)
                else:
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
        if process is not None:
            process.terminate()

    # Not necessary, textual terminal get the keyboard input
    except KeyboardInterrupt:
        # Ctrl+C pressed → stop subprocess
        console.logger.log_tool("[tool]Ctrl+C received:[/tool] terminating subprocess...", tool_name=tool_name)
        if process is not None:
            process.terminate()

    finally:
        try:
            if process is not None:
                await asyncio.wait_for(process.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            console.logger.log_tool("Subprocess didn't exit in time → killing it.", tool_name=tool_name, error=True)
            if process is not None:
                process.kill()
                await process.wait()
        finally:
            if hasattr(console, "hide_subprocess_panel"):
                console.hide_subprocess_panel()

    return "\n".join(captured_lines)


def execute_subprocess(console, tool_name, base_args, max_duration, max_lines):
    stream_task = None
    done_event = threading.Event()
    result = {"output": ""}

    def _launcher() -> None:
        nonlocal stream_task
        if hasattr(console, "show_subprocess_panel"):
            console.show_subprocess_panel()

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
        # Keep the real task reference so Ctrl+C can cancel it.
        console.set_stream_task(stream_task)

        def _on_done(task: asyncio.Task) -> None:
            try:
                if not task.cancelled():
                    result["output"] = task.result() or ""
            except Exception as e:
                console.logger.log_msg(f"Echo task error: {e!r}\n", error=True)
            finally:
                done_event.set()

        stream_task.add_done_callback(_on_done)

    # `/rerun` workers can have their own asyncio loop in a non-UI thread.
    # Route UI/task creation to Textual app thread unless we are already there.
    if threading.current_thread() is threading.main_thread():
        _launcher()
    else:
        # `console.app` is your Textual App instance.
        console.app.call_from_thread(_launcher)

    console.logger.log_tool("[tool]Subprocess created![tool]", tool_name=tool_name)
    # Wait for streaming command to finish and return collected lines.
    # In UI thread we avoid blocking to prevent deadlocks.
    if threading.current_thread() is threading.main_thread():
        return ""
    done_event.wait()
    return result["output"]


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

    # Add '/' for Topic, service, action, node
    ros_categories_list = ["Topic", "Service", "Action", "Node"]
    if string_name in ros_categories_list and len(input_string) > 0 and input_string[0] != "/":
        input_string = f"/{input_string}"
        ret = input_string

    if input_string not in real_string_list:
        console.logger.log_tool(f'{string_name}: "{input_string}" does not exists', tool_name=tool_name)

        # Get the suggestions list sorted by similitud value
        _, topic_sim_list = _get_suggestions(real_string_list, input_string)

        # Open the ModalScreen
        console.open_radiolist(topic_sim_list, tool_name, string_name, input_string)

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


def last_output_lines(console, tool_name: str, output: str, n_lines: int = 10) -> str:
    """
    Keep only the last `max_lines` lines in tool output and log this behavior.
    """
    lines = output.splitlines()
    if console is not None and hasattr(console, "logger"):
        console.logger.log_tool(
            f"Returning only the last {n_lines} lines in result['output'].",
            tool_name=tool_name,
        )
    return "\n".join(lines[-n_lines:])
