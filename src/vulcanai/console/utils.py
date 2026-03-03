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
import subprocess
import sys
import threading
import time

from textual.markup import escape  # To remove potential errors in textual terminal


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
        app = getattr(self.spinner_status, "app", None)
        if app is not None and threading.current_thread() is not threading.main_thread():
            app.call_from_thread(self.spinner_status.start, text)
        else:
            self.spinner_status.start(text)

    def on_request_end(self):
        app = getattr(self.spinner_status, "app", None)
        if app is not None and threading.current_thread() is not threading.main_thread():
            app.call_from_thread(self.spinner_status.stop)
        else:
            self.spinner_status.stop()


def attach_ros_logger_to_console(console):
    """
    Redirect ALL rclpy RcutilsLogger output (nodes + executor + rclpy internals)
    to a Textual console.
    """

    try:
        from rclpy.impl.rcutils_logger import RcutilsLogger
    except ImportError:
        console.logger.log_msg(
            "No ROS 2 installation could be found. ROS 2 logs will not be redirected to VulcanAI console."
        )
        return

    # Textual
    app = console.app
    if app is None:
        # The textual terminal is not initialized
        return

    # Avoid double-patching
    if getattr(RcutilsLogger, "_textual_patched", False):
        # Already attached
        return

    def _write(markup: str) -> None:
        console.logger.log_msg(markup)

    def patched_log(self, msg, level, *args, **kwargs):
        # Format message similarly to printf-style logger
        try:
            level = (level % args) if args else str(level)
        except Exception:
            level = f"{level} {args}"

        ros_log_lev_dict = {
            "10": "DEBUG",
            "20": "INFO",
            "30": "WARN",
            "40": "ERROR",
            "50": "FATAL",
        }

        markup = f"<gray>[ROS] [{ros_log_lev_dict[level]}]"

        name = getattr(self, "name", "ros")  # Logger name if available
        if name != "":
            markup += f" [{name}] "
        markup += f"[{msg}]</gray>"

        # Ensure UI-thread safe write
        if threading.get_ident() == getattr(app, "_thread_id", None):
            app.call_later(_write, markup)
        else:
            app.call_from_thread(_write, markup)

    RcutilsLogger.log = patched_log
    RcutilsLogger._textual_patched = True


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
        # Keep the real task reference so Ctrl+C can cancel it.
        console.set_stream_task(stream_task)

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

    # Worker threads may have their own asyncio loop; only run directly on UI thread.
    if threading.current_thread() is threading.main_thread():
        _launcher()
    else:
        console.app.call_from_thread(_launcher)

    console.logger.log_tool("[tool]Subprocess created![tool]", tool_name=tool_name)


def run_oneshot_cmd(args: list[str]) -> str:
    try:
        return subprocess.check_output(args, stderr=subprocess.STDOUT, text=True)

    except subprocess.CalledProcessError as e:
        raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")
