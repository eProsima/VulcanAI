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

import sys


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
