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



class VulcanAILogger:

    """
    Logger class for VulcanAI components.
    Provides methods to log messages with different tags and colors.

    Needs VulcanAI console object with a _log method to output messages.
    """

    def __init__(self, console):
        self.console = console

        self.vulcanai_theme = {
            "registry": "#068399",
            "manager": "#0d87c0",
            "executor": "#15B606",
            "validator": "#C49C00",
            "tool": "#EB921E",
            "error": "#FF0000",
            "console": "#8F6296",
            "warning": "#D8C412",
        }

    # region UTILS

    def parse_color(self, msg):
        """
        Function used to parse custom [tag]...[/tag] in messages and convert them
        to <style>...</style> based on the vulcanai_theme defined colors.
        (style can be color, bold, italic, underline, etc.)

        returns the processed message with textual styles.
        """

        ret = ""
        i = 0
        n = len(msg)

        # Parse the message looking for [tag]...[/tag] patterns
        while i < n:
            if msg[i] == '[':
                # Found a potential tag start/end
                tmp = ""
                i += 1
                end = ""

                # Check if it's an end tag, by looking for '/' in the next char
                if msg[i] == '/':
                    i += 1
                    end = "/"

                # Get the tag name
                while i < n and msg[i] != ']':
                    tmp += msg[i]
                    i += 1

                # Build the style replacement
                style = f"[{end}{tmp}]"
                # Check if the tag is in the vulcanai_theme to replace by color
                # if not found, keep the original message tag
                if tmp in self.vulcanai_theme:
                    style = f"<{end}{self.vulcanai_theme[tmp]}>"
                ret += f"{style}"

            else:
                # Adds regular characters
                ret += msg[i]

            i += 1

        return ret

    # endregion

    # region LOG

    def log_manager(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"[error][MANAGER] [ERROR][/error]"
        else:
            prefix = f"<bold>[manager][MANAGER][/manager]</bold>"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_executor(self, msg: str, error: bool = False, tool: bool = False, tool_name: str = '', color: str = ""):
        if error:
            prefix = f"[error][EXECUTOR] [ERROR][/error]"
        elif tool:
            self.log_tool(msg, tool_name=tool_name)
            return
        else:
            prefix = f"[executor][EXECUTOR][/executor]"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_tool(self, msg: str, tool_name: str = '', error: bool = False, color: str = ""):
        if tool_name:
            tag = f"<bold>[TOOL <italic>{tool_name}</italic>]</bold>"
        else:
            tag = '<bold>[TOOL]</bold>'
        if error:
            prefix = f"[error]{tag} [ERROR][/error]"
        else:
            prefix = f"[tool]{tag}[/tool]"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_registry(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"<bold>[error][REGISTRY] [ERROR][/error]</bold>"
        else:
            prefix = f"<bold>[registry][REGISTRY][/registry]</bold>"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_validator(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"[error][VALIDATOR] [ERROR][/error]"
        else:
            prefix= f"[validator][VALIDATOR][/validator]"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_error(self, msg: str, color: str = ""):

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"[error][ERROR][/error] {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_console(self, msg: str, color: str = ""):
        if color == "":
            msg = f"[console]{msg}[/console]"

        processed_msg = self.parse_color(msg)
        if color == "":
            self.console._log(processed_msg)
        else:
            log_color = self.vulcanai_theme.get(color, "")
            self.console._log(processed_msg, color=2 if log_color != "" else -1)

    def log_warning(self, msg: str, color: str = ""):
        prefix = f"<bold>[warning][WARN][/warning]</bold>"

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix} {color_begin}{msg}{color_end}"

        processed_msg = self.parse_color(msg)
        self.console._log(processed_msg)

    def log_msg(self, msg: str, error: bool = False, color: str = ""):
        if error:
            color = self.vulcanai_theme.get("error", "")

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        processed_msg = self.parse_color(f"{color_begin}{msg}{color_end}")
        self.console._log(processed_msg)

    # endregion