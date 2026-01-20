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

import re


class VulcanAILogger:

    """
    Logger class for VulcanAI components.
    Provides methods to log messages with different tags and colors.

    TODO. try unlinking the console with the logger.
    Right now the logger needs VulcanAI console object to output messages.
    """

    vulcanai_theme = {
            "registry": "#068399",
            "manager": "#0d87c0",
            "executor": "#15B606",
            "vulcanai": "#56AA08",
            "user": "#91DD16",
            "validator": "#C49C00",
            "tool": "#EB921E",
            "error": "#FF0000",
            "console": "#8F6296",
            "warning": "#D8C412",
        }

    def __init__(self, console):
        self.console = console

    # region UTILS

    def parse_color(self, msg):
        """
        Parse custom [tag]...[/tag] in messages and convert them
        to <style>...</style> based on the vulcanai_theme defined colors.
        """

        # Matches [tag] or [/tag]
        pattern = re.compile(r'\[(\/?)([^\]]+)\]')

        def replace_tag(match):
            slash, tag = match.groups()

            # If the tag is defined in the theme, replace it
            if tag in self.vulcanai_theme:
                return f"<{slash}{self.vulcanai_theme[tag]}>"

            # Otherwise, keep the original tag
            return match.group(0)

        return pattern.sub(replace_tag, msg)

    def process_msg(self, msg: str, prefix: str = "", color: str = "") -> str:
        color_begin = color_end = color
        if color != "":
            if color in self.vulcanai_theme:
                color = self.vulcanai_theme["console"]
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix}{color_begin}{msg}{color_end}"

        return self.parse_color(msg)

    # endregion

    # region LOG

    def log_manager(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"[error][MANAGER] [ERROR][/error] "
        else:
            prefix = f"<bold>[manager][MANAGER][/manager]</bold> "

        processed_msg = self.process_msg(msg, color=color, prefix=prefix)
        self.console.add_line(processed_msg)

    def log_executor(self, msg: str, error: bool = False, tool: bool = False, tool_name: str = '', color: str = ""):
        if error:
            prefix = f"[error][EXECUTOR] [ERROR][/error] "
        elif tool:
            self.log_tool(msg, tool_name=tool_name)
            return
        else:
            prefix = f"[executor][EXECUTOR][/executor] "

        processed_msg = self.process_msg(msg, color=color, prefix=prefix)
        self.console.add_line(processed_msg)

    def log_tool(self, msg: str, tool_name: str = '', error: bool = False, color: str = ""):
        if tool_name:
            tag = f"<bold>[TOOL <italic>{tool_name}</italic>]</bold>"
        else:
            tag = '<bold>[TOOL]</bold>'
        if error:
            prefix = f"[error]{tag} [ERROR][/error] "
        else:
            prefix = f"[tool]{tag}[/tool] "

        processed_msg = self.process_msg(msg, color=color, prefix=prefix)
        self.console.add_line(processed_msg)

    def log_registry(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"<bold>[error][REGISTRY] [ERROR][/error]</bold> "
        else:
            prefix = f"<bold>[registry][REGISTRY][/registry]</bold> "

        processed_msg = self.process_msg(msg, color=color, prefix=prefix)
        self.console.add_line(processed_msg)

    def log_validator(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = f"[error][VALIDATOR] [ERROR][/error] "
        else:
            prefix = f"[validator][VALIDATOR][/validator] "

        processed_msg = self.process_msg(msg, color=color, prefix=prefix)
        self.console.add_line(processed_msg)

    def log_console(self, msg: str, color: str = ""):
        if color == "":
            msg = f"[console]{msg}[/console]"

        processed_msg = self.parse_color(msg)
        if color == "":
            self.console.add_line(processed_msg)
        else:
            log_color = self.vulcanai_theme.get(color, "")
            self.console.add_line(processed_msg, self.vulcanai_theme["console"] if log_color != "" else "")

    def log_msg(self, msg: str, error: bool = False, color: str = ""):
        if error:
            color = self.vulcanai_theme.get("error", "")

        processed_msg = self.process_msg(msg, color=color)
        self.console.add_line(processed_msg)

    def log_user(self, msg: str):
        prefix = f"<bold>[user][USER] >>>[/user]</bold> "

        processed_msg = self.process_msg(msg, prefix=prefix)
        self.console.add_line(processed_msg)

    # endregion