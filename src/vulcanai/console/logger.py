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
from typing import Optional, Protocol


class LogSink(Protocol):
    """A default console that prints to standard output."""

    def write(self, msg: str, color: str = "") -> None: ...


class RichStdoutSink:
    def __init__(self, logger_theme) -> None:
        from rich.console import Console
        from rich.theme import Theme

        self.console = Console(theme=Theme(logger_theme))

    def write(self, msg: str, color: str = "") -> None:
        self.console.print(msg)


class VulcanAILogger:
    """
    Logger class for VulcanAI components.
    Provides methods to log messages with different tags and colors.
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

    _default_instance: Optional["VulcanAILogger"] = None
    _rich_markup = True

    @classmethod
    def default(cls) -> "VulcanAILogger":
        """Get the default VulcanAILogger instance. This method acts as a singleton factory for the logger."""
        if cls._default_instance is None:
            cls._default_instance = cls()
        return cls._default_instance

    def __init__(self, sink: Optional[LogSink] = None):
        # A default console will be used if none is provided
        self.sink: LogSink = sink or RichStdoutSink(VulcanAILogger.vulcanai_theme)

    # region UTILS

    def set_sink(self, sink: "LogSink") -> None:
        """Set a new sink for the logger."""
        self.sink = sink

    def set_textualizer_console(self, textual_console) -> None:
        """Set a Textual console as sink for the logger and configure formatting tags."""
        self.set_sink(textual_console)
        VulcanAILogger._rich_markup = False

    def parse_color(self, msg):
        """
        Parse custom [tag]...[/tag] in messages and convert them
        to <style>...</style> based on the vulcanai_theme defined colors.
        """

        # Matches [tag] or [/tag]
        pattern = re.compile(r"\[(\/?)([^\]]+)\]")

        def replace_tag(match):
            slash, tag = match.groups()

            # If the tag is defined in the theme, replace it
            if tag in self.vulcanai_theme:
                return f"<{slash}{self.vulcanai_theme[tag]}>"

            # Otherwise, keep the original tag
            return match.group(0)

        return pattern.sub(replace_tag, msg)

    def parse_rich_markup(self, msg: str) -> str:
        """Parse rich markup tags if rich markup is enabled."""
        if VulcanAILogger._rich_markup:
            # Convert <bold> / </bold> / <italic> / </italic> etc.
            msg = re.sub(r"<(/?)(bold|italic|underline|reverse|dim)>", r"[\1\2]", msg)
            # Convert hex colors: <#RRGGBB> ... </#RRGGBB>  -> [#RRGGBB] ... [/#RRGGBB]
            msg = re.sub(r"<(/?)(#[0-9a-fA-F]{6})>", r"[\1\2]", msg)
        return msg

    def process_msg(self, msg: str, prefix: str = "", color: str = "") -> str:
        """Process the message by adding prefix, applying color formatting and rich markup if enabled."""
        color_begin = color_end = color
        if color != "":
            if color in self.vulcanai_theme:
                color = self.vulcanai_theme[color]
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        msg = f"{prefix}{color_begin}{msg}{color_end}"

        return self.parse_rich_markup((self.parse_color(msg)))

    # endregion

    # region LOG

    def log_manager(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = "[error][MANAGER] [ERROR][/error] "
        else:
            prefix = "<bold>[manager][MANAGER][/manager]</bold> "

        processed_msg = self.process_msg(msg, prefix=prefix, color=color)
        self.sink.write(processed_msg)

    def log_executor(self, msg: str, error: bool = False, tool: bool = False, tool_name: str = "", color: str = ""):
        if error:
            prefix = "[error][EXECUTOR] [ERROR][/error] "
        elif tool:
            self.log_tool(msg, tool_name=tool_name)
            return
        else:
            prefix = "[executor][EXECUTOR][/executor] "

        processed_msg = self.process_msg(msg, prefix=prefix, color=color)
        self.sink.write(processed_msg)

    def log_tool(self, msg: str, tool_name: str = "", error: bool = False, color: str = ""):
        if tool_name:
            tag = f"<bold>[TOOL <italic>{tool_name}</italic>]</bold>"
        else:
            tag = "<bold>[TOOL]</bold>"
        if error:
            prefix = f"[error]{tag} [ERROR][/error] "
        else:
            prefix = f"[tool]{tag}[/tool] "

        processed_msg = self.process_msg(msg, prefix=prefix, color=color)
        self.sink.write(processed_msg)

    def log_registry(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = "[error][REGISTRY] [ERROR][/error] "
        else:
            prefix = "<bold>[registry][REGISTRY][/registry]</bold> "

        processed_msg = self.process_msg(msg, prefix=prefix, color=color)
        self.sink.write(processed_msg)

    def log_validator(self, msg: str, error: bool = False, color: str = ""):
        if error:
            prefix = "[error][VALIDATOR] [ERROR][/error] "
        else:
            prefix = "[validator][VALIDATOR][/validator] "

        processed_msg = self.process_msg(msg, prefix=prefix, color=color)
        self.sink.write(processed_msg)

    def log_console(self, msg: str, color: str = ""):
        if color == "":
            msg = f"[console]{msg}[/console]"

        processed_msg = self.parse_color(msg)
        if color == "":
            self.sink.write(processed_msg)
        else:
            log_color = self.vulcanai_theme.get(color, "")
            self.sink.write(processed_msg, self.vulcanai_theme["console"] if log_color != "" else "")

    def log_msg(self, msg: str, error: bool = False, color: str = ""):
        if error:
            color = self.vulcanai_theme.get("error", "")

        processed_msg = self.process_msg(msg, color=color)
        self.sink.write(processed_msg)

    def log_user(self, msg: str):
        prefix = "<bold>[user][USER] >>>[/user]</bold> "

        processed_msg = self.process_msg(msg, prefix=prefix)
        self.sink.write(processed_msg)

    # endregion
