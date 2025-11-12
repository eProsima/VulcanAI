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

from rich.console import Console
from rich.theme import Theme

vulcanai_theme = Theme({
    "manager": "bold blue",
    "executor": "bold green",
    "step": "bold yellow",
    "tool": "bold cyan",
    "validator": "bold orange_red1",
    "error": "bold red",
    "console": "bold magenta"
})

console = Console(theme=vulcanai_theme)


class VulcanAILogger:
    """Logger class for VulcanAI components."""
    @staticmethod
    def log_manager(msg: str, error: bool = False):
        if error:
            msg = f"[error][MANAGER] [ERROR][/error] {msg}"
        else:
            msg = f"[manager][MANAGER][/manager] {msg}"
        #console.print(msg)
        console.print_system(msg)

    @staticmethod
    def log_executor(msg: str, error: bool = False, tool: bool = False, tool_name: str = ''):
        if error:
            msg = f"[error][EXECUTOR] [ERROR][/error] {msg}"
        elif tool:
            VulcanAILogger.log_tool(msg, tool_name=tool_name)
            return
        else:
            msg = f"[executor][EXECUTOR][/executor] {msg}"
        #console.print(msg)
        console.print_system(msg)

    @staticmethod
    def log_tool(msg: str, tool_name: str = '', error: bool = False):
        if tool_name:
            tag = f"[TOOL [italic]{tool_name}[/italic]]"
        else:
            tag = '[TOOL]'
        if error:
            msg = f"[error]{tag} [ERROR][/error]  {msg}"
        else:
            msg = f"[step]{tag}[/step] {msg}"
        #console.print(msg)
        console.print_system(msg)

    @staticmethod
    def log_registry(msg: str, error: bool = False):
        if error:
            msg = f"[error][REGISTRY] [ERROR][/error]  {msg}"
        else:
            msg = f"[tool][REGISTRY][/tool] {msg}"
        #console.print(msg)
        console.print_system(msg)

    @staticmethod
    def log_validator(msg: str):
        msg = f"[validator][VALIDATOR][/validator] {msg}"
        #console.print(msg)
        console.print_system(msg)

    @staticmethod
    def log_error(msg: str):
        #console.print(f"[error][ERROR][/error] {msg}")
        console.print_system(f"[error][ERROR][/error] {msg}")
