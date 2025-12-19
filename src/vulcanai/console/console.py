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

from __future__ import annotations

import argparse
import asyncio
import os
import pyperclip  # To paste the clipboard into the terminal
import sys

from collections import deque
from textual import events, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.events import MouseEvent
from textual.markup import escape  # To remove potential errors in textual terminal
from textual.widgets import Input, Static, TextArea

from vulcanai.console.modal_screens import CheckListModal, RadioListModal, ReverseSearchModal
from vulcanai.console.utils import attach_ros_logger_to_console, common_prefix, SpinnerHook, StreamToTextual

from vulcanai.console.CustomLogTextArea import CustomLogTextArea


class VulcanAILogger:

    """Logger class for VulcanAI components."""

    def __init__(self, console):
        self.console = console

        self.vulcanai_theme = {
            "registry": "#068399",
            "manager": "#0d87c0",
            "executor": "#15B606",
            "validator": "#C49C00",
            "error": "#FF0000",
            "console": "#8F6296",
        }

    def parse_color(self, msg):
        ret = ""

        i = 0
        n = len(msg)
        while i < n:
            if msg[i] == '[':
                tmp = ""
                i += 1
                end = ""
                if msg[i] == '/':
                    i += 1
                    end = "/"

                while i < n and msg[i] != ']':
                    tmp += msg[i]
                    i += 1

                style = f"[{end}{tmp}]"
                if tmp in self.vulcanai_theme:
                    style = f"<{end}{self.vulcanai_theme[tmp]}>"
                ret += f"{style}"

            else:
                ret += msg[i]

            i += 1

        return ret


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
            tag = f"[TOOL <italic>{tool_name}</italic>]"
        else:
            tag = '[TOOL]'
        if error:
            prefix = f"[error]{tag} [ERROR][/error]"
        else:
            prefix = f"[validator]{tag}[/validator]"

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

    def log_msg(self, msg: str, color: str = ""):

        color_begin = color_end = color
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        processed_msg = self.parse_color(f"{color_begin}{msg}{color_end}")
        self.console._log(processed_msg)


class VulcanConsole(App):

    # CSS = """
    # #log { height: 1fr; }
    # #cmd { dock: bottom; }
    # """

    CSS = """
    Screen {
        layout: horizontal;
    }

    #left {
        width: 1fr;
    }

    #right {
        width: 48;
        layout: vertical;
        border: tall #56AA08;
        padding: 0;
    }

    #logcontent {
        height: 1fr;
    }

    #cmd {
        dock: bottom;
    }

    #history_title {
        content-align: center middle;
        margin: 0;
        padding: 0;
    }

    #history_scroll {
        height: 1fr;      /* fill remaining space */
        margin: 1;
    }

    #history {
        width: 100%;
    }
    """

    BINDINGS = [
        Binding("ctrl+l", "clear_log", "Clear log"),
        Binding("f2", "show_help", "Show help", priority=True),
        Binding("ctrl+r", "reverse_search", "Reverse search"),
        Binding("ctrl+c", "stop_streaming_task", "Stop Streaming"),
        Binding("up", "history_prev", show=False),
        Binding("down", "history_next", show=False),
        Binding("f3", "copy", "Copy selection"),
    ]

    def __init__(self, tools_from_entrypoints: str = "", user_context: str = "", main_node = None,
                 model: str = "gpt-5-nano", k: int = 7, iterative: bool = False):
        super().__init__() # Textual lib

        self.manager = None
        self.last_plan = None
        self.last_bb = None
        self.hooks = SpinnerHook(self)

        self.model = model
        self.k = k

        self.iterative = iterative
        self.tools_from_entrypoints = tools_from_entrypoints
        self.user_context = user_context
        self.main_node = main_node

        self.commands = None
        self.tab_matches = []
        self.tab_index = 0
        self.log_lines_dq = deque(maxlen=500)

        # Terminal qol
        self.history = []

        self.stream_task = None
        self.suggestion_index = -1

        self.left_pannel = None

        self.logger = VulcanAILogger(self)


    async def on_mouse_down(self, event: MouseEvent) -> None:
        """
        Function used to paste the string for the user clipboard
        """

        if event.button == 2:  # Middle click
            await self._paste_clipboard()
            event.prevent_default()
            event.stop()

    async def on_mount(self) -> None:
        # TODO. danip
        self.left_pannel = self.query_one("#logcontent", CustomLogTextArea)

        # Disable terminal input
        self.set_input_enabled(False)
        sys.stdout = StreamToTextual(self, "stdout")
        sys.stderr = StreamToTextual(self, "stderr")

        self.loop = asyncio.get_running_loop()

        await asyncio.sleep(0)
        asyncio.create_task(self.bootstrap())

    def compose(self) -> ComposeResult:

        vulcanai_title_slant = """[#56AA08]\
 _    __      __                 ___    ____
| |  / /_  __/ /________  ____  /   |  /  _/
| | / / / / / / ___/ __ `/ __ \/ /| |  / /
| |/ / /_/ / / /__/ /_/ / / / / ___ |_/ /
|___/\__,_/_/\___/\__,_/_/ /_/_/  |_/___/[/#56AA08]
"""

        with Horizontal():
            # LEFT
            with Vertical(id="left"):
                yield CustomLogTextArea(id="logcontent")
                yield Input(placeholder="> ", id="cmd")

            # RIGHT
            with Vertical(id="right"):
                yield Static(vulcanai_title_slant, id="history_title")
                yield Static(f" Loading info...", id="variables")
                with VerticalScroll(id="history_scroll"):
                    # IMPORTANT: markup=True so [bold reverse] works
                    yield Static("", id="history", markup=True)

    async def bootstrap(self, user_input: str="") -> None:
        """
        Function used to print information at runtime execution of a function
        """

        def worker(user_input: str="") -> None:

            # INITIALIZE CODE
            if user_input == "":
                self.init_manager()

                # Add the commands
                # Command registry: name -> handler
                self.commands = {
                    "/help": self.cmd_help,
                    "/tools": self.cmd_tools,
                    "/edit_tools": self.cmd_edit_tools,
                    "/change_k": self.cmd_change_k,
                    "/history": self.cmd_history_index,
                    "/show_history": self.cmd_show_history,
                    "/clear_history": self.cmd_clear_history,
                    "/plan": self.cmd_plan,
                    "/rerun": self.cmd_rerun,
                    "/bb": self.cmd_blackboard_state,
                    "/clear": self.cmd_clear,
                    "/exit": self.cmd_quit,
                }

                # Cycling through tab matches
                self.tab_matches = []
                self.tab_index = 0

                # Override hooks with spinner controller
                try:
                    self.manager.llm.set_hooks(self.hooks)
                except Exception:
                    pass

                current_path = os.path.dirname(os.path.abspath(__file__))
                self.manager.register_tools_from_file(f"{current_path}/../tools/default_tools.py")

                if self.tools_from_entrypoints != "":
                    self.manager.register_tools_from_entry_points(self.tools_from_entrypoints)

                self.manager.add_user_context(self.user_context)

                self.manager.bb["console"] = self

                # Add the shared node to the console manager blackboard to be used by tools
                if self.main_node != None:
                    self.manager.bb["main_node"] = self.main_node
                    attach_ros_logger_to_console(self, self.main_node)
                else:
                    self._log("WARNING. No ROS node added", log_color=3)

            # MAIN FUNCTION (Handle commands or queries)
            else:
                # Disable terminal input
                self.set_input_enabled(False)

                try:
                    images = []

                    # Add the images
                    if "--image=" in user_input:
                        images = self.get_images(user_input)

                    # Handle user request
                    try:
                        result = self.manager.handle_user_request(user_input, context={"images": images})
                    except Exception as e:

                        self._log(f"[error]Error handling request:[/error] {e}")
                        return

                    self.last_plan = result.get("plan", None)
                    self.last_bb = result.get("blackboard", None)

                    bb_ret = result.get('blackboard', {None})
                    bb_ret = str(bb_ret).replace('<', '\'').replace('>', '\'')


                    self._log(f"Output of plan: {bb_ret}", log_color=2)
                except KeyboardInterrupt:
                    self._log("<yellow>Exiting...</yellow>")
                    return
                except EOFError:
                    self._log("<yellow>Exiting...</yellow>")
                    return

        # This is the main thread, here is where the hook is started for queries
        if user_input != "":
            self.hooks.on_request_start()

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker(user_input))

        if user_input == "":
            self.is_ready = True
            self._log("VulcanAI Interactive Console", log_color=2)
            self._log("Type <bold>'exit'</bold> to quit.", log_color=2)
            #self._log("")

        # Activate the terminal input
        self.set_input_enabled(True)

    # region Utilities

    def _apply_history_to_input(self) -> None:
        cmd_input = self.query_one("#cmd", Input)
        if self.history_index is None or self.history_index == len(self.history):
            cmd_input.value = ""
        else:
            cmd_input.value = self.history[self.history_index]
        cmd_input.focus()

    def _update_history_panel(self) -> None:
        """Update right-hand history with highlight."""
        history_widget = self.query_one("#history", Static)

        lines = []
        for i, cmd in enumerate(self.history):
            text = f"{i+1:>3}: {escape(cmd)}"
            if self.history_index is not None and self.history_index == i:
                # Highlight current selection
                text = f"[bold reverse]{text}[/]"
            lines.append(text)

        history_widget.update("\n".join(lines))

    def _update_variables_panel(self) -> None:
        text = f" AI model: {self.model}\n K = {self.manager.k}\n history_depth = {self.manager.history_depth}"
        kvalue_widget = self.query_one("#variables", Static)
        kvalue_widget.update(text)

    @work  # Runs in a worker. waiting won't freeze the UI
    async def open_checklist(self, tools_list: list[str], active_tools_num: int) -> None:
        """
        Function used in '/edit_tools' command.
        It creates a dialog with all the tools.
        """

        # Create the checklist dialog
        selected = await self.push_screen_wait(CheckListModal(tools_list, active_tools_num))

        if selected is None:
            self._log("Selection cancelled.", log_color=3)
        else:

            for tool_tmp in tools_list:
                # Remove "- "
                tool = tool_tmp[2:]
                if tool_tmp in selected:
                    if self.manager.registry.activate_tool(tool):
                        self._log(f"Activated tool <bold>'{tool}'</bold>", log_color=2)
                else:
                    if self.manager.registry.deactivate_tool(tool):
                        self._log(f"Deactivated tool <bold>'{tool}'</bold>", log_color=2)

    def open_radiolist(self, option_list: list[str], tool: str = "") -> str:
        """
        Function used to open a RadioList ModalScreen in the suggestion
        string process.

        Used when the user inputs a wrong topic, service, action, ...
        """

        def _radiolist_callback(selected: str | None):
            """
            Docstring for _radiolist_callback

            :param selected: Description
            :type selected: str | None
            """

            color_tmp = "#EB921E"
            if selected is None:
                self._log(f"<bold {color_tmp}>[TOOL<italic>{tool}</italic>]</bold {color_tmp}> " + \
                          f"Suggestion cancelled")
                self.suggestion_index = -2
                return

            self._log(f"<bold {color_tmp}>[TOOL<italic>{tool}</italic>]</bold {color_tmp}> " + \
                      f"Selected suggestion: \"{option_list[selected]}\"")
            self.suggestion_index = selected

        self.app.call_from_thread(
            lambda: self.push_screen(
                RadioListModal(option_list),
                callback=_radiolist_callback,
            )
        )

    # endregion

    # region Commands

    def cmd_help(self, _) -> None:
        table = "\n".join(
            [
                "___________________\n"
                "<bold>Available commands:</bold>\n"
                "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
                "/<bold>help</bold>           - Show this help message\n"
                "/<bold>tools</bold>          - List available tools\n"
                "/<bold>edit_tools</bold>     - Edit the list of available tools\n"
                "/<bold>change_k 'int'</bold> - Change the 'k' value for the top_k algorithm selection or show the current value if no 'int' is provided\n"
                "/<bold>history 'int'</bold>  - Change the history depth or show the current value if no 'int' is provided\n"
                "/<bold>show_history</bold>   - Show the current history\n"
                "/<bold>clear_history</bold>  - Clear the history\n"
                "/<bold>plan</bold>           - Show the last generated plan\n"
                "/<bold>rerun</bold>          - Rerun the last plan\n"
                "/<bold>bb</bold>             - Show the last blackboard state\n"
                "/<bold>clear</bold>          - Clears the console screen\n"
                "/<bold>exit</bold>           - Exit the console\n"
                "<bold>Query any other text</bold> to process it with the LLM and execute the plan generated.\n\n"
                "Add --image='path' to include images in the query. It can be used multiple times to add more images.\n"
                "Example: 'user_prompt' --image=/path/to/image1 --image=/path/to/image2'\n"
                "___________________\n"
                "<bold>Available keybinds:</bold>\n"
                "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
                "<bold>F2</bold>                - Show this help message\n"
                "<bold>Ctrl+Q</bold>            - Exit the console\n"
                "<bold>Ctrl+L</bold>            - Clears the console screen\n"
                "<bold>Ctrl+U</bold>            - Clears the entire command line input\n"
                "<bold>Ctrl+K</bold>            - Clears from the cursor to then end of the line\n"
                "<bold>Ctrl+W</bold>            - Delete the word before the cursor\n"
                "<bold>Ctrl+'left/right'</bold> - Move cursor backward/forward by one word\n"
                "<bold>Ctrl+R</bold>            - Reverse search through command history (try typing part of a previous command).\n"
            ]
        )
        self._log(table, log_color=2)

    def cmd_tools(self, _) -> None:
        tmp_msg = f"(current index k={self.manager.k})"

        tool_msg = ("_" * len(tmp_msg)) + '\n'
        tool_msg += f"<bold>Available tools:</bold>\n"
        tool_msg += tmp_msg + '\n' + ("‾" * len(tmp_msg)) + '\n'

        for tool in self.manager.registry.tools.values():
            tool_msg += f"- <bold>{tool.name}:</bold> {tool.description}\n"
        self._log(tool_msg, log_color=2)

    def cmd_edit_tools(self, _) -> None:
        tools_list = []
        for tool in self.manager.registry.tools.values():
            tools_list.append(f"- {tool.name}")

        active_tools_num = len(tools_list)

        for deactivated_tool in self.manager.registry.deactivated_tools.values():
            tools_list.append(f"- {deactivated_tool.name}")

        self.open_checklist(tools_list, active_tools_num)

    def cmd_change_k(self, args) -> None:
        if len(args) == 0:
            self._log(f"Current 'k' is {self.manager.k}", log_color=2)
            return
        if len(args) != 1 or not args[0].isdigit():
            self._log(f"Usage: /change_k 'int' - Actual 'k' is {self.manager.k}",
                      log_type="error", log_color=2)
            return

        new_k = int(args[0])
        self.manager.k = new_k
        self.manager.update_k_index(new_k)
        self._update_variables_panel()

    def cmd_history_index(self, args) -> None:
        if len(args) == 0:
            self._log(f"Current 'history depth' is {self.manager.history_depth}",
                      log_color=2)
            return
        if len(args) != 1 or not args[0].isdigit():
            self._log(f"Usage: /history 'int' - Actual 'history depth' is {self.manager.history_depth}",
                      log_type="error", log_color=2)
            return

        new_hist = int(args[0])
        self.manager.update_history_depth(new_hist)
        self._update_variables_panel()

    def cmd_show_history(self, _) -> None:
        if not self.manager.history:
            self._log("No history available.", log_color=2)
            return

        history_msg = "\nCurrent history (oldest first):\n"
        for i, (user_text, plan_summary) in enumerate(self.manager.history):
            history_msg += f"{i+1}. User: {user_text}\n   Plan summary: {plan_summary}\n"

        self._log(history_msg, log_color=2)

    def cmd_clear_history(self, _) -> None:
        """Clear all history and reset UI."""

        # Reset history storage
        self.history.clear()
        self.history_index = None

        # Update right panel (empty)
        history_widget = self.query_one("#history", Static)
        history_widget.update("")

        # Update left terminal log (empty)
        #log = self.query_one("#logcontent", Static)
        #log.update("")
        #self.left_pannel.update("")

        # Add feedback line
        self._log("History cleared.")

    def cmd_plan(self, _) -> None:
        if self.last_plan:
            self._log("Last generated plan:", log_color=2)
            self._log(self.last_plan, log_color=2)
        else:
            self._log("No plan has been generated yet.", log_color=2)

    def cmd_rerun(self, _) -> None:
        if self.last_plan:
            self._log("Rerunning last plan...", log_color=2)
            result = self.manager.executor.run(self.last_plan, self.manager.bb)
            self.last_bb = result.get("blackboard", None)
            self._log(f"Output of rerun: {result.get('blackboard', {None})}", log_color=2)
        else:
            self._log("No plan to rerun.", log_color=2)

    def cmd_blackboard_state(self, _) -> None:
        if self.last_bb:
            self._log("Last blackboard state:", log_color=2)
            self._log(self.last_bb, log_color=2)
        else:
            self._log("No blackboard available.", log_color=2)

    def cmd_clear(self, _) -> None:
        self.log_lines_dq.clear()
        self.left_pannel.clear_console()
        #self.query_one("#logcontent", Static).update("")

    def cmd_quit(self, _) -> None:
        self.exit()

    def cmd_echo(self, args) -> None:
        self._log(" ".join(args))

    # endregion

    # region Logging

    def add_line_dq(self, input: str,
            color: str = "",
            subprocess_flag: bool = False) -> None:

        # Split incoming text into individual lines
        lines = input.splitlines()

        color_begin = ""
        color_end = ""
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"


        # Append each line; deque automatically truncates old ones
        for line in lines:
            line_processed = line
            if subprocess_flag:
                line_processed = escape(line)
            #self.log_lines_dq.append(f"{color_begin}{line_processed}{color_end}")
            text = f"{color_begin}{line_processed}{color_end}"
            self.left_pannel.append_marked(text)

    def replace_line(self, input: str, row: int,
            color: str = "",
            subprocess_flag: bool = False) -> None:

        # Split incoming text into individual lines
        lines = input.splitlines()

        color_begin = ""
        color_end = ""
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"


        # Append each line; deque automatically truncates old ones
        for line in lines:
            line_processed = line
            if subprocess_flag:
                line_processed = escape(line)
            #self.log_lines_dq.append(f"{color_begin}{line_processed}{color_end}")
            text = f"{color_begin}{line_processed}{color_end}"
            self.left_pannel.replace_row(text, row)

    def delete_last_line(self):
        self.left_pannel.delete_last_row()

    def render_log(self, text: str = "") -> None:
        # log_area = self.query_one("#logcontent", TextArea)

        # # Set the full content
        # log_area.text = "\n".join(self.log_lines_dq)
        if text == "":
            self.left_pannel.append_marked(self.log_lines_dq[-1])
        else:
            self.left_pannel.append_marked(text)

        # Keep view pinned to bottom (like tail -f)
        #log_area.scroll_end(animate=False)

    def _log(self, text: str,
            log_type: str = "", log_color: int = -1,
            subprocess_flag: bool = False) -> None:

        if log_color == 0:
            color_type = "#FF0000"
        elif log_color == 1:
            color_type = "#56AA08"
        elif log_color == 2:
            color_type = "#8F6296"
        elif log_color == 3:
            color_type = "#C49C00"
        elif log_color == 4:
            color_type = "#069899"
        else:
            color_type = ""

        self.add_line_dq(text, subprocess_flag=subprocess_flag, color=color_type)
        #self.render_log(text="")
        #self.render_log(text=text)
        return

    def print_command_prompt(self, cmd: str=""):
        """
        Prints in the terminal the prompt where the user command inputs
        are written. [USER] >>> <command_input>
        """

        color_user = "#91DD16"
        self._log(f"<bold {color_user}>[USER] >>></bold {color_user}> {cmd}")

    # endregion

    # region Input

    def set_input_enabled(self, enabled: bool) -> None:
        cmd = self.query_one("#cmd", Input)
        cmd.disabled = not enabled
        if enabled:
            self.set_focus(cmd)

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        """
        Function for entering a key
        """

        if not self.is_ready:
            return

        cmd = event.value.strip()
        if not cmd:
            return

        cmd_input = self.query_one("#cmd", Input)

        try:
            if event.input.id != "cmd":
                return
            user_input = (event.value or "").strip()

            # The the user_input in the history navigation list (used when the up, down keys are pressed)
            self.history.append(user_input)
            self.history_index = len(self.history)
            self._update_history_panel()
            event.input.value = ""
            event.input.focus()

            # Reset tab state
            self.tab_matches = []
            self.tab_index = 0

            if not user_input:
                cmd_input.focus()
                return

            # Echo what the user typed (keep this if you like the prompt arrow)
            self.print_command_prompt(cmd)

            # If it start with '/', just print it as output and stop here
            if user_input.startswith("/"):
                self.handle_command(user_input)
                return

            await asyncio.sleep(0)
            asyncio.create_task(self.bootstrap(user_input))

        except KeyboardInterrupt:
            self._log("<yellow>Exiting...</yellow>")
            return
        except EOFError:
            self._log("<yellow>Exiting...</yellow>")
            return

    def handle_command(self, user_input: str) -> None:
        # Otherwise, parse as a command
        parts = user_input.split()
        cmd = parts[0].lower()
        args = parts[1:]

        handler = self.commands.get(cmd)
        if handler is None:
            # Only complain for slash-commands
            self._log(f"Unknown command: [b]{cmd}[/]. Type '/help'.", log_color=2)
        else:
            try:
                handler(args)
            except Exception as e:
                self._log(f"Error: {e!r}", log_color=0)

    async def _paste_clipboard(self) -> None:
        cmd_input = self.query_one("#cmd", Input)

        try:
            paste_text = pyperclip.paste() or ""
        except Exception as e:
            self._log(f"Clipboard error: {e}", log_color=0)
            return

        if not paste_text:
            return

        # Remove endlines
        cut = paste_text.find("\n")
        if cut != -1:
            paste_text = paste_text[:cut]

        value = cmd_input.value
        cursor = cmd_input.cursor_position

        # Insert text at cursor
        cmd_input.value = value[:cursor] + paste_text + value[cursor:]
        cmd_input.cursor_position = cursor + len(paste_text)
        cmd_input.focus()


    async def on_key(self, event: events.Key) -> None:
        """
        Handle keys.

        Navigate throw history: "up", "down"
        Autocomplete command: "tab"
        Delete the word before: "Ctrl + w"
        Delethe the word after: "Ctrl + delete", "escape"
        Paste the copied text in the clipboard to the input terminal: "Ctrl + v"
        """

        key = event.key
        cmd_input = self.query_one("#cmd", Input)

        # AUTOCOMPLETE: Tab
        if key == "tab":
            # Current text (don’t strip right-side spaces; keep user’s spacing)
            raw = cmd_input.value or ""
            # Leading spaces are not part of the command token
            left = len(raw) - len(raw.lstrip())
            value = raw[left:]

            if len(value) <= 0:
                return

            # Split into head (first token) and the remainder
            head, *rest = value.split(maxsplit=1)
            # Nothing typed yet: list all commands
            all_cmds = sorted(self.commands) if self.commands else []
            if not all_cmds:
                return

            self.tab_matches = [c for c in all_cmds if c.startswith(head)] if head else all_cmds
            self.tab_index = 0

            matches = self.tab_matches
            if not matches:
                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # If multiple matches, check for a longer common prefix to insert first
            if len(matches) > 1:
                prefix, commands = common_prefix(matches)
                new_value = prefix
                self.print_command_prompt(cmd_input)
                self._log(commands, log_color=2)
            else:
                # Single match: complete directly
                new_value = matches[0]

            # Rebuild the input value:
            cmd_input.value = new_value
            cmd_input.cursor_position = len(cmd_input.value)

            cmd_input.focus()
            event.prevent_default()
            event.stop()
            return

        # DELETE before
        if key == "ctrl+w":
            value = cmd_input.value
            cursor = cmd_input.cursor_position
            i = cursor-1

            while i > 0:
                if(value[i] == ' '):
                    break
                i -= 1

            cmd_input.value = value[:i] + value[cursor:]
            cmd_input.cursor_position = i

            cmd_input.focus()
            event.prevent_default()
            event.stop()
            return

        # DELETE after
        if key in ("ctrl+delete", "escape") :
            value = cmd_input.value
            cursor = cmd_input.cursor_position
            i = cursor
            n = len(value)
            count = 0
            first = False

            while i < n:
                if(value[i] == ' '):
                    if first:
                        break
                else:
                    first = True
                i += 1
                count +=1

            cmd_input.value = value[:cursor] + value[i:]
            cmd_input.cursor_position = max(i - count, 0)

            cmd_input.focus()
            event.prevent_default()
            event.stop()
            return

        # PASTE
        if key == "ctrl+v":
            await self._paste_clipboard()
            event.prevent_default()
            event.stop()
            return

        # Any other keypress resets tab cycle if the prefix changes
        if len(key) == 1 or key in ("backspace", "delete"):
            self.tab_matches = []
            self.tab_index = 0

    def set_stream_task(self, input_stream):
        """
        Function used in the tools to set the current streaming task.
        with this variable the user can finish the execution of the
        task by using the signal "Ctrl + C"
        """

        self.stream_task = input_stream

    # endregion

    # region Actions (key bindings)

    # Called by Binding("ctrl+l", "clear_log", ...)
    def action_clear_log(self) -> None:
        self.cmd_clear(_=None)

    # Called by Binding("ctrl+h", "show_help", ...)
    def action_show_help(self) -> None:
        self.print_command_prompt("/help")

        self.cmd_help(_=None)

    # Called by Binding("ctrl+r", "reverse_search", ...)
    def action_reverse_search(self) -> None:
        if not self.history:
            return

        def done(result: str | None) -> None:
            """
            Callback when modal closes.
            """

            cmd_input = self.query_one("#cmd", Input)
            if result:
                cmd_input.value = result
                cmd_input.cursor_position = len(result)
            cmd_input.focus()

        self.push_screen(ReverseSearchModal(self.history), done)

    # Called by Binding("ctrl+C", "stop_streaming_task", ...)
    def action_stop_streaming_task(self):

        if self.stream_task != None and not self.stream_task.done():
            self.stream_task.cancel() # Triggers CancelledError in the task
            self.stream_task = None

        else:
            self._log("Press (Ctrl+Q) to exit.", log_color=3)

    def action_history_prev(self) -> None:
        if not self.history:
            return

        if self.history_index is None:
            self.history_index = len(self.history) - 1
        else:
            self.history_index = max(0, self.history_index - 1)

        self._apply_history_to_input()
        self._update_history_panel()

    def action_history_next(self) -> None:
        if not self.history:
            return

        if self.history_index is None:
            self.history_index = len(self.history)
        elif self.history_index >= len(self.history) - 1:
            self.history_index = len(self.history)
        else:
            self.history_index += 1

        self._apply_history_to_input()
        self._update_history_panel()

    def action_copy(self) -> None:
        ta = self.query_one("#logcontent", TextArea)

        selected = ta.selected_text  # <-- modern Textual API :contentReference[oaicite:2]{index=2}
        if not selected:
            self.notify("No selection")
            return

        pyperclip.copy(selected)     # <-- system clipboard
        self.notify("Copied with pyperclip")

    # endregion

    def run_console(self) -> None:
        """
        Run function for the VulcanAI
        """

        self.run()


    def init_manager(self) -> None:
        if self.iterative:
            from vulcanai.core.manager_iterator import IterativeManager as ConsoleManager
        else:
            from vulcanai.core.manager_plan import PlanManager as ConsoleManager

        # Print in textual terminal:
        # Initializing Manager '<PlanManager/IterativeManager>' ...
        self._log(f"Initializing Manager <bold>'{ConsoleManager.__name__}'</bold>...", log_color=2)

        self.manager = ConsoleManager(model=self.model, k=self.k, logger=self.logger)

        # Print in textual terminal:
        # Manager initialized with model '<model>'
        self._log(f"Manager initialized with model <bold>'{self.model}</bold>'", log_color=2)
        self._update_variables_panel()

    def get_images(self, user_input: str) -> None:
        parts = user_input.split()
        images = []

        for part in parts:
            if part.startswith("--image="):
                images.append(part[len("--image="):])
        return images


def main() -> None:
    parser = argparse.ArgumentParser(description="VulcanAI Interactive Console")
    parser.add_argument(
        "--model", type=str, default="gpt-5-nano",
        help="LLM model to used in the agent (ej: gpt-5-nano, gemini-2.0-flash, etc.)"
    )
    parser.add_argument(
        "--register-from-file", type=str, nargs="*", default=[],
        help="Register tools from a python file (or multiple files)"
    )
    parser.add_argument(
        "--register-from-entry-point", type=str, nargs="*", default=[],
        help="Register tools from a python entry-point (or multiple entry-points)"
    )
    parser.add_argument(
        "-k", type=int, default=7,
        help="Maximum number of tools to pass to the LLM"
    )
    parser.add_argument(
        "-i", "--iterative", action="store_true", default=False,
        help="Enable Iterative Manager (default: off)"
    )

    # TODO. change
    args = parser.parse_args()
    """console = VulcanConsole("turtle_tools", user_context, node)
    console.run()"""
    console = VulcanConsole(model=args.model, k=args.k, iterative=args.iterative)
    if args.register_from_file:
        for file in args.register_from_file:
            console.manager.register_tools_from_file(file)
    if args.register_from_entry_point:
        for entry_point in args.register_from_entry_point:
            console.manager.register_tools_from_entry_points(entry_point)
    console.run_console()


if __name__ == "__main__":
    main()