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
import pyperclip  # To paste the clipboard into the terminal
import sys

from textual import events, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.events import MouseEvent
from textual.markup import escape  # To remove potential errors in textual terminal
from textual.widgets import Input, Static

from vulcanai.console.logger import VulcanAILogger
from vulcanai.console.modal_screens import CheckListModal, RadioListModal, ReverseSearchModal
from vulcanai.console.utils import attach_ros_logger_to_console, common_prefix, SpinnerHook, StreamToTextual

from vulcanai.console.CustomLogTextArea import CustomLogTextArea

import threading


class VulcanConsole(App):

    # CSS Styles
    # Two panels: left (log + input) and right (history + variables)
    #   Right panel: 48 characters length
    #   Left panel: fills remaining space
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
        height: 1fr;
        margin: 1;
    }

    #history {
        width: 100%;
    }
    """

    # Bindings for the console
    BINDINGS = [
        Binding("ctrl+l", "clear_log", "Clear log"),
        Binding("f2", "show_help", "Show help", priority=True),
        Binding("ctrl+r", "reverse_search", "Reverse search"),
        Binding("ctrl+c", "stop_streaming_task", "Stop Streaming"),
        Binding("up", "history_prev", show=False),
        Binding("down", "history_next", show=False),
    ]

    def __init__(self,
                 register_from_file:str = "", tools_from_entrypoints: str = "",
                 user_context: str = "", main_node = None,
                 model: str = "gpt-5-nano", k: int = 7, iterative: bool = False):
        super().__init__() # Textual lib

        # -- Main variables --
        # Manager instance
        self.manager = None
        # Last generated plan
        self.last_plan = None
        # Last generated blackboard state
        self.last_bb = None
        # Spinner hook for LLM requests
        self.hooks = SpinnerHook(self)
        # AI model
        self.model = model
        # 'k' value for top_k tools selection
        self.k = k
        # Iterative mode
        self.iterative = iterative
        # CustomLogTextArea instance
        self.left_pannel = None
        # Logger instance
        self.logger = VulcanAILogger(self)

        # Tools to register from entry points
        self.register_from_file = register_from_file
        self.tools_from_entrypoints = tools_from_entrypoints
        self.user_context = user_context
        # ROS 2 main node
        self.main_node = main_node

        # -- Console extra variables --

        # Commands available in the console
        self.commands = None
        # Tab completion state
        self.tab_matches = []
        # Current index in the tab matches
        self.tab_index = 0

        # Terminal qol
        self.history = []

        # Streaming task control
        self.stream_task = None
        # Suggestion index for RadioListModal
        self.suggestion_index = -1
        self.suggestion_index_changed = threading.Event()


    async def on_mouse_down(self, event: MouseEvent) -> None:
        """
        Function used to paste the string for the user clipboard

        on_mouse_down() function it is called when a mouse button is pressed.
        In this case, only the middle button is used to paste the clipboard content.
        """

        if event.button == 2:  # Middle click
            await self._paste_clipboard()
            event.prevent_default()
            event.stop()

    async def on_mount(self) -> None:
        """
        Function called when the console is mounted.
        """


        self.left_pannel = self.query_one("#logcontent", CustomLogTextArea)

        # Disable terminal input
        self.set_input_enabled(False)
        sys.stdout = StreamToTextual(self, "stdout")
        sys.stderr = StreamToTextual(self, "stderr")

        self.loop = asyncio.get_running_loop()
        asyncio.create_task(self.bootstrap())

    def compose(self) -> ComposeResult:
        """
        Function used to create the console layout.
        It is called at the beggining of the console execution.
        """

        vulcanai_title_slant = \
"""[#56AA08]
 _    __      __                 ___    ____
| |  / /_  __/ /________  ____  /   |  /  _/
| | / / / / / / ___/ __ `/ __ \/ /| |  / /
| |/ / /_/ / / /__/ /_/ / / / / ___ |_/ /
|___/\__,_/_/\___/\__,_/_/ /_/_/  |_/___/[/#56AA08]
"""

        # Textual layout
        with Horizontal():
            # Left
            with Vertical(id="left"):
                # Log Area
                yield CustomLogTextArea(id="logcontent")
                # Input Area
                yield Input(placeholder="> ", id="cmd")

            # Right
            with Vertical(id="right"):
                # Title Area
                yield Static(vulcanai_title_slant, id="history_title")
                # Variable info Area
                yield Static(f" Loading info...", id="variables")
                # History Area
                with VerticalScroll(id="history_scroll"):
                    # NOTE: markup=True so [bold reverse] works
                    yield Static("", id="history", markup=True)

    async def bootstrap(self) -> None:
        """
        Function used to initialize the console manager.
        Print information at runtime execution of a function, without blocking the main thread
        so Textual Log does not freeze.
        """

        def worker() -> None:
            """
            Worker function to run in a separate thread.

            user_input: (str) The user input to process.
            """

            self.init_manager()

            # -- Add the commands --
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

            # Tab matches initialization
            self.tab_matches = []
            self.tab_index = 0

            # -- Spinner controller --
            try:
                self.manager.llm.set_hooks(self.hooks)
            except Exception:
                pass

            # -- Register tools --
            # Default tools

            # File paths tools
            for tool_file_path in self.register_from_file:
                self.manager.register_tools_from_file(tool_file_path)

            # Entry points tools
            if self.tools_from_entrypoints != "":
                self.manager.register_tools_from_entry_points(self.tools_from_entrypoints)

            # Add user context
            self.manager.add_user_context(self.user_context)
            # Add console to blackboard
            self.manager.bb["console"] = self

            # Add the shared node to the console manager blackboard to be used by tools
            if self.main_node != None:
                self.manager.bb["main_node"] = self.main_node
                attach_ros_logger_to_console(self, self.main_node)
            else:
                self.logger.log_warning("No ROS node added")

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker())

        self.is_ready = True
        self.logger.log_console("VulcanAI Interactive Console")
        self.logger.log_console("Type <bold>'exit'</bold> to quit.")

        # Activate the terminal input
        self.set_input_enabled(True)

    async def queriestrap(self, user_input: str="") -> None:
        """
        Function used to handle user requests.
        Print information at runtime execution of a function, without blocking the main thread
        so Textual Log does not freeze.
        """

        def worker(user_input: str="") -> None:
            """
            Worker function to run in a separate thread.

            user_input: (str) The user input to process.
            """
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
                    self.logger.log_msg(f"[error]Error handling request:[/error] {e}")
                    return

                # Store the plan and blackboard state
                self.last_plan = result.get("plan", None)
                self.last_bb = result.get("blackboard", None)

                # Print the backboard state
                bb_ret = result.get('blackboard', None)
                bb_ret = str(bb_ret).replace('<', '\'').replace('>', '\'')
                self.logger.log_console(f"Output of plan: {bb_ret}")

            except KeyboardInterrupt:
                self.logger.log_msg("<yellow>Exiting...</yellow>")
                return
            except EOFError:
                self.logger.log_msg("<yellow>Exiting...</yellow>")
                return

        # This is the main thread, here is where the hook is started for queries
        self.hooks.on_request_start()

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker(user_input))

        # Activate the terminal input
        self.set_input_enabled(True)

    # region Utilities

    def _apply_history_to_input(self) -> None:
        """
        Function used to apply the current history index to the input box.

        Used in the history navigation actions (up/down keys).
        """
        # Get the input box
        cmd_input = self.query_one("#cmd", Input)
        if self.history_index is None or self.history_index == len(self.history):
            cmd_input.value = ""
        else:
            # Set the input value to the current history index
            cmd_input.value = self.history[self.history_index]

        # Focus the input box
        cmd_input.focus()

    def _update_history_panel(self) -> None:
        """
        Function used to update the right panel 'history' widget with
        the current history list of written commands/queries.
        """
        # Get the history widget
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
        """
        Function used to update the right panel 'variables' widget with
        the current variables info (model, k, history_depth).
        """
        text = f" AI model: {self.model}\n K = {self.manager.k}\n history_depth = {self.manager.history_depth}"
        kvalue_widget = self.query_one("#variables", Static)
        kvalue_widget.update(text)

    @work  # Runs in a worker. waiting won't freeze the UI
    async def open_checklist(self, tools_list: list[str], active_tools_num: int) -> None:
        """
        Function used to open a Checklist ModalScreen in the console.
        Used in the /edit_tools command.
        """
        # Create the checklist dialog
        selected = await self.push_screen_wait(CheckListModal(tools_list, active_tools_num))

        if selected is None:
            self.logger.log_warning("Selection cancelled.")
        else:

            # Iterate over all tools and activate/deactivate accordingly
            # to the selection made by the user
            for tool_tmp in tools_list:
                # Remove "- " prefix
                tool = tool_tmp[2:]

                if tool_tmp in selected:
                    # Current tool checbox, activated
                    if self.manager.registry.activate_tool(tool):
                        self.logger.log_console(f"Activated tool <bold>'{tool}'</bold>")
                else:
                    # Current tool checbox, deactivated
                    if self.manager.registry.deactivate_tool(tool):
                        self.logger.log_console(f"Deactivated tool <bold>'{tool}'</bold>")

    @work
    async def open_radiolist(self, option_list: list[str], tool: str = "") -> str:
        """
        Function used to open a RadioList ModalScreen in the console.
        Used in the tool suggestion selection, for default tools.
        """
        # Create the checklist dialog
        selected = await self.push_screen_wait(RadioListModal(option_list))

        if selected is None:
            self.logger.log_tool(f"Suggestion cancelled", tool_name=tool)
            self.suggestion_index = -2
            return

        self.logger.log_tool(f"Selected suggestion: \"{option_list[selected]}\"", tool_name=tool)
        self.suggestion_index = selected
        self.suggestion_index_changed.set() # signal change


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
                "<bold>F3</bold>                - Copy selection area\n"
                "<bold>Ctrl+Q</bold>            - Exit the console\n"
                "<bold>Ctrl+L</bold>            - Clears the console screen\n"
                "<bold>Ctrl+U</bold>            - Clears the entire command line input\n"
                "<bold>Ctrl+K</bold>            - Clears from the cursor to then end of the line\n"
                "<bold>Ctrl+W</bold>            - Delete the word before the cursor\n"
                "<bold>Ctrl+'left/right'</bold> - Move cursor backward/forward by one word\n"
                "<bold>Ctrl+R</bold>            - Reverse search through command history (try typing part of a previous command).\n"
            ]
        )
        self.logger.log_console(table, "console")

    def cmd_tools(self, _) -> None:
        tmp_msg = f"(current index k={self.manager.k})"
        tool_msg = ("_" * len(tmp_msg)) + '\n'
        tool_msg += f"<bold>Available tools:</bold>\n"
        tool_msg += tmp_msg + '\n' + ("‾" * len(tmp_msg)) + '\n'

        for tool in self.manager.registry.tools.values():
            tool_msg += f"- <bold>{tool.name}:</bold> {tool.description}\n"
        self.logger.log_console(tool_msg, "console")

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
            self.logger.log_console(f"Current 'k' is {self.manager.k}")
            return
        if len(args) != 1 or not args[0].isdigit():
            self.logger.log_console(f"Usage: /change_k 'int' - Actual 'k' is {self.manager.k}")
            return

        new_k = int(args[0])
        self.manager.k = new_k
        self.manager.update_k_index(new_k)
        # Update right panel info
        self._update_variables_panel()

    def cmd_history_index(self, args) -> None:
        if len(args) == 0:
            self.logger.log_console(f"Current 'history depth' is {self.manager.history_depth}")
            return
        if len(args) != 1 or not args[0].isdigit():
            self.logger.log_console(f"Usage: /history 'int' - Actual 'history depth' is {self.manager.history_depth}")
            return

        new_hist = int(args[0])
        self.manager.update_history_depth(new_hist)
        # Update right panel info
        self._update_variables_panel()

    def cmd_show_history(self, _) -> None:
        if not self.manager.history:
            self.logger.log_console("No history available.")
            return

        history_msg = \
            "________________\n" + \
            "<bold>Current history:</bold>\n" + \
            "(oldest first)\n" + \
            "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"

        self.logger.log_console(history_msg, "console")
        for i, (user_text, plan_summary) in enumerate(self.manager.history):
            user_req_cmd = user_text.split('\n')
            self.logger.log_msg(f"{i+1}. <bold>[USER] >>> {user_req_cmd[1]}</bold>\n",)
            self.logger.log_msg(f"<bold>Plan summary:</bold> {plan_summary}\n")

    def cmd_clear_history(self, _) -> None:
        # Reset history
        self.history.clear()
        self.history_index = None

        # Empty right panel 'history'
        history_widget = self.query_one("#history", Static)
        history_widget.update("")

        # Add feedback line
        self.logger.log_msg("History cleared.")

    def cmd_plan(self, _) -> None:
        if self.last_plan:
            self.logger.log_console("<bold>Last generated plan:</bold>")
            self.logger.log_console(str(self.last_plan), color="white")
        else:
            self.logger.log_console("No plan has been generated yet.")

    def cmd_rerun(self, _) -> None:
        if self.last_plan:
            self.logger.log_console("Rerunning last plan...")
            result = self.manager.executor.run(self.last_plan, self.manager.bb)
            self.last_bb = result.get("blackboard", None)
            # Parse the blackboard to avoid <...> issues in textual
            last_bb_parsed = str(self.last_bb)
            last_bb_parsed = last_bb_parsed.replace('<', '\'').replace('>', '\'')
            self.logger.log_console(f"Output of rerun: {last_bb_parsed}")
        else:
            self.logger.log_console("No plan to rerun.")

    def cmd_blackboard_state(self, _) -> None:
        if self.last_bb:
            self.logger.log_console("<bold>Lastest blackboard state:</bold>")
            # Parse the blackboard to avoid <...> issues in textual
            last_bb_parsed = str(self.last_bb)
            last_bb_parsed = last_bb_parsed.replace('<', '\'').replace('>', '\'')
            self.logger.log_console(last_bb_parsed)
        else:
            self.logger.log_console("No blackboard available.")

    def cmd_clear(self, _) -> None:
        self.left_pannel.clear_console()

    def cmd_quit(self, _) -> None:
        self.exit()

    # endregion

    # region Logging

    def add_line(self, input: str,
            color: str = "",
            subprocess_flag: bool = False) -> None:
        """
        Function used to write an input in the VulcanAI terminal.
        """
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
            text = f"{color_begin}{line_processed}{color_end}"
            self.left_pannel.append_line(text)

    def replace_line(self, input: str, row: int,
            color: str = "",
            subprocess_flag: bool = False) -> None:
        """
        Function used to replace a line of the VulcanAI terminal.
        """
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
            text = f"{color_begin}{line_processed}{color_end}"
            self.left_pannel.replace_line(text, row)

    def delete_last_line(self):
        """
        Function used to remove the last line in the VulcanAI terminal.
        """
        self.left_pannel.delete_last_row()

    def print_command_prompt(self, cmd: str="", extra_prefix: str="") -> None:
        """
        Function used to print the command prompt with the user command.
        "[USER] >>> 'command_input'
        """
        color_user = "#91DD16"
        self.logger.log_msg(f"{extra_prefix}<bold {color_user}>[USER] >>></bold {color_user}> {cmd}")

    # endregion

    # region Input

    def set_input_enabled(self, enabled: bool) -> None:
        """
        Function used to enable/disable the terminal input box.
        """
        cmd = self.query_one("#cmd", Input)
        cmd.disabled = not enabled
        if enabled:
            self.set_focus(cmd)

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        """
        Function called when the user submits a command in the input box.
        It handles the command, queries and updates the history.
        """
        if not self.is_ready:
            # Console not ready yet
            return

        cmd = event.value.strip()
        if not cmd:
            # Empty command
            return

        cmd_input = self.query_one("#cmd", Input)

        try:
            if event.input.id != "cmd":
                # Not the command input box
                return

            # Get the user input and strip leading/trailing spaces
            user_input = (event.value or "").strip()

            # The the user_input in the history navigation list (used when the up, down keys are pressed)
            self.history.append(user_input)
            self.history_index = len(self.history)
            # Update the right panel 'history' widget
            self._update_history_panel()
            event.input.value = ""
            event.input.focus()

            # Reset tab state
            self.tab_matches = []
            self.tab_index = 0

            if not user_input:
                cmd_input.focus()
                return

            # Echo what the user typed
            self.print_command_prompt(cmd)

            # If it start with '/', just print it as output and stop here
            if user_input.startswith("/"):
                self.handle_command(user_input)
                return

            await asyncio.sleep(0)
            asyncio.create_task(self.queriestrap(user_input))

        except KeyboardInterrupt:
            self.logger.log_msg("<yellow>Exiting...</yellow>")
            return
        except EOFError:
            self.logger.log_msg("<yellow>Exiting...</yellow>")
            return

    def handle_command(self, user_input: str) -> None:
        """
        Function used to handle slash-commands in the console.
        1. If the command is known, execute it.
        2. If the command is unknown, print an error message.
        """
        # Parse as a command
        parts = user_input.split()
        cmd = parts[0].lower()
        args = parts[1:]

        handler = self.commands.get(cmd)
        if handler is None:
            # Only complain for slash-commands
            self.logger.log_console(f"Unknown command: <bold>{cmd}</bold>. Type '/help'.")
        else:
            try:
                handler(args)
            except Exception as e:
                self.logger.log_msg(f"[error]Error: {e!r}[/error]")

    async def _paste_clipboard(self) -> None:
        """
        Function used to paste the clipboard content into the terminal input box.
        """
        # Get the input box
        cmd_input = self.query_one("#cmd", Input)

        try:
            paste_text = pyperclip.paste() or ""
        except Exception as e:
            self.logger.log_msg(f"[error]Clipboard error: {e}[/error]")
            return

        if not paste_text:
            # Nothing to paste
            return

        # Remove endlines
        cut = paste_text.find("\n")
        if cut != -1:
            # Only paste up to the first newline
            paste_text = paste_text[:cut]

        value = cmd_input.value
        cursor = cmd_input.cursor_position

        # Insert text at cursor
        cmd_input.value = value[:cursor] + paste_text + value[cursor:]
        cmd_input.cursor_position = cursor + len(paste_text)
        cmd_input.focus()


    async def on_key(self, event: events.Key) -> None:
        """
        Function used to handle key events in the terminal input box.

        It handles:
            - "tab": Autocomplete command.
            - "ctr+w": Delete the word before the cursor.
            - "ctrl+delete", "escape": Delete the word after the curso.
            - "ctrl+v": Paste the clipboard content into the terminal input box.
        """
        key = event.key
        cmd_input = self.query_one("#cmd", Input)

        # -- tab: Autocomplete ------------------------------------------------
        if key == "tab":
            # Current text
            # (don’t strip right-side spaces; keep user’s spacing)
            raw = cmd_input.value or ""
            # Leading spaces are not part of the command token
            left = len(raw) - len(raw.lstrip())
            value = raw[left:]

            if len(value) <= 0:
                # Nothing typed yet
                return

            # Split into head (first token) and the remainder
            head, *_ = value.split(maxsplit=1)
            # Nothing typed yet: list all commands
            all_cmds = sorted(self.commands) if self.commands else []
            if not all_cmds:
                # No commands available
                return

            # Get matching commands
            self.tab_matches = [c for c in all_cmds if c.startswith(head)] if head else all_cmds
            self.tab_index = 0

            matches = self.tab_matches
            if not matches:
                # No matches, do nothing
                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # If multiple matches, check for a longer common prefix to insert first
            if len(matches) > 1:
                # Find common prefix
                prefix, commands = common_prefix(matches)
                new_value = prefix
                # If the common prefix is just the original head, cycle through options
                self.print_command_prompt(cmd_input.value)
                self.logger.log_console(commands)
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

        # - ctrl+w: Delete before cursor --------------------------------------
        if key == "ctrl+w":
            # Input value and cursor position
            value = cmd_input.value
            cursor = cmd_input.cursor_position
            i = cursor - 1
            # Flag used to detect first non-space character
            first_char = False

            # Iterate backwards to find the start of the previous word
            while i > 0:
                if value[i] == ' ':
                    # First space after a word
                    if first_char:
                        i += 1
                        break
                else:
                    first_char = True
                i -= 1

            # Update input value and cursor position
            cmd_input.value = value[:i] + value[cursor:]
            cmd_input.cursor_position = i
            cmd_input.focus()
            event.prevent_default()
            event.stop()

            return

        # - escape/ctrl+delete: Delete after cursor ---------------------------
        if key in ("ctrl+delete", "escape") :
            # Input value and cursor position
            value = cmd_input.value
            cursor = cmd_input.cursor_position
            i = cursor
            n = len(value)
            count = 0
            # Flag used to detect first non-space character
            first_char = False

            # Iterate forwards to find the end of the next word
            while i < n:
                if(value[i] == ' '):
                    if first_char:
                        break
                else:
                    first_char = True
                i += 1
                count +=1

            # Update input value and cursor position
            cmd_input.value = value[:cursor] + value[i:]
            cmd_input.cursor_position = max(i - count, 0)
            cmd_input.focus()
            event.prevent_default()
            event.stop()

            return

        # -- ctrl+v: Paste clipboard ------------------------------------------
        if key == "ctrl+v":
            # Paste clipboard content
            await self._paste_clipboard()
            event.prevent_default()
            event.stop()

            return

        # Any other keypress resets tab cycle if the prefix changes
        if len(key) == 1 or key in ("backspace", "delete"):
            # Reset tab state
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

    def action_clear_log(self) -> None:
        """
        Function used to clear the console log area.
        Used in the Ctrl + L key binding.

        Binding("ctrl+l", "clear_log", ...),
        """
        self.cmd_clear(_=None)

    def action_show_help(self) -> None:
        """
        Function used to show the help command in the console.
        Used in the F2 key binding.

        Binding("f2", "show_help", ...),
        """
        self.print_command_prompt("/help")
        self.cmd_help(_=None)

    def action_reverse_search(self) -> None:
        """
        Function used to open the Reverse Search ModalScreen in the console.
        Used in the Ctrl + R key binding.

        Binding("ctrl+r", "reverse_search", ...)
        """
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

    def action_stop_streaming_task(self) -> None:
        """
        Function used to stop the current streaming task in the console.
        Used in the Ctrl + C key binding.

        Binding("ctrl+c", "stop_streaming_task", ...),
        """
        if self.stream_task != None and not self.stream_task.done():
            # Cancel the streaming task
            self.stream_task.cancel() # Triggers CancelledError in the task
            self.stream_task = None

        else:
            # No streaming task running, just notify the user
            self.notify("Press (Ctrl+Q) to exit.")

    def action_history_prev(self) -> None:
        """
        Function used to navigate to the previous command in the history.
        Used in the Up Arrow key binding.

        Binding("up", "history_prev", ...),
        """
        if not self.history:
            # No history, do nothing
            return

        if self.history_index is None:
            # The history index class it is not initialized
            self.history_index = len(self.history) - 1
        else:
            self.history_index = max(0, self.history_index - 1)

        self._apply_history_to_input()
        self._update_history_panel()

    def action_history_next(self) -> None:
        """
        Function used to navigate to the next command in the history.
        Used in the Down Arrow key binding.

        Binding("down", "history_next", ...),
        """
        if not self.history:
            # No history, do nothing
            return

        if self.history_index is None:
            self.history_index = len(self.history)
        elif self.history_index >= len(self.history) - 1:
            self.history_index = len(self.history)
        else:
            self.history_index += 1

        self._apply_history_to_input()
        self._update_history_panel()

    # endregion

    def run_console(self) -> None:
        """
        Function used to run VulcanAI.
        """
        self.run()


    def init_manager(self) -> None:
        """
        Function used to initialize VulcanAI Manager.
        """
        if self.iterative:
            from vulcanai.core.manager_iterator import IterativeManager as ConsoleManager
        else:
            from vulcanai.core.manager_plan import PlanManager as ConsoleManager

        self.logger.log_console(f"Initializing Manager <bold>'{ConsoleManager.__name__}'</bold>...")

        self.manager = ConsoleManager(model=self.model, k=self.k, logger=self.logger)

        self.logger.log_console(f"Manager initialized with model <bold>'{self.model}</bold>'")
        # Update right panel info
        self._update_variables_panel()

    def get_images(self, user_input: str) -> None:
        """
        Function used to get the images added by the user in the LLM query.
        """
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


    args = parser.parse_args()

    # console = VulcanConsole(model=args.model, k=args.k, iterative=args.iterative)
    # if args.register_from_file:
    #     for file in args.register_from_file:
    #         console.manager.register_tools_from_file(file)
    # if args.register_from_entry_point:
    #     for entry_point in args.register_from_entry_point:
    #         console.manager.register_tools_from_entry_points(entry_point)
    # console.run_console()

    console = VulcanConsole(register_from_file=args.register_from_file,
                            tools_from_entrypoints=args.register_from_entry_point,
                            model=args.model, k=args.k, iterative=args.iterative)
    console.run_console()


if __name__ == "__main__":
    main()