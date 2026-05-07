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
import sys
import threading

import pyperclip  # To paste the clipboard into the terminal
from textual import constants as textual_constants
from textual import events, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.events import MouseEvent
from textual.markup import escape  # To remove potential errors in textual terminal
from textual.widgets import Input, Static

from vulcanai.console.logger import VulcanAILogger
from vulcanai.console.modal_screens import CheckListModal, RadioListModal, ReverseSearchModal
from vulcanai.console.terminal_session import TerminalSession
from vulcanai.console.utils import (
    SpinnerHook,
    StreamToTextual,
    attach_ros_logger_to_console,
    common_prefix,
)
from vulcanai.console.widget_custom_log_text_area import CustomLogTextArea
from vulcanai.console.widget_spinner import SpinnerStatus


class TextualLogSink:
    """A default console that prints to standard output."""

    def __init__(self, textual_console) -> None:
        self.console = textual_console

    def write(self, msg: str, color: str = "") -> None:
        # Logger calls can come from worker threads
        # Route UI writes to Textual app thread
        app_thread_id = getattr(self.console, "_thread_id", None)
        if app_thread_id is not None and threading.get_ident() != app_thread_id:
            self.console.call_from_thread(self.console.add_line, msg, color)
            return

        self.console.add_line(msg, color)


class VulcanConsole(App):
    DEFAULT_CMD_PLACEHOLDER = "> "
    # Extra info when a streaming process is active
    STREAM_CMD_PLACEHOLDER = "Press Ctrl+C to stop stream | > "
    # CSS Styles
    # Two panels: left (log + input) and right (history + variables)
    #   Right panel: 48 characters length
    #   Left panel: fills remaining space

    CSS = """
    Screen {
        layout: horizontal;
        overflow: hidden hidden;
    }

    #root {
        width: 100%;
        height: 100%;
        overflow: hidden hidden;
    }

    #left {
        width: 1fr;
        layout: vertical;
        overflow: hidden hidden;
    }

    #right {
        width: 48;
        layout: vertical;
        border: tall #56AA08;
        padding: 0;
        overflow: hidden hidden;
    }

    #logcontent {
        height: 1fr;
        min-height: 1;
        border: tall #333333;
        scrollbar-size-vertical: 1;
        scrollbar-size-horizontal: 1;
        color: #ffffff;
    }

    #streamcontent {
        height: 0;
        min-height: 0;
        border: solid #56AA08;
        display: none;
        overflow: auto auto;
        scrollbar-size-vertical: 1;
        scrollbar-size-horizontal: 1;
    }

    #llm_spinner {
        height: 0;
        display: none;
        content-align: left middle;
        padding-left: 2;
    }

    #cmd {
        dock: bottom;
        color: #ffffff;
    }

    #history_title {
        content-align: center middle;
        margin: 0;
        padding: 0;
        color: #ffffff;
    }

    #history_scroll {
        height: 1fr;
        margin: 1;
        scrollbar-size-vertical: 0;
        scrollbar-size-horizontal: 0;
    }

    #history {
        width: 100%;
    }

    #variables {
        color: #ffffff;
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

    def __init__(
        self,
        model: str = "gpt-5-nano",
        k: int = 7,
        iterative: bool = False,
        register_from_file: str = "",
        tools_from_entrypoints: str = "",
        user_context: str = "",
        main_node=None,
        default_tools: bool = True,
        debug: bool = False,
    ):
        # Used to set the same textual colors in a docker container
        os.environ.setdefault("COLORTERM", "truecolor")
        textual_constants.COLOR_SYSTEM = "truecolor"

        # Keep Hugging Face download progress bars out of redirected Textual stdout/stderr.
        os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")

        super().__init__()  # Textual lib

        # -- Main variables --
        # Manager instance
        self.manager = None
        # List of generated plans
        self.plans_list = []
        # Last generated blackboard state
        self.last_bb = None
        # Spinner hook for LLM requests
        self.spinner_status = None
        self.hooks = None
        # AI model
        self.model = model
        # 'k' value for top_k tools selection
        self.k = k
        # Flag to indicate if default tools should be enabled
        self.default_tools = default_tools
        # Iterative mode
        self.iterative = iterative
        # Enable debug-only logs
        self.debug_flag = debug
        # CustomLogTextArea instance
        self.main_pannel = None
        # Subprocess output panel
        self.stream_pannel = None
        # Logger instance
        self.logger = VulcanAILogger.default()
        self.logger.set_textualizer_console(TextualLogSink(self))

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
        # Number of active streaming processes routing logs to subprocess panel
        self._route_logs_to_stream_panel = 0
        # Visibility state for stream panel routing
        self._stream_panel_visible = False
        # Buffered main-log lines emitted while stream panel is open and waiting
        # for user Ctrl+C closure
        self._deferred_main_lines_after_stream: list[str] = []
        # Suggestion index for RadioListModal
        self.suggestion_index = -1
        self.suggestion_index_changed = threading.Event()

        self._gnome_profile_schema: str | None = None
        self._gnome_scrollbar_policy_backup: str | None = None

    async def on_mouse_down(self, event: MouseEvent) -> None:
        """
        Function used to paste the string for the user clipboard

        on_mouse_down() function is called when a mouse button is pressed.
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

        self.main_pannel = self.query_one("#logcontent", CustomLogTextArea)
        self.stream_pannel = self.query_one("#streamcontent", CustomLogTextArea)
        self.spinner_status = self.query_one("#llm_spinner", SpinnerStatus)
        self.hooks = SpinnerHook(self.spinner_status)

        # Disable terminal input
        self.set_input_enabled(False)
        sys.stdout = StreamToTextual(self, "stdout")
        sys.stderr = StreamToTextual(self, "stderr")

        if self.main_node is not None or self.default_tools:
            attach_ros_logger_to_console(self)

        self.loop = asyncio.get_running_loop()
        asyncio.create_task(self.bootstrap())

    def compose(self) -> ComposeResult:
        """
        Function used to create the console layout.
        It is called at the beggining of the console execution.
        """

        color_tmp = VulcanAILogger.vulcanai_theme["vulcanai"]

        vulcanai_title_slant = f"""[{color_tmp}]
 _    __      __                 ___    ____
| |  / /_  __/ /________  ____  /   |  /  _/
| | / / / / / / ___/ __ `/ __ \/ /| |  / /
| |/ / /_/ / / /__/ /_/ / / / / ___ |_/ /
|___/\__,_/_/\___/\__,_/_/ /_/_/  |_/___/[/{color_tmp}]
"""

        # Textual layout
        with Horizontal():
            # Left
            with Vertical(id="left"):
                # Log Area
                logcontent = CustomLogTextArea(id="logcontent")
                yield logcontent
                # Subprocess stream area (hidden by default)
                # Kept below the main log to avoid visually pushing main logs upward
                streamcontent = CustomLogTextArea(id="streamcontent")
                yield streamcontent
                # Spinner Area
                yield SpinnerStatus(logcontent, id="llm_spinner")
                # Input Area
                yield Input(placeholder=self.DEFAULT_CMD_PLACEHOLDER, id="cmd")

            # Right
            with Vertical(id="right"):
                # Title Area
                yield Static(vulcanai_title_slant, id="history_title")
                # Variable info Area
                yield Static(" Loading info...", id="variables")
                # History Area
                with VerticalScroll(id="history_scroll"):
                    # NOTE: markup=True so [bold reverse] works
                    yield Static("", id="history", markup=True)

    async def bootstrap(self) -> None:
        """
        Function used to initialize the console manager asynchronously.
        Blocking operations (file I/O) run in executor, non-blocking in event loop.
        """

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self.init_manager)

        # -- Add the commands (non-blocking, runs in event loop) --
        self.commands = {
            "/help": self.cmd_help,
            "/tools": self.cmd_tools,
            "/edit_tools": self.cmd_edit_tools,
            "/change_k": self.cmd_change_k,
            "/history": self.cmd_history_index,
            "/debug": self.cmd_debug,
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

        # Load a default ROS 2 node if default tools are enabled but no node is provided
        if self.default_tools and self.main_node is None:
            try:
                from vulcanai.tools.default_tools import ROS2DefaultToolNode

                self.main_node = ROS2DefaultToolNode()
            except ImportError:
                self.logger.log_console("Unable to load ROS 2 default node for default tools.")

        # -- Register tools (file I/O - run in executor) --
        # File paths tools
        for tool_file_path in self.register_from_file:
            await loop.run_in_executor(None, self.manager.register_tools_from_file, tool_file_path)

        # Entry points tools
        for ep in self.tools_from_entrypoints:
            await loop.run_in_executor(None, self.manager.register_tools_from_entry_points, ep)

        # Add user context (non-blocking)
        self.manager.add_user_context(self.user_context)
        # Add console to blackboard
        self.manager.bb["console"] = self

        # Add the shared node to the console manager blackboard
        if self.main_node is not None:
            self.manager.bb["main_node"] = self.main_node

        self.is_ready = True
        self.logger.log_console("VulcanAI Interactive Console")
        self.logger.log_console("Clipboard: select text and press F4 to copy. Use Ctrl+V or middle-click to paste.")
        self.logger.log_console("Use <bold>'/exit'</bold> or press <bold>'Ctrl+Q'</bold> to quit.")

        # Anchor the main log at the bottom after all startup output has been queued,
        # so the user sees the latest line (and input prompt) without manual scrolling.
        if self.main_pannel is not None:
            self.main_pannel.scroll_end(animate=False, immediate=True, x_axis=False)
            self.call_after_refresh(self.main_pannel.scroll_end, animate=False, immediate=True, x_axis=False)
            self.call_later(self.main_pannel.scroll_end, animate=False, immediate=True, x_axis=False)

        # Activate the terminal input
        self.set_input_enabled(True)

    async def queriestrap(self, user_input: str = "") -> None:
        """
        Function used to handle user requests.
        Print information at runtime execution of a function, without blocking the main thread
        so Textual Log does not freeze.
        """

        def worker(user_input: str = "") -> None:
            """
            Worker function to run in a separate thread.

            user_input: (str) The user input to process.
            """
            # Disable terminal input
            self.set_input_enabled(False)

            try:
                bb_before_ids = {k: id(v) for k, v in self.manager.bb.items()}
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
                self.plans_list.append(result.get("plan", None))
                self.last_bb = result.get("blackboard", None)

                # Print the blackboard state
                bb_ret = result.get("blackboard", None)
                if bb_ret and self.debug_flag:
                    updated_keys = self._updated_blackboard_keys(bb_before_ids, bb_ret)
                    bb_ret_str = "No new output generated by executed tools."
                    if updated_keys:
                        bb_ret_str = self._format_blackboard_subset(bb_ret, updated_keys)
                    self.logger.log_console(f"Output of plan: {bb_ret_str}")

            except KeyboardInterrupt:
                if self.stream_task is None:
                    self.logger.log_msg("<yellow>Exiting...</yellow>")
                else:
                    self.stream_task.cancel()  # triggers CancelledError in the task
                    self.stream_task = None
            except EOFError:
                self.logger.log_msg("<yellow>Exiting...</yellow>")
                return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker(user_input))

        # Activate the terminal input
        self.set_input_enabled(True)

    # region Utilities

    def _updated_blackboard_keys(self, before_ids: dict[str, int], bb_after) -> list[str]:
        """
        Return blackboard keys that were created or replaced during the latest execution.
        """
        updated_keys = []
        for key, value in bb_after.items():
            if key not in before_ids or before_ids[key] != id(value):
                updated_keys.append(key)
        return updated_keys

    def _format_blackboard_subset(self, bb, keys: list[str]) -> str:
        """
        Format a subset of blackboard keys for debug logging in Textual console.
        """
        if not keys:
            return "{}"

        subset = {k: bb.get(k) for k in keys if k in bb}
        if not subset:
            return "{}"

        # Keep output on a single line while avoiding Textual tag parsing issues.
        return repr(subset).replace("<", "'").replace(">", "'")

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
        cmd_input.cursor_position = len(cmd_input.value)
        cmd_input.focus()

    def _update_history_panel(self) -> None:
        """
        Function used to update the right panel 'history' widget with
        the current history list of written commands/queries.
        """
        # Get the history widget
        history_widget = self.query_one("#history", Static)

        plan_count = 0
        lines = []
        for i, cmd in enumerate(self.history):
            cmd_esc = escape(cmd)
            prefix = ""
            tmp_color = ""
            if len(cmd_esc) > 0 and cmd_esc[0] != "/":
                tmp_color = VulcanAILogger.vulcanai_theme["vulcanai"]
                prefix = f" [{tmp_color}][Plan {plan_count}][/{tmp_color}]\n"
                plan_count += 1
            else:
                tmp_color = VulcanAILogger.vulcanai_theme["console"]

            text = f"{prefix} [{tmp_color}]{i + 1}:[/{tmp_color}] {escape(cmd)}"
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

        text = (
            f" AI model: {self.model.replace('ollama-', '')}\n K = {self.manager.k}\n"
            f" history_depth = {self.manager.history_depth}"
        )
        kvalue_widget = self.query_one("#variables", Static)
        kvalue_widget.update(text)

    @work  # Runs in a worker. waiting won't freeze the UI
    async def open_checklist(self, grouped: list, active_set: set) -> None:
        """
        Function used to open a Checklist ModalScreen in the console.
        Used in the /edit_tools command.
        """
        # Create the checklist dialog
        selected = await self.push_screen_wait(CheckListModal(grouped, active_set))

        if selected is None:
            self.logger.log_msg("<yellow>Selection cancelled.</yellow>")
        else:
            selected_set = set(selected)
            # Collect all non-help tool names
            all_tool_names = set(self.manager.registry.tools.keys()) | set(
                self.manager.registry.deactivated_tools.keys()
            )
            all_tool_names.discard("help")

            for tool_name in all_tool_names:
                if tool_name in selected_set:
                    if self.manager.registry.activate_tool(tool_name):
                        self.logger.log_console(f"Activated tool <bold>'{tool_name}'</bold>")
                else:
                    if self.manager.registry.deactivate_tool(tool_name):
                        self.logger.log_console(f"Deactivated tool <bold>'{tool_name}'</bold>")

    @work
    async def open_radiolist(
        self, option_list: list[str], tool: str = "", category: str = "", input_string: str = ""
    ) -> str:
        """
        Function used to open a RadioList ModalScreen in the console.
        Used in the tool suggestion selection, for default tools.
        """
        # Create the checklist dialog
        selected = await self.push_screen_wait(RadioListModal(option_list, category, input_string))

        if selected is None:
            self.logger.log_tool("Suggestion cancelled", tool_name=tool)
            self.suggestion_index = -2
            self.suggestion_index_changed.set()
            return

        self.logger.log_tool(f'Selected suggestion: "{option_list[selected]}"', tool_name=tool)
        self.suggestion_index = selected
        self.suggestion_index_changed.set()  # signal change

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
                "/<bold>change_k 'int'</bold> - Change the 'k' value for the top_k algorithm selection"
                " or show the current value if no 'int' is provided\n"
                "/<bold>history 'int'</bold>  - Change the history depth or show the current value if no"
                " 'int' is provided\n"
                "/<bold>debug 'bool'</bold>   - Set debug mode to true/false, or show current value if no"
                " bool is provided\n"
                "/<bold>show_history</bold>   - Show the current history\n"
                "/<bold>clear_history</bold>  - Clear the history\n"
                "/<bold>plan</bold>           - Show the last generated plan\n"
                "/<bold>rerun 'int'</bold>    - Rerun the last plan or the specified plan by index\n"
                "/<bold>bb</bold>             - Show the last blackboard state\n"
                "/<bold>clear</bold>          - Clears the console screen\n"
                "/<bold>exit</bold>           - Exit the console\n"
                "<bold>Query any other text</bold> to process it with the LLM and execute the plan generated.\n\n"
                "Add --image='path' to include images in the query. It can be used multiple times to add"
                " more images.\n"
                "Example: 'user_prompt' --image=/path/to/image1 --image=/path/to/image2'\n"
                "___________________\n"
                "<bold>Available keybinds:</bold>\n"
                "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
                "<bold>F2</bold>                - Show this help message\n"
                "<bold>F4</bold>                - Copy selection area\n"
                "<bold>Ctrl+Q</bold>            - Exit the console\n"
                "<bold>Ctrl+L</bold>            - Clears the console screen\n"
                "<bold>Ctrl+U</bold>            - Clears the entire command line input\n"
                "<bold>Ctrl+K</bold>            - Clears from the cursor to then end of the line\n"
                "<bold>Ctrl+W</bold>            - Delete the word before the cursor\n"
                "<bold>Ctrl+'left/right'</bold> - Move cursor backward/forward by one word\n"
                "<bold>Ctrl+R</bold>            - Reverse search through command history (try typing part of a"
                " previous command).\n"
            ]
        )
        self.logger.log_console(table, "console")

    def cmd_tools(self, _) -> None:
        def get_tool_summary(tool) -> str:
            return getattr(tool, "tool_description", None) or tool.description

        tmp_msg = f"(current index k={self.manager.k})"
        tool_msg = ("_" * len(tmp_msg)) + "\n"
        tool_msg += "<bold>Available tools:</bold>\n"
        tool_msg += tmp_msg + "\n" + ("‾" * len(tmp_msg)) + "\n"

        tool_names = [name for name in self.manager.registry.tools.keys() if name != "help"]
        grouped = self.manager.registry.group_tool_names(tool_names)

        for entry_name, subtools in grouped:
            if subtools is None:
                tool = self.manager.registry.tools[entry_name]
                tool_msg += f"<bold>{tool.name}:</bold> {get_tool_summary(tool)}\n"
            else:
                tool_msg += f"<bold>{entry_name}:</bold>\n"
                for subtool in subtools:
                    full_name = f"{entry_name}_{subtool}"
                    tool = self.manager.registry.tools[full_name]
                    tool_msg += f"  - <bold>{subtool}:</bold> {get_tool_summary(tool)}\n"

        if "help" in self.manager.registry.tools:
            help_tool = self.manager.registry.tools["help"]
            tool_msg += f"- <bold>{help_tool.name}:</bold> {get_tool_summary(help_tool)}\n"

        self.logger.log_console(tool_msg, "console")

    def cmd_edit_tools(self, _) -> None:
        all_names = sorted(
            n
            for n in list(self.manager.registry.tools.keys()) + list(self.manager.registry.deactivated_tools.keys())
            if n != "help"
        )
        active_set = set(self.manager.registry.tools.keys())
        grouped = self.manager.registry.group_tool_names(all_names)
        self.open_checklist(grouped, active_set)

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

    def cmd_debug(self, args) -> None:
        if len(args) == 0:
            self.logger.log_console(f"Current 'debug' is {self.debug_flag}")
            return

        if len(args) != 1:
            self.logger.log_console(f"Usage: /debug 'bool' - Actual 'debug' is {self.debug_flag}")
            return

        value = args[0].strip().lower()
        if value not in ("true", "false"):
            self.logger.log_console(f"Usage: /debug 'bool' - Actual 'debug' is {self.debug_flag}")
            return

        self.debug_flag = value == "true"
        self.logger.log_console(f"Set 'debug' to {self.debug_flag}")

    def cmd_show_history(self, _) -> None:
        if not self.manager.history:
            self.logger.log_console("No history available.")
            return

        history_msg = (
            "________________\n" + "<bold>Current history:</bold>\n" + "(oldest first)\n" + "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
        )

        self.logger.log_console(history_msg, "console")
        for i, (user_text, plan_summary) in enumerate(self.manager.history):
            user_req_cmd = user_text.split("\n")
            self.logger.log_msg(
                f"{i + 1}. <bold>[USER] >>> {user_req_cmd[1]}</bold>\n",
            )
            self.logger.log_msg(f"<bold>Plan summary:</bold> {plan_summary}\n")

    def cmd_clear_history(self, _) -> None:
        # Reset history
        self.history.clear()
        self.history_index = None

        # Empty right panel 'history'
        history_widget = self.query_one("#history", Static)
        history_widget.update("")
        # Clear the list of plans
        self.plans_list.clear()

        # Add feedback line
        self.logger.log_msg("History cleared.")

    def cmd_plan(self, _) -> None:
        if len(self.plans_list) > 0:
            self.logger.log_console("<bold>Last generated plan:</bold>")
            self.logger.log_console(str(self.plans_list[-1]), color="white")
        else:
            self.logger.log_console("No plan has been generated yet.")

    def cmd_rerun(self, args) -> None:
        self._rerun_worker(args)  # start worker (dont await)

    @work(thread=True)
    async def _rerun_worker(self, args) -> None:
        """
        Worker function used to run the command "rerun".
        It has to be a worker(thead=True) because the call 'self.manager.executor.run'
        might have a "call_from_thread" in the tool executed,
        and it is only valid in non Textual app Threads (separated Thread).

        @work runs on the app's event loop (app thread) and is for async, non-blocking code.
        @work(thread=True) runs in a separate OS thread and is for blocking.

        e.g.:
        'move_turtle' tool contains a 'call_from_thread'
        'ros2_topic' tool does not contains a 'call_from_thread'
        """
        selected_plan = 0
        if len(args) == 0:
            # No index specified. Last plan selected
            selected_plan = len(self.plans_list) - 1
        elif len(args) != 1 or not args[0].isdigit():
            self.logger.log_console("Usage: /rerun 'int'")
            return
        else:
            selected_plan = int(args[0])
            if selected_plan < -1:
                self.logger.log_console("Usage: /rerun 'int' [int > -1].")
                return

        if not self.plans_list:
            self.logger.log_console("No plan to rerun.")
            return
        elif selected_plan >= len(self.plans_list):
            self.logger.log_console("Selected Plan index do not exists. selected_plan >= len(executed_plans).")
            return

        self.logger.log_console(f"Rerunning {selected_plan}-th plan...")

        # Execute the plan
        result = self.manager.executor.run(self.plans_list[selected_plan], self.manager.bb)

        last_bb = result.get("blackboard", None)
        last_bb_parsed = str(last_bb).replace("<", "'").replace(">", "'")

        # UI updates must happen on the app thread:
        def apply_result():
            self.last_bb = last_bb
            if self.debug_flag:
                self.logger.log_console(f"Output of rerun: {last_bb_parsed}")

        self.call_from_thread(apply_result)

    def cmd_blackboard_state(self, _) -> None:
        if self.last_bb:
            self.logger.log_console("<bold>Lastest blackboard state:</bold>")
            # Parse the blackboard to avoid <...> issues in textual
            last_bb_parsed = str(self.last_bb)
            last_bb_parsed = last_bb_parsed.replace("<", "'").replace(">", "'")
            self.logger.log_console(last_bb_parsed)
        else:
            self.logger.log_console("No blackboard available.")

    def cmd_clear(self, _) -> None:
        if self.stream_pannel is not None:
            self.stream_pannel.clear_console()
        self.main_pannel.clear_console()

    def cmd_quit(self, _) -> None:
        self.exit()

    # endregion

    # region Logging

    def show_subprocess_panel(self, show_notice: bool = True) -> None:
        """
        Show the dedicated subprocess output panel at the top of the main panel.
        """

        if self.stream_pannel is None:
            return

        follow_main_output = False
        if self.main_pannel is not None:
            follow_main_output = self.main_pannel.is_near_vertical_scroll_end(tolerance=2)

        if show_notice:
            self.logger.log_tool(
                "Streaming terminal opened. <bold>Press Ctrl+C to stop</bold> the running process.", color="tool"
            )

        self.stream_pannel.clear_console()
        self.stream_pannel.display = True
        self.stream_pannel.styles.height = 12
        self.query_one("#left", Vertical).refresh(layout=True)
        self.stream_pannel.refresh(layout=True)
        self.refresh(layout=True)
        self._stream_panel_visible = True
        cmd = self.query_one("#cmd", Input)
        cmd.placeholder = self.STREAM_CMD_PLACEHOLDER
        if follow_main_output and self.main_pannel is not None:
            self.main_pannel.scroll_end(animate=False, immediate=True, x_axis=False)
            self.call_after_refresh(self.main_pannel.scroll_end, animate=False, immediate=True, x_axis=False)
            self.call_later(self.main_pannel.scroll_end, animate=False, immediate=True, x_axis=False)

    def change_route_logs(self, value: bool = False) -> None:
        """
        Manage logger sink output routing to stream panel.

        value = True  -> Increment active streaming routes.
        value = False -> Decrement active streaming routes.
        """

        if value:
            self._route_logs_to_stream_panel += 1
            if self.stream_pannel is not None and not self._stream_panel_visible:
                # Routing is active -> ensure stream panel is visible
                self.show_subprocess_panel(show_notice=False)
        else:
            self._route_logs_to_stream_panel = max(0, self._route_logs_to_stream_panel - 1)
            if not self._is_stream_task_active():
                # No active stream: clear stale route counter/state, but keep the
                # stream panel visible until user explicitly closes it with "Ctrl + C"
                self._route_logs_to_stream_panel = 0
                self.stream_task = None

    def _is_stream_task_active(self) -> bool:
        """
        Return True if a streaming task/future is currently active
        """

        if self.stream_task is None:
            return False
        try:
            return not self.stream_task.done()
        except Exception:
            try:
                return not self.stream_task.cancelled()
            except Exception:
                # Unknown task-like object
                return True

    def reset_stream_state(self) -> None:
        """
        Reset stream routing and panel visibility
        """

        self._route_logs_to_stream_panel = 0
        self.stream_task = None
        if self._stream_panel_visible:
            self.hide_subprocess_panel()

    def should_defer_line_until_stream_panel_close(self, line: str) -> bool:
        """
        Return True when a line should be deferred until the stream panel is closed
        """

        return any(marker in line for marker in ("[TOOL", "[EXECUTOR]", "[ROS]", "Output of plan:"))

    def write_deferred_main_output(self) -> None:
        """
        Flush buffered main-log lines captured while stream panel was open.
        """

        if not self._deferred_main_lines_after_stream or self.main_pannel is None:
            return

        follow_main_output = self.main_pannel.is_near_vertical_scroll_end(tolerance=2)
        pending = self._deferred_main_lines_after_stream
        self._deferred_main_lines_after_stream = []

        for text in pending:
            if not self.main_pannel.append_line(text, force_follow_output=follow_main_output):
                self.logger.log_console("Warning: Trying to add an empty deferred line.")

    def hide_subprocess_panel(self) -> None:
        """
        Hide the subprocess output panel and return space to the main log panel.
        """

        if self.stream_pannel is None:
            return

        follow_main_output = False
        if self.main_pannel is not None:
            follow_main_output = self.main_pannel.is_near_vertical_scroll_end(tolerance=2)

        self.stream_pannel.display = False
        self.stream_pannel.styles.height = 0
        self.stream_pannel.refresh(layout=True)
        self.refresh(layout=True)
        self._stream_panel_visible = False
        self.write_deferred_main_output()
        cmd = self.query_one("#cmd", Input)
        cmd.placeholder = self.DEFAULT_CMD_PLACEHOLDER
        if follow_main_output and self.main_pannel is not None:
            self.main_pannel.scroll_end(animate=False, immediate=True, x_axis=False)
            self.call_after_refresh(self.main_pannel.scroll_end, animate=False, immediate=True, x_axis=False)

    def add_subprocess_line(self, input: str) -> None:
        """
        Write output into the dedicated subprocess panel.
        """
        if self.stream_pannel is None:
            self.add_line(input)
            return

        # Reopen only if there is an active stream/routing context
        if not self._stream_panel_visible:
            # After user send sigint "Ctrl + C" reset both flags
            # Late subprocess lines should go to main log
            # without reopening the stream panel
            should_reopen = self._route_logs_to_stream_panel > 0 or self._is_stream_task_active()
            if should_reopen:
                self.show_subprocess_panel(show_notice=False)

            if not self._stream_panel_visible:
                self.add_line(input)
                return

        lines = input.splitlines()
        for line in lines:
            if not self.stream_pannel.append_line(line):
                self.logger.log_console("Warning: Trying to add an empty subprocess line.")

    def add_line(self, input: str, color: str = "", subprocess_flag: bool = False) -> None:
        """
        Function used to write an input in the VulcanAI terminal.
        """

        app_thread_id = getattr(self, "_thread_id", None)
        if app_thread_id is not None and threading.get_ident() != app_thread_id:
            self.call_from_thread(self.add_line, input, color, subprocess_flag)
            return

        # Split incoming text into individual lines
        lines = input.splitlines()

        color_begin = ""
        color_end = ""
        if color != "":
            color_begin = f"<{color}>"
            color_end = f"</{color}>"

        target_panel = self.main_pannel
        if self._route_logs_to_stream_panel > 0 and self.stream_pannel is not None:
            if not self._stream_panel_visible:
                # If somehow the streaming panel is closed without the user
                # reopen it before routing output to the stream area
                self.show_subprocess_panel(show_notice=False)
            if self._stream_panel_visible:
                target_panel = self.stream_pannel

        force_follow_main = False
        if target_panel is self.main_pannel:
            force_follow_main = self.main_pannel.is_near_vertical_scroll_end(tolerance=2)

        # Append each line; deque automatically truncates old ones
        for line in lines:
            line_processed = line
            if subprocess_flag:
                line_processed = escape(line)
            text = f"{color_begin}{line_processed}{color_end}"

            # Keep tool/executor output hidden in main panel while stream panel remains
            # visible after stream completion. Flushes on Ctrl+C panel close
            if (
                target_panel is self.main_pannel
                and self._stream_panel_visible
                and not self._is_stream_task_active()
                and self._route_logs_to_stream_panel == 0
                and self.should_defer_line_until_stream_panel_close(line_processed)
            ):
                self._deferred_main_lines_after_stream.append(text)
                continue

            if not target_panel.append_line(text, force_follow_output=force_follow_main):
                self.logger.log_console("Warning: Trying to add an empty line.")

    def delete_last_line(self):
        """
        Function used to remove the last line in the VulcanAI terminal.
        """
        self.main_pannel.delete_last_row()

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

        # Strip '<' and '>' so inputs like '</chatter>' (ROS topic names) are not
        # consumed by the Textual markup parser, which would drop the token from
        # the echoed line and raise "'<name>' is not a valid color" errors when
        # the same text flows through manager/plan logs.
        cmd = event.value.strip().replace("<", "").replace(">", "")
        if not cmd:
            # Empty command
            return

        cmd_input = self.query_one("#cmd", Input)

        try:
            if event.input.id != "cmd":
                # Not the command input box
                return

            # Already sanitized above; reuse to keep echo, history, and query aligned
            user_input = cmd

            # Defensive recovery: if no stream is active but routing stayed enabled,
            # clear only routing (keep panel visible until Ctrl+C)
            if (not self._is_stream_task_active()) and self._route_logs_to_stream_panel > 0:
                self._route_logs_to_stream_panel = 0

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
            self.logger.log_user(cmd)

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
                self.logger.log_user(cmd_input.value)
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
                if value[i] == " ":
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
        if key in ("ctrl+delete", "escape"):
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
                if value[i] == " ":
                    if first_char:
                        break
                else:
                    first_char = True
                i += 1
                count += 1

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
        With this variable the user can finish the execution of the
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
        self.logger.log_user("/help")
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

        if self._is_stream_task_active():
            # Cancel the active streaming task/future.
            try:
                self.stream_task.cancel()  # Triggers CancelledError / cancelled flag
            except Exception:
                pass
            self.reset_stream_state()

        elif self.stream_pannel is not None and (self._stream_panel_visible or self._route_logs_to_stream_panel > 0):
            # No active stream, but stale stream UI/routing is visible
            self.reset_stream_state()

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

        session = TerminalSession()
        with session:
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

        self.manager = ConsoleManager(model=self.model, k=self.k, logger=self.logger, default_tools=self.default_tools)

        self.logger.log_console(f"Manager initialized with model <bold>'{self.model.replace('ollama-', '')}</bold>'")
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
                images.append(part[len("--image=") :])
        return images


def main() -> None:
    parser = argparse.ArgumentParser(description="VulcanAI Interactive Console")
    parser.add_argument(
        "--model",
        type=str,
        default="gpt-5-nano",
        help="LLM model to used in the agent (ej: gpt-5-nano, gemini-2.0-flash, etc.)",
    )
    parser.add_argument(
        "--register-from-file",
        type=str,
        nargs="*",
        default=[],
        help="Register tools from a python file (or multiple files)",
    )
    parser.add_argument(
        "--register-from-entry-point",
        type=str,
        nargs="*",
        default=[],
        help="Register tools from a python entry-point (or multiple entry-points)",
    )
    parser.add_argument("-k", type=int, default=7, help="Maximum number of tools to pass to the LLM")
    parser.add_argument(
        "-i", "--iterative", action="store_true", default=False, help="Enable Iterative Manager (default: off)"
    )
    parser.add_argument("--debug", action="store_true", default=False, help="Enable debug logs (default: off)")

    args = parser.parse_args()

    console = VulcanConsole(
        register_from_file=args.register_from_file,
        tools_from_entrypoints=args.register_from_entry_point,
        model=args.model,
        k=args.k,
        iterative=args.iterative,
        debug=args.debug,
    )
    console.run_console()


if __name__ == "__main__":
    main()
