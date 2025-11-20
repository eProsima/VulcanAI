from __future__ import annotations

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

import sys
import argparse

from textual.app import App, ComposeResult
from textual.widgets import Input, Static, Checkbox, Button, Label
from textual import events
from textual.containers import VerticalScroll, Horizontal, Vertical, Container
from textual.binding import Binding
# sipnner
from textual.timer import Timer
# checkbox
from textual.screen import ModalScreen
# non-blocking executions to print in textual terminal
from textual import work
import asyncio
# library used to paste the clipboard into the terminal
import pyperclip


class StreamToTextual:
    """
    Class used to redirect the stdout/stderr streams in the textual terminal
    """

    def __init__(self, app, stream_name: str = "stdout"):
        self.app = app
        self._real_stream = getattr(sys, stream_name)

    def write(self, data: str):
        if not data:
            return

        # optional: still write to real stdout/stderr
        self._real_stream.write(data)
        self._real_stream.flush()

        if data.strip():
            # Ensure update happens on the app thread
            self.app.call_from_thread(self.app.append_log_text, data)

    def flush(self):
        self._real_stream.flush()

class SpinnerHook:
    """
    Single entrant spinner controller for console.
    - Starts the spinner on the LLM request.
    - Stops the spinner when LLM request is over.
    """

    def __init__(self, console):

        self.console = console

        # Spinner states
        self.spinner_timer: Timer | None = None
        self.spinner_frames = ["⠋","⠙","⠹","⠸","⠼","⠴","⠦","⠧","⠇","⠏"]
        self.spinner_frame_index = 0
        self.spinner_line_index: int | None = None

    def on_request_start(self, text: str = "Querying LLM...") -> None:
        """
        Create the spinner line at the end of the log and start updating it.
        """

        self.color = "#0d87c0"
        self.update_color = "#15B606"
        self.text = text

        # Check if it is already running
        if self.spinner_timer is not None:
            return

        # Initialized the class variables
        self.spinner_line_index = len(self.console._log_lines)
        self.console._log(f"[{self.color}]{text}[/{self.color}]")
        self.spinner_frame_index = 0

        # Update every 0.1s
        self.spinner_timer = self.console.set_interval(0.1, self.update_spinner)
        # Update the terminal
        self.console.render_log()

    def update_spinner(self) -> None:
        """
        Timer callback. Rotate the spinner frame on the stored last log line.
        """

        # Check if the spinner is not running
        if self.spinner_line_index is None:
            return

        frame = self.spinner_frames[self.spinner_frame_index]
        self.spinner_frame_index = (self.spinner_frame_index + 1) % len(self.spinner_frames)

        # Update that specific line only
        self.console._log_lines[self.spinner_line_index] = \
            f"[{self.update_color}]{frame}[/{self.update_color}] " + \
            f"[{self.color}]{self.text}[/{self.color}]"

        # Update the terminal
        self.console.render_log()

    def on_request_end(self) -> None:
        """
        Stop the spinner.
        Optional, replace the line with final_text."""

        # Check if the spinner is running
        if self.spinner_timer is not None:
            self.spinner_timer.stop()
            self.spinner_timer = None

        # Update the spinner message line
        if self.spinner_line_index is not None:
            self.console._log_lines[self.spinner_line_index] += f"[{self.update_color}] Query finished![/{self.update_color}]"
            self.spinner_line_index = None
            self.console.render_log()

class CheckListScreen(ModalScreen[list[str] | None]):


    CSS = """
    CheckListScreen {
        align: center middle;
    }

    .dialog {
        width: 60%;
        max-width: 90%;
        height: 80%;          /* fixed portion of terminal */
        border: round $accent;
        padding: 1 2;
        background: $panel;
    }

    .title {
        text-align: center;
        margin-bottom: 1;
    }

    /* This is the important part */
    .checkbox-list {
        height: 1fr;          /* take all remaining vertical space */
                              /* no max-height, no overflow-y here */
    }

    .btns {
        height: 3;            /* give buttons row a fixed height */
        padding-top: 1;
        content-align: right middle;
    }
    """

    def __init__(self, lines: list[str], active_tools_num: int = 0) -> None:
        super().__init__()
        self._lines = list(lines)
        self.active_tools_num = active_tools_num

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Label("Pick tools you want to enable", classes="title")

            # SCROLLABLE CHECKBOX LIST
            with VerticalScroll(classes="checkbox-list"):
                for i, line in enumerate(self._lines, start=1):
                    yield Checkbox(line, value=i <= self.active_tools_num, id=f"cb{i}")

            # Buttons
            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            boxes = list(self.query(Checkbox))
            selected = [self._lines[i] for i, cb in enumerate(boxes) if cb.value]
            self.dismiss(selected)
        elif event.button.id == "cancel":
            self.dismiss(None)

    def on_mount(self) -> None:
        first_cb = self.query_one(Checkbox)
        self.set_focus(first_cb)

class VulcanConsole(App):

    CSS = """
    #log { height: 1fr; }
    #cmd { dock: bottom; }
    """

    BINDINGS = [
        Binding("ctrl+l", "clear_log", "Clear log"),
        Binding("f2", "show_help", "Show help", priority=True),
    ]

    def __init__(self, tools_from_entrypoints: str = "", user_context: str = "", main_node = None,
                 model: str = "gpt-5-nano", k: int = 7, iterative: bool = False):
        super().__init__() # textual lib

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
        self.log_lines = []

        # terminal qol
        self.history = []

    async def on_mount(self) -> None:
        # Disable terminal input
        self.set_input_enabled(False)
        sys.stdout = StreamToTextual(self, "stdout")
        sys.stderr = StreamToTextual(self, "stderr")

        # TODO
        # https://patorjk.com/software/taag/#p=display&f=Small+Slant&t=VulcanAI&x=none&v=4&h=4&w=80&we=false
#Standard
        vulcanai_tittle_std = \
"""
 __     __     _                    _    ___
 \ \   / /   _| | ___ __ _ _ __    / \  |_ _|
  \ \ / / | | | |/ __/ _` | '_ \  / _ \  | |
   \ V /| |_| | | (_| (_| | | | |/ ___ \ | |
    \_/  \__,_|_|\___\__,_|_| |_/_/   \_\___|
"""

#slant
        vulcanai_tittle_slant = \
"""
 _    __      __                 ___    ____
| |  / /_  __/ /________  ____  /   |  /  _/
| | / / / / / / ___/ __ `/ __ \/ /| |  / /
| |/ / /_/ / / /__/ /_/ / / / / ___ |_/ /
|___/\__,_/_/\___/\__,_/_/ /_/_/  |_/___/
"""
#small slant
        vulcanai_tittle_small_slant = \
"""
  _   __     __              ___   ____
 | | / /_ __/ /______ ____  / _ | /  _/
 | |/ / // / / __/ _ `/ _ \/ __ |_/ /
 |___/\_,_/_/\__/\_,_/_//_/_/ |_/___/
"""

#Doom
        vulcanai_tittle_doom = \
"""
 _   _       _                  ___  _____
| | | |     | |                / _ \|_   _|
| | | |_   _| | ___ __ _ _ __ / /_\ \ | |
| | | | | | | |/ __/ _` | '_ \|  _  | | |
\ \_/ / |_| | | (_| (_| | | | | | | |_| |_
 \___/ \__,_|_|\___\__,_|_| |_\_| |_/\___/
"""
# Small Block
        vulcanai_tittle_block = \
"""
▌ ▌   ▜          ▞▀▖▜▘
▚▗▘▌ ▌▐ ▞▀▖▝▀▖▛▀▖▙▄▌▐
▝▞ ▌ ▌▐ ▌ ▖▞▀▌▌ ▌▌ ▌▐
 ▘ ▝▀▘ ▘▝▀ ▝▀▘▘ ▘▘ ▘▀▘
"""
#Small
        vulcanai_tittle_small = \
"""
 __   __    _                _   ___
 \ \ / /  _| |__ __ _ _ _   /_\ |_ _|
  \ V / || | / _/ _` | ' \ / _ \ | |
   \_/ \_,_|_\__\__,_|_||_/_/ \_\___|
"""

        #self._log(f"{vulcanai_tittle_std}", log_color=1)
        #self._log(f"")
        self._log(f"{vulcanai_tittle_slant}", log_color=1)
        #self._log(f"")
        #self._log(f"{vulcanai_tittle_small_slant}", log_color=1)
        #self._log(f"")
        #self._log(f"{vulcanai_tittle_block}", log_color=1)
        #self._log(f"")
        #self._log(f"{vulcanai_tittle_small}", log_color=1)


        await asyncio.sleep(0)
        asyncio.create_task(self.bootstrap())

    def compose(self) -> ComposeResult:
        with VerticalScroll(id="logview"):
            yield Static("", id="logcontent")
        yield Input(placeholder="> ", id="cmd")

    def append_log_text(self, text: str) -> None:
        """Append text to the logcontent Static."""
        text = text.rstrip("\n")
        if not text:
            return

        self.log_lines.append(text)
        content = "\n".join(self.log_lines)

        log_static = self.query_one("#logcontent", Static)
        log_static.update(content)


    def attach_ros_logger_to_console(self, node):
        """
        Function that remove ROS node overlaping prints in the terminal
        """
        logger = node.get_logger()

        def info_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[INFO] {msg}[/gray]")

        def warn_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[WARN] {msg}[/gray]")

        def error_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[ERROR] {msg}[/gray]")

        logger.info = info_hook
        logger.warning = warn_hook
        logger.error = error_hook


    async def bootstrap(self, user_input: str="") -> None:
        """
        Function used to print information in runtime execution of a function
        """

        def worker(user_input: str="") -> None:

            # INITIALIZE CODE
            if user_input == "":
                self.init_manager()

                # add the commands
                # command registry: name -> handler
                self.commands = {
                    "/help": self.cmd_help,
                    "/tools": self.cmd_tools,
                    "/edit_tools": self.cmd_edit_tools,
                    "/change_k": self.cmd_change_k,
                    "/history": self.cmd_history_index,
                    "/show_history": self.cmd_show_history,
                    "/plan": self.cmd_plan,
                    "/rerun": self.cmd_rerun,
                    "/bb": self.cmd_blackboard_state,
                    "/clear": self.cmd_clear,
                    "/exit": self.cmd_quit,
                }

                # cycling through tab matches
                self.tab_matches = []
                self.tab_index = 0

                # Override hooks with spinner controller
                try:
                    self.manager.llm.set_hooks(self.hooks)
                except Exception:
                    pass

                if self.tools_from_entrypoints != "":
                    self.manager.register_tools_from_entry_points(self.tools_from_entrypoints)
                else:
                    self._log("WARNING. No tools added", log_color=3)

                self.manager.add_user_context(self.user_context)

                # Add the shared node to the console manager blackboard to be used by tools
                if self.main_node != None:
                    self.manager.bb["main_node"] = self.main_node
                    self.attach_ros_logger_to_console(self.main_node)
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

                    self._log(f"Output of plan: {result.get('blackboard', {None})}", log_color=2)

                except KeyboardInterrupt:
                    self._log("[yellow]Exiting...[/yellow]")
                    return
                except EOFError:
                    self._log("[yellow]Exiting...[/yellow]")
                    return

        # This is the main thread, here is where the hook is started for queries
        if user_input != "":
            self.hooks.on_request_start()

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker(user_input))

        if user_input == "":
            self.is_ready = True
            self._log("VulcanAI Interactive Console", log_color=2)
            self._log("Type [bold]'exit'[/bold] to quit.\n", log_color=2)

        # Activate the terminal input
        self.set_input_enabled(True)

    # region Utilities

    @work  # runs in a worker. waiting won't freeze the UI
    async def open_checklist(self, tools_list: list[str], active_tools_num: int) -> None:
        """
        Function used in '/edit_tools' command.
        It creates a dialog with all the tools.
        """

        # create the checklist dialog
        selected = await self.push_screen_wait(CheckListScreen(tools_list, active_tools_num))

        if selected is None:
            self._log("Selection cancelled.", log_color=3)
        else:

            for tool_tmp in tools_list:
                # remove "- "
                tool = tool_tmp[2:]
                if tool_tmp in selected:
                    self.manager.registry.activate_tool(tool)
                else:
                    if self.manager.registry.deactivate_tool(tool):
                        self._log(f"Deactivated tool [bold]'{tool}'[/bold]", log_color=2)

    # endregion

    # region Commands

    def cmd_help(self, _) -> None:
        table = "\n".join(
            [
                "[bold]Available commands:[/bold]\n"
                "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
                "/[bold]help[/bold]           - Show this help message\n"
                "/[bold]tools[/bold]          - List available tools\n"
                "/[bold]edit_tools[/bold]     - Edit the list of available tools\n"
                "/[bold]change_k <int>[/bold] - Change the 'k' value for the top_k algorithm selection or show the current value if no <int> is provided\n"
                "/[bold]history <int>[/bold]  - Change the history depth or show the current value if no <int> is provided\n"
                "/[bold]show_history[/bold]   - Show the current history\n"
                "/[bold]plan[/bold]           - Show the last generated plan\n"
                "/[bold]rerun[/bold]          - Rerun the last plan\n"
                "/[bold]bb[/bold]             - Show the last blackboard state\n"
                "/[bold]clear[/bold]          - Clears the console screen\n"
                "/[bold]exit[/bold]           - Exit the console\n"
                "[bold]Query any other text[/bold] to process it with the LLM and execute the plan generated.\n\n"
                "Add --image=<path> to include images in the query. It can be used multiple times to add more images.\n"
                "Example: '<user_prompt> --image=/path/to/image1 --image=/path/to/image2'\n"
                "\n"
                "[bold]Available keybinds:[/bold]\n"
                "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
                "[bold]F2[/bold]                - Show this help message\n"
                "[bold]Ctrl+Q[/bold]            - Exit the console\n"
                "[bold]Ctrl+L[/bold]            - Clears the console screen\n"
                "[bold]Ctrl+U[/bold]            - Clears the entire command line input\n"
                "[bold]Ctrl+K[/bold]            - Clears from the cursor to then end of the line\n"
                "[bold]Ctrl+W[/bold]            - Delete the word before the cursor\n"
                "[bold]Ctrl+<left/right>[/bold] - Move cursor backward/forward by one word\n"
                "[bold]Ctrl+R[/bold]            - Reverse search through command history (try typing part of a previous command).\n"
            ]
        )
        self._log(table, log_color=2)

    def cmd_tools(self, _) -> None:
        tmp_msg = f"Available tools (current index k={self.manager.k}):"
        tool_msg = f"[bold]{tmp_msg}[/bold]\n"
        tool_msg += "‾" * len(tmp_msg) +'\n'

        for tool in self.manager.registry.tools.values():
            tool_msg += f"- [bold]{tool.name}:[/bold] {tool.description}\n"
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
            self._log(f"Usage: /change_k <int> - Actual 'k' is {self.manager.k}",
                      log_type="error", log_color=2)
            return

        new_k = int(args[0])
        self.manager.k = new_k
        self.manager.update_k_index(new_k)

    def cmd_history_index(self, args) -> None:
        if len(args) == 0:
            self._log(f"Current 'history depth' is {self.manager.history_depth}",
                      log_color=2)
            return
        if len(args) != 1 or not args[0].isdigit():
            self._log(f"Usage: /history <int> - Actual 'history depth' is {self.manager.history_depth}",
                      log_type="error", log_color=2)
            return

        new_hist = int(args[0])
        self.manager.update_history_depth(new_hist)

    def cmd_show_history(self, _) -> None:
        if not self.manager.history:
            self._log("No history available.", log_color=2)
            return

        history_msg = "\nCurrent history (oldest first):\n"
        for i, (user_text, plan_summary) in enumerate(self.manager.history):
            history_msg += f"{i+1}. User: {user_text}\n   Plan summary: {plan_summary}\n"

        self._log(history_msg, log_color=2)

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
        self.log_lines.clear()
        self.query_one("#logcontent", Static).update("")

    def cmd_quit(self, _) -> None:
        self.exit()

    def cmd_echo(self, args) -> None:
        self._log(" ".join(args))

    # endregion

    # region Logging

    def render_log(self) -> None:
        log_static = self.query_one("#logcontent", Static)
        log_static.update("\n".join(self.log_lines))

        self.query_one("#logview", VerticalScroll).scroll_end(animate=False)

    def _log(self, line: str, log_type: str = "", log_color: int = -1, print_args_idx: int=-1) -> None:
        msg = ""

        color_type = ""

        if log_type == "register":
            color_tmp = "#068399"
            msg = f"[bold {color_tmp}]\[REGISTRY][/bold {color_tmp}] "
        elif log_type == "manager":
            color_tmp = "#0d87c0"
            msg = f"[bold {color_tmp}]\[MANAGER][/bold {color_tmp}] "
        elif log_type == "executor":
            color_tmp = "#15B606"
            msg = f"[bold {color_tmp}]\[EXECUTOR][/bold {color_tmp}] "
        elif log_type == "validator":
            msg = "[bold orange_red1]\[VALIDATOR][/bold orange_red1] "
        elif log_type == "error":
            msg = "[bold red]\[ERROR][/bold red] "


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
            msg += f"{line}"
            self.log_lines.append(msg)
            self.render_log()
            return

        msg += f"[{color_type}]{line}[/{color_type}]"
        self.log_lines.append(msg)
        self.render_log()

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

        try:
            if event.input.id != "cmd":
                return
            user_input = (event.value or "").strip()

            # the the user_input in the history navigation list (used when the up, down keys are pressed)
            self.history.append(user_input)
            event.input.value = ""
            event.input.focus()

            # reset tab state
            self.tab_matches = []
            self.tab_index = 0

            if not user_input:
                self.cmd_input.focus()
                return

            # Terminal history inputs navigation
            self._history_index = None

            # echo what the user typed (keep this if you like the prompt arrow)
            color_user = "#91DD16"
            self._log(f"[bold {color_user}]\[USER] >>>[/bold {color_user}] {cmd}")

            # If it doesn't start with '/', just print it as output and stop here
            if user_input.startswith("/"):
                self.handle_command(user_input)
                return

            await asyncio.sleep(0)
            asyncio.create_task(self.bootstrap(user_input))

        except KeyboardInterrupt:
            self._log("[yellow]Exiting...[/yellow]")
            return
        except EOFError:
            self._log("[yellow]Exiting...[/yellow]")
            return

    def handle_command(self, user_input: str) -> None:
        # Otherwise, parse as a command
        parts = user_input.split()
        cmd = parts[0].lower()
        args = parts[1:]

        handler = self.commands.get(cmd)
        if handler is None:
            # only complain for slash-commands
            self._log(f"Unknown command: [b]{cmd}[/]. Type '/help'.", log_color=2)
        else:
            try:
                handler(args)
            except Exception as e:
                self._log(f"Error: {e!r}", log_color=0)

    """def _reverse_search_reset(self) -> None:
        self._rev_search_active = False
        self._rev_search_query = ""
        self._rev_search_index = None

    def _reverse_search_update(self) -> None:
        # Search backwards in history using self._rev_search_query.
        cmd_input = self.query_one("#cmd", Input)

        if not self.history:
            cmd_input.value = f"(reverse-i-search)`{self._rev_search_query}`: "
            cmd_input.cursor_position = len(cmd_input.value)
            return

        query = self._rev_search_query
        # If index is None, start from the end
        start = len(self.history) if self._rev_search_index is None else self._rev_search_index

        found = None
        for i in range(start - 1, -1, -1):
            if query in self.history[i]:
                found = (i, self.history[i])
                break

        if found is not None:
            self._rev_search_index, match = found
            cmd_input.value = f"(reverse-i-search)`{query}`: {match}"
        else:
            # No match: just show query
            cmd_input.value = f"(reverse-i-search)`{query}`: "

        cmd_input.cursor_position = len(cmd_input.value)"""

    async def on_key(self, event: events.Key) -> None:
        """Handle Up/Down for history navigation."""
        key = event.key
        cmd_input = self.query_one("#cmd", Input)

        # NAVIGATE
        if key in ("up", "down"):

            # Only handle history navigation if input is focused
            if self.focused is not cmd_input:
                return

            if not self.history:
                return

            # Initialize history index if not already set
            if not hasattr(self, "_history_index") or self._history_index is None:
                self._history_index = len(self.history)

            # store the command input if it is new
            if self._history_index == len(self.history):
                self.terminal_input = cmd_input.value

            if key == "up" and self._history_index > 0:
                self._history_index -= 1
            elif key == "down" and self._history_index < len(self.history):
                self._history_index += 1
            else:
                return  # ignore if out of range

            # Update input value based on history
            if 0 <= self._history_index < len(self.history):
                cmd_input.value = self.history[self._history_index]
            else:
                cmd_input.value = self.terminal_input

            # Move cursor to end
            cmd_input.cursor_position = len(cmd_input.value)
            event.stop()
            return

        # REVERSE SEARCH: Ctrl+R
        """if key == "ctrl+r":
            # Only handle if input focused and we have history
            if self.focused is not cmd_input or not self.history:
                return

            current_query = cmd_input.value  # use current input as search query

            # Start a new search if:
            # - not active yet, or
            # - query changed
            if not self._rev_search_active: #or (current_query != (self._rev_search_query or "")):
                self._log("entra")
                self._rev_search_active = True
                cmd_input.value = "(reverse-i-search)`"
                #cmd_input.value = new_value
                cmd_input.cursor_position = len(cmd_input.value)

                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return"""

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
                prefix = self.common_prefix(matches, raw)
                new_value = prefix
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
            try:
                paste_text = pyperclip.paste() or ""
            except Exception as e:
                self._log(f"Clipboard error: {e}", log_color=0)
                return

            if not paste_text:
                return

            # Remove endlines in the paste
            i = 0
            n = len(paste_text)
            while i < n:
                if paste_text[i] == '\n':
                    paste_text = paste_text[:i]
                    break
                i += 1

            self._log(paste_text)

            value = cmd_input.value
            cursor = cmd_input.cursor_position

            # Insert clipboard text at cursor
            cmd_input.value = value[:cursor] + paste_text + value[cursor:]
            cmd_input.cursor_position = cursor + len(paste_text)

            cmd_input.focus()
            event.prevent_default()
            event.stop()
            return

        """# When in reverse-search mode, normal keys update the query and run the search
        if self._rev_search_active:
            # Accept current match with Enter
            if key == "enter":
                # Extract the matched command, if any
                if (
                    self._rev_search_index is not None
                    and 0 <= self._rev_search_index < len(self.history)
                ):
                    cmd_input.value = self.history[self._rev_search_index]
                else:
                    cmd_input.value = self._rev_search_query  # fallback
                cmd_input.cursor_position = len(cmd_input.value)
                self._reverse_search_reset()

                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # Cancel reverse search with Escape / Ctrl+C
            if key in ("escape", "ctrl+c"):
                self._reverse_search_reset()
                cmd_input.value = ""
                cmd_input.cursor_position = 0

                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # Backspace edits the query
            if key == "backspace":
                if self._rev_search_query:
                    self._rev_search_query = self._rev_search_query[:-1]
                self._reverse_search_update()

                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # Printable characters extend the query
            if len(key) == 1 and key.isprintable():
                self._rev_search_query += key
                self._reverse_search_update()

                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return"""

        # Any other keypress resets tab cycle if the prefix changes
        if len(key) == 1 or key in ("backspace", "delete"):
            self.tab_matches = []
            self.tab_index = 0

    def common_prefix(self, strings: str, cmd_input: str="") -> str:
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

        # echo what the user typed (keep this if you like the prompt arrow)
        color_user = "#91DD16"
        self._log(f"[bold {color_user}]\[USER] >>>[/bold {color_user}] {cmd_input}")
        self._log(commands, log_color=2)

        return common_prefix

    # endregion

    # region Actions (key bindings)

    # Called by Binding("ctrl+l", "clear_log", ...)
    def action_clear_log(self) -> None:
        self.cmd_clear(_=None)

    # Called by Binding("ctrl+h", "show_help", ...)
    def action_show_help(self) -> None:
        self.cmd_help(_=None)

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
        self._log(f"Initializing Manager [bold]'{ConsoleManager.__name__}'[/bold] ...", log_color=2)

        self.manager = ConsoleManager(model=self.model, k=self.k, logger=self._log)

        # Print in textual terminal:
        # Manager initialized with model '<model>'
        self._log(f"Manager initialized with model [bold]'{self.model}[/bold]'", log_color=2)

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