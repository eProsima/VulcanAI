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

from threading import Lock
import argparse
import os

from prompt_toolkit import PromptSession
from rich.progress import Progress, SpinnerColumn, TextColumn
from vulcanai.models.model import IModelHooks
from vulcanai.console.logger import console

from textual.screen import ModalScreen
from textual.app import App, ComposeResult
from textual.widgets import Input, Log, Static, Checkbox, Button, Label
from textual import events


from rich.text import Text
from rich.markup import escape

from textual import work

from textual.containers import VerticalScroll, Horizontal, Vertical

import asyncio, time
from typing import Callable, Iterable, Optional

from textual.timer import Timer

import sys

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



class Prompt(Static):
    """A tiny prompt label shown to the left of the input."""

    def __init__(self, text: str = "> "):
        super().__init__(text, id="prompt")

class SpinnerHook:

    def __init__(self, console):

        self.console = console

        # spinner states
        self.spinner_timer: Timer | None = None
        self.spinner_frames = ["⠋","⠙","⠹","⠸","⠼","⠴","⠦","⠧","⠇","⠏"]
        self.spinner_frame_index = 0
        self.spinner_line_index: int | None = None


        #self.running_query = False
        #self.create_spinner_timer()

    async def create_spinner_timer(self):
        self.spinner_timer = self.console.set_interval(0.1, self.update_spinner)

    def on_request_start(self, text: str = "Querying LLM...") -> None:
        """
        Create the spinner line at the end of the log and start updating it.
        """

        self.color = "#0d87c0"
        self.update_color = "#15B606"
        self.text = text

        if self.spinner_timer is not None:
            return  # already running

        # Add a new line for the spinner and remember its index
        self.spinner_line_index = len(self.console._log_lines)
        self.console._log(f"[{self.color}]{text}[/{self.color}]")
        #self.console._log_lines.append(f"[{self.running_color}]{text}[/{self.running_color}]")
        self.spinner_frame_index = 0

        # Update every 0.1s
        self.spinner_timer = self.console.set_interval(0.1, self.update_spinner)
        #self.call_from_thread(self.create_spinner_timer)
        #self.running_query = True

        self.console.render_log()

    def update_spinner(self) -> None:
        """
        Timer callback. Rotate the spinner frame on the stored last log line.
        """
        #if self.running_query == False:
        #    return


        if self.spinner_line_index is None:
            return

        frame = self.spinner_frames[self.spinner_frame_index]
        self.spinner_frame_index = (self.spinner_frame_index + 1) % len(self.spinner_frames)

        # Update that specific line only
        self.console._log_lines[self.spinner_line_index] = f"[{self.update_color}]{frame}[/{self.update_color}] [{self.color}]{self.text}[/{self.color}]"
        #self.console._log(f"[{self.running_color}] Sleeping {frame} [/{self.running_color}]")
        self.console.render_log()

    def on_request_end(self) -> None:
        """
        Stop the spinner.
        Optional, replace the line with final_text."""

        if self.spinner_timer is not None:
            self.spinner_timer.stop()
            self.spinner_timer = None

        if self.spinner_line_index is not None:
            self.console._log_lines[self.spinner_line_index] += f"[{self.update_color}] Query finished![/{self.update_color}]"
            self.spinner_line_index = None
            self.console.render_log()


# ---------- Modal checklist ----------
class CheckListScreen(ModalScreen[list[str] | None]):
    """A modal screen with the tools checkboxes and Submit/Cancel buttons."""

    DEFAULT_CSS = """
    CheckListScreen {
        align: center middle;
    }
    .dialog {
        width: 60%;
        max-width: 90%;
        border: round $accent;
        padding: 1 2;
        background: $panel;
    }
    .title {
        text-align: center;
        margin-bottom: 1;
    }
    .btns {
        height: auto;
        dock: bottom;
        padding-top: 1;
        content-align: right middle;
    }
    """


    def __init__(self, lines: Iterable[str], active_tools_num: int=0) -> None:
        super().__init__()
        self._lines = list(lines)
        self.active_tools_num = active_tools_num

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Label("Pick the lines you want to print", classes="title")
            # Make one checkbox per provided line
            for i, line in enumerate(self._lines, start=1):
                yield Checkbox(line, value=i<=self.active_tools_num, id=f"cb{i}")#True, id=f"cb{i}")
            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            boxes = list(self.query(Checkbox))
            # Use the original strings instead of Checkbox.label (renderable)
            selected = [self._lines[i] for i, cb in enumerate(boxes) if cb.value]
            self.dismiss(selected)  # -> list[str]
        elif event.button.id == "cancel":
            self.dismiss(None)

    def on_mount(self) -> None:
        # Focus the first checkbox for keyboard toggling with space
        first_cb = self.query_one(Checkbox)
        self.set_focus(first_cb)

class VulcanConsole(App):

    # Terminal style
    """CSS =
    Screen {
        layout: vertical;
    }

    # Header at top, then the log grows, then input row at bottom
    # A small prompt sits to the left of the input for terminal vibes

    # Output area
    # Make it fill remaining space and look terminal-ish
    # Log already has a good style; just ensure it expands
    # and wraps nicely.
    Log#log {
        height: 1fr;
        border: solid rgb(0, 205, 0);
        background: $boost;
        padding: 1 2;
        overflow-y: auto;
    }

    .input-row {
        height: auto;
        layout: horizontal;
        padding: 0 1;
        dock: bottom;
        background: $panel;
    }

    # Prompt label
    # Slightly dim to look like a shell prompt
    # Input stretches to fill the row
    # The Input gets a monospace look by default under Textual

    # Prompt label style
    # (Using Static for a simple label avoids extra dependencies.)

    # Make the input larger for comfortable typing
    Input#cmd {
        width: 1fr;
        padding: 0 1;
    }

    Static#prompt {
        width: auto;
        color: $text-muted;
        content-align: left middle;
    }

    Static#hint {
        height: auto;
        color: $text-muted;
        padding: 0 2;
    }
    """

    CSS = """
    #log { height: 1fr; }
    #cmd { dock: bottom; }
    """

    """BINDINGS = [
        ("ctrl+q", "app_quit", "Quit"),
        ("ctrl+p", "clear", "Clear"),
        ("f1", "help", "Help"),
        ("ctrl+alt+c", "copy", "Copy Selection"),  # new: mirrors terminal habit
    ]"""

    BINDINGS = [
        ("ctrl+c", "copy_to_clipboard", "Copy log to clipboard"),
        ("y", "copy_log", "Copy log to clipboard"),
        #("ctrl+l", "clear", "Clear the terminal"),
    ]

    def __init__(self, tools_from_entrypoints: str = "", user_context: str = "", main_node = None,
                 model: str = "gpt-5-nano", k: int = 7, iterative: bool = False):
        super().__init__() # textual lib

        self.manager = None
        self.session = PromptSession()
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
        self._tab_matches = []
        self._tab_index = 0
        self._log_lines = []

        # terminal qol
        self.history = []

    async def on_mount(self) -> None:

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
        """yield Header(show_clock=True)
        #yield Log(id="term", highlight=True)#, markup=True)
        yield Log(id="log", highlight=True)
        yield Static("Press [b]F1[/b] for help • [b]Ctrl+C[/b] to quit • [b]Tab[/b] to autocomplete", id="hint")
        with Static(classes="input-row"):
            yield Prompt("> ")
            yield Input(placeholder="Type a command and press Enter…", id="cmd")
        yield Footer()"""

        """yield Static("", id="log")
        yield Input(placeholder="> ", id="cmd")"""

        with VerticalScroll(id="logview"):
            yield Static("", id="logcontent")
        yield Input(placeholder="> ", id="cmd")

    def append_log_text(self, text: str) -> None:
        """Append text to the logcontent Static."""
        text = text.rstrip("\n")
        if not text:
            return

        self._log_lines.append(text)
        content = "\n".join(self._log_lines)

        log_static = self.query_one("#logcontent", Static)
        log_static.update(content)


    def attach_ros_logger_to_console(self, node):
        logger = node.get_logger()

        def info_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[INFO] {msg}[/gray]")
            #return original_info(msg, *args, **kwargs)

        def warn_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[WARN] {msg}[/gray]")
            #return original_warn(msg, *args, **kwargs)

        def error_hook(msg, *args, **kwargs):
            self.call_from_thread(self._log, f"[gray]\[ROS] \[ERROR] {msg}[/gray]")
            #return original_error(msg, *args, **kwargs)

        logger.info = info_hook
        logger.warning = warn_hook
        logger.error = error_hook


    async def bootstrap(self, user_input: str="") -> None:
        """
        Function used to print information in runtime execution of a function
        """

        def worker(user_input: str="") -> None:

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
                self._tab_matches = []
                self._tab_index = 0

                # Override hooks with spinner controller
                try:
                    self.manager.llm.set_hooks(self.hooks)
                except Exception:
                    pass


                if self.tools_from_entrypoints != "":
                    self.manager.register_tools_from_entry_points(self.tools_from_entrypoints)


                self.manager.add_user_context(self.user_context)


                # Add the shared node to the console manager blackboard to be used by tools
                if self.main_node != None:
                    self.manager.bb["main_node"] = self.main_node
                    self.attach_ros_logger_to_console(self.main_node)
                    #attach_ros_logger_to_console(self, self.main_node)
            else:
                self.set_input_enabled(False)

                try:
                    images = []
                    if "--image=" in user_input:
                        images = self.get_images(user_input)

                    # Handle user request
                    try:
                        result = self.manager.handle_user_request(user_input, context={"images": images})
                    except Exception as e:
                        #self.print(f"[error]Error handling request:[/error] {e}")
                        self._log(f"[error]Error handling request:[/error] {e}")
                        return

                    self.last_plan = result.get("plan", None)
                    self.last_bb = result.get("blackboard", None)

                    #self.print(f"Output of plan: {result.get('blackboard', {None})}")
                    self._log(f"Output of plan: {result.get('blackboard', {None})}", log_color=2)

                except KeyboardInterrupt:
                    #console.print("[yellow]Exiting...[/yellow]")
                    self._log("[yellow]Exiting...[/yellow]")
                    return
                except EOFError:
                    #console.print("[yellow]Exiting...[/yellow]")
                    self._log("[yellow]Exiting...[/yellow]")
                    return

        if user_input != "":
            self.hooks.on_request_start()

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: worker(user_input))

        if user_input == "":

            self._is_ready = True
            self.set_input_enabled(True)
            self._log("VulcanAI Interactive Console", log_color=2)
            self._log("Type [bold]'exit'[/bold] to quit.\n", log_color=2)
        else:
            self.set_input_enabled(True)

    # region Utilities

    """@property
    def term(self):
        return self.query_one("#log", Log)

    @property
    def cmd_input(self):
        return self.query_one("#cmd", Input)#, id="term")

    def print_system(self, message: str):
        #self.term.write(f"[bold cyan]system[/]: {message}")
        self.term.write(f"{message}")

        #self.term.write(Text.from_markup(f"[{green_hex}]system[/]: {escape(message)}"))
        #self.term.write(Text.from_markup(f"{escape(message)}"))

    def print_output(self, message: str):
        self.term.write(message)"""

    @work  # runs in a worker so waiting won't freeze the UI
    async def open_checklist(self, tools_list: list[str], active_tools_num: int) -> None:
        """
        Function used in '/edit_tools' command.
        It creates a dialog with all the tools.
        """
        # create the checklist dialog
        selected = await self.push_screen_wait(CheckListScreen(tools_list, active_tools_num))

        if selected is None:
            self._log("Selection cancelled.", log_color=3)
        elif not selected:
            self._log("No items selected.", log_color=3)
        else:

            for tool_tmp in tools_list:
                tool = tool_tmp[2:] # remove "- "
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
                "/[bold]clear[/bold]          - Clear the console screen\n"
                "/[bold]exit[/bold]           - Exit the console\n"
                "[bold]Query any other text[/bold] to process it with the LLM and execute the plan generated.\n\n"
                "Add --image=<path> to include images in the query. It can be used multiple times to add more images.\n"
                "Example: '<user_prompt> --image=/path/to/image1 --image=/path/to/image2'"
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
            tools_list.append(f"- {tool.name}")#: {tool.description}")

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
        self._log_lines.clear()
        self.query_one("#logcontent", Static).update("")

    def cmd_quit(self, _) -> None:
        self.exit()

    def cmd_echo(self, args) -> None:
        self._log(" ".join(args))

    # endregion

    # region Logging

    def render_log(self) -> None:
        """self.query_one("#log", Static).update("\n".join(self._log_lines))"""
        log_static = self.query_one("#logcontent", Static)
        log_static.update("\n".join(self._log_lines))

        self.query_one("#logview", VerticalScroll).scroll_end(animate=False)

    """def render_log(self):
        log = self.query_one("#log", Static)
        # Allow Rich markup for colors and styles
        log.update("\n".join(self._log_lines), markup=True)"""

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


        """if print_args_idx > 0:
            msg += line[:print_args_idx]
            i = print_args_idx
            n = len(line)

            color_1 = "#C49C00"
            color_2 = "#069899"
            msg += f"[{color_1}]"

            while i < n:
                c = line[i]
                if c == '=' or c == ':':
                    msg += f"[/{color_1}]"
                    msg += line[i]
                    msg += f"[{color_2}]"
                elif c == ',':
                    msg += f"[/{color_2}]"
                    msg += line[i]
                    msg += f"[{color_1}]"
                elif c == '}' or c == ')':
                    msg += f"[/{color_2}]"
                    msg+=line[i]
                    break
                else:
                    msg += line[i]

                i+=1

            if i < n:
                msg += line[i:]



            self._log_lines.append(msg)
            self.render_log()
            return"""




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
            self._log_lines.append(msg)
            self.render_log()
            return

        msg += f"[{color_type}]{line}[/{color_type}]"
        self._log_lines.append(msg)
        self.render_log()

    """def _log(self, line: str):
        if "error" in line.lower():
            line = f"[red]{line}[/red]"
        elif "warn" in line.lower():
            line = f"[yellow]{line}[/yellow]"
        elif "success" in line.lower():
            line = f"[green]{line}[/green]"
        self._log_lines.append(line)
        self.render_log()"""

    # endregion

    # region Input

    def set_input_enabled(self, enabled: bool) -> None:
        cmd = self.query_one("#cmd", Input)
        cmd.disabled = not enabled
        if enabled:
            self.set_focus(cmd)

    """@work  # runs in a worker so waiting won't freeze the UI
    async def handle_user_query(self, user_input) -> None:
        #""
        Function used in '/edit_tools' command.
        It creates a dialog with all the tools.
        #""
        # create the checklist dialog
        # Check for image input. Must be always at the end of the input

        try:
            images = []
            if "--image=" in user_input:
                images = self.get_images(user_input)

            # Handle user request
            try:
                result = self.manager.handle_user_request(user_input, context={"images": images})
            except Exception as e:
                #self.print(f"[error]Error handling request:[/error] {e}")
                self._log(f"[error]Error handling request:[/error] {e}")
                return

            self.last_plan = result.get("plan", None)
            self.last_bb = result.get("blackboard", None)

            #self.print(f"Output of plan: {result.get('blackboard', {None})}")
            self._log(f"Output of plan: {result.get('blackboard', {None})}")

        except KeyboardInterrupt:
            #console.print("[yellow]Exiting...[/yellow]")
            self._log("[yellow]Exiting...[/yellow]")
            return
        except EOFError:
            #console.print("[yellow]Exiting...[/yellow]")
            self._log("[yellow]Exiting...[/yellow]")
            return"""

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        """
        Enter key
        """

        if not self._is_ready:
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
            self._tab_matches = []
            self._tab_index = 0

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

            #self.handle_user_query(user_input)
            await asyncio.sleep(0)
            asyncio.create_task(self.bootstrap(user_input))

        except KeyboardInterrupt:
            #console.print("[yellow]Exiting...[/yellow]")
            self._log("[yellow]Exiting...[/yellow]")
            return
        except EOFError:
            #console.print("[yellow]Exiting...[/yellow]")
            self._log("[yellow]Exiting...[/yellow]")
            return

        """
        self.print_output(user_input)
        event.input.value = ""
        event.input.focus()
        return
        """

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

    async def on_key(self, event: events.Key) -> None:
        """Handle Up/Down for history navigation."""
        key = event.key
        cmd_input = self.query_one("#cmd", Input)


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

            self._log(f"Historyindex = {self._history_index}")

            # Update input value based on history
            if 0 <= self._history_index < len(self.history):
                cmd_input.value = self.history[self._history_index]
            else:
                cmd_input.value = self.terminal_input

            # Move cursor to end
            cmd_input.cursor_position = len(cmd_input.value)
            event.stop()
            return

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
            remainder = rest[0] if rest else ""

            # Nothing typed yet: list all commands
            all_cmds = sorted(self.commands) if self.commands else []
            if not all_cmds:
                return

            self._tab_matches = [c for c in all_cmds if c.startswith(head)] if head else all_cmds
            self._tab_index = 0

            matches = self._tab_matches
            if not matches:
                cmd_input.focus()
                event.prevent_default()
                event.stop()
                return

            # If multiple matches, check for a longer common prefix to insert first
            if len(matches) > 1:
                prefix = self.common_prefix(matches)
                new_value = prefix
            else:
                # Single match: complete directly
                new_value = matches[0]

            # Rebuild the input value:
            cmd_input.value = new_value
            cmd_input.cursor_position = len(cmd_input.value)

            cmd_input.focus()          # keep caret in the input
            event.prevent_default()
            event.stop()
            return

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

            cmd_input.focus()          # keep caret in the input
            event.prevent_default()
            event.stop()
            return

        if key in ("ctrl+delete", "escape") :
            value = cmd_input.value
            cursor = cmd_input.cursor_position
            i = cursor-1
            n = len(value)

            while i < n:
                if(value[i] == ' '): # TODO danip. mirar cuando tiene que borrar espacios en blanco
                    break
                i += 1

            cmd_input.value = value[:cursor] + value[i:]
            cmd_input.cursor_position = i

            cmd_input.focus()          # keep caret in the input
            event.prevent_default()
            event.stop()
            return

        if key == "ctrl+v":
            try:
                paste_text = pyperclip.paste() or ""
            except Exception as e:
                self._log(f"Clipboard error: {e}", log_color=0)
                return

            if not paste_text:
                return

            value = cmd_input.value
            cursor = cmd_input.cursor_position

            # Insert clipboard text at cursor
            cmd_input.value = value[:cursor] + paste_text + value[cursor:]
            cmd_input.cursor_position = cursor + len(paste_text)

            cmd_input.focus()
            event.prevent_default()
            event.stop()
            return

        # Any other keypress resets tab cycle if the prefix changes
        if len(key) == 1 or key in ("backspace", "delete"):
            self._tab_matches = []
            self._tab_index = 0

    def common_prefix(self, strings: str) -> str:
        if not strings:
            return ""

        common_prefix = strings[0]
        commands = strings[0]

        for i in range(1, len(strings)):
            commands += f" {strings[i]}"

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

        self._log(commands, log_color=2)

        return common_prefix

    # endregion

    # region Actions (key bindings)


    # endregion

    def run_console(self) -> None:
        #self.print("VulcanAI Interactive Console")
        #self.print("Type 'exit' to quit.\n")

        self.run()

        """while True:
            try:
                user_input = self.session.prompt("[USER] >>> ")
                if user_input.strip().lower() in ("exit", "quit"):
                    break

                # Internal commands start with /<command>
                if user_input.startswith("/"):
                    self.handle_command(user_input)
                    continue

                # Check for image input. Must be always at the end of the input
                images = []
                if "--image=" in user_input:
                    images = self.get_images(user_input)

                # Handle user request
                try:
                    result = self.manager.handle_user_request(user_input, context={"images": images})
                except Exception as e:
                    self.print(f"[error]Error handling request:[/error] {e}")
                    continue

                self.last_plan = result.get("plan", None)
                self.last_bb = result.get("blackboard", None)

                self.print(f"Output of plan: {result.get('blackboard', {None})}")

            except KeyboardInterrupt:
                console.print("[yellow]Exiting...[/yellow]")
                break
            except EOFError:
                console.print("[yellow]Exiting...[/yellow]")
                break"""

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


    def print(self, msg: str) -> None:
        console.print(f"[console]{msg}[/console]")

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
