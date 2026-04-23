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

from textual import events
from textual.app import ComposeResult
from textual.containers import Container, Horizontal, Vertical, VerticalScroll
from textual.content import Content
from textual.screen import ModalScreen
from textual.widgets import Button, Checkbox, Input, Label, RadioButton, RadioSet


class ReverseSearchModal(ModalScreen[str | None]):
    """
    Bottom modal for reverse-i-search.
    """

    DEFAULT_CSS = """
    ReverseSearchModal {
        align-horizontal: center;
        align-vertical: bottom;
    }

    #rev-box {
        width: 100%;
        height: 3;
        background: $surface;
        border-top: solid $accent;
        padding: 0 1;
        layout: horizontal;
    }

    #rev-label {
        width: 2fr;
        content-align: left middle;
    }

    #rev-input {
        width: 1fr;
    }
    """

    def __init__(self, history: list[str]) -> None:
        super().__init__()
        self.history = history
        self.search_query: str = ""
        self.match_index: int | None = None

    def compose(self) -> ComposeResult:
        with Container(id="rev-box"):
            # Label shows "(reverse-i-search)`query`: match"
            yield Label("(reverse-i-search)``: ", id="rev-label")
            yield Input(placeholder="type to search…", id="rev-input")

    def on_mount(self) -> None:
        self.query_one("#rev-input", Input).focus()

    def update_label(self) -> None:
        """
        Update label based on current query + best match from history.
        """

        label = self.query_one("#rev-label", Label)
        query = self.search_query

        if not self.history or not query:
            label.update(f"(reverse-i-search)`{query}`: ")
            return

        # Start from the end and search backwards for first match
        start = len(self.history) if self.match_index is None else self.match_index
        found = None
        for i in range(start - 1, -1, -1):
            if query in self.history[i]:
                found = (i, self.history[i])
                break

        if found is not None:
            self.match_index, match = found
            label.update(f"(reverse-i-search)`{query}`: {match}")
        else:
            self.match_index = None
            label.update(f"(reverse-i-search)`{query}`: ")

    def on_input_changed(self, event: Input.Changed) -> None:
        """
        Whenever user types in the textbox, update query + label.
        """

        if event.input.id != "rev-input":
            return
        self.search_query = event.value
        self.match_index = None  # restart from most recent
        self.update_label()

    async def on_key(self, event: events.Key) -> None:
        key = event.key

        # Accept current match
        if key in ("enter", "tab"):
            result: str | None
            if self.match_index is not None and 0 <= self.match_index < len(self.history):
                result = self.history[self.match_index]
            else:
                result = self.search_query or None

            self.dismiss(result)
            event.stop()
            return

        # Cancel with Esc / Ctrl+C
        if key in ("escape", "ctrl+c"):
            self.dismiss(None)
            event.stop()
            return

        # Ctrl+R while in modal = go to previous match (optional)
        if key == "ctrl+r" and self.search_query:
            # look for previous match starting before current match_index
            start = self.match_index if self.match_index is not None else len(self.history)
            found = None
            for i in range(start - 1, -1, -1):
                if self.search_query in self.history[i]:
                    found = (i, self.history[i])
                    break

            if found is not None:
                self.match_index, match = found
                label = self.query_one("#rev-label", Label)
                label.update(f"(reverse-i-search)`{self.search_query}`: {match}")

            event.prevent_default()
            event.stop()
            return


class CheckListModal(ModalScreen[list[str] | None]):
    CSS = """
    CheckListModal {
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

    .group-child {
        margin-left: 4;
        margin-right: 2;
        padding-right: 2;
    }

    .btns {
        height: auto;
        width: 100%;
        margin-top: 1;
        padding: 0;
        content-align: center middle;
        align-horizontal: center;
    }

    .btns Button {
        padding: 0 3;
        margin: 0 2;
    }
    """

    def __init__(self, grouped: list, active_tools: set) -> None:
        super().__init__()
        self.grouped = grouped  # [(prefix, [subtools]) | (name, None), ...]
        self.active_tools = active_tools
        self._parent_to_children: dict[str, list[str]] = {}
        self._child_to_parent: dict[str, str] = {}
        self._id_to_tool: dict[str, str] = {}  # cb_id -> full tool name (children & standalone only)

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Label("Pick tools you want to enable", classes="title")

            with VerticalScroll(classes="checkbox-list"):
                idx = 0
                for group_name, subtools in self.grouped:
                    if subtools is None:
                        # Standalone tool
                        cb_id = f"cb{idx}"
                        self._id_to_tool[cb_id] = group_name
                        yield Checkbox(
                            group_name,
                            value=group_name in self.active_tools,
                            id=cb_id,
                        )
                        idx += 1
                    else:
                        # Group parent
                        parent_id = f"cb{idx}"
                        idx += 1
                        child_ids = []
                        for subtool in subtools:
                            child_id = f"cb{idx}"
                            full_name = f"{group_name}_{subtool}"
                            self._id_to_tool[child_id] = full_name
                            child_ids.append(child_id)
                            idx += 1

                        self._parent_to_children[parent_id] = child_ids
                        for cid in child_ids:
                            self._child_to_parent[cid] = parent_id

                        all_active = all(f"{group_name}_{s}" in self.active_tools for s in subtools)
                        yield Checkbox(group_name, value=all_active, id=parent_id)

                        for subtool, child_id in zip(subtools, child_ids):
                            full_name = f"{group_name}_{subtool}"
                            yield Checkbox(
                                subtool,
                                value=full_name in self.active_tools,
                                id=child_id,
                                classes="group-child",
                            )

            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        cb_id = event.checkbox.id

        if cb_id in self._parent_to_children:
            # Parent toggled -> set all children to same value
            with self.prevent(Checkbox.Changed):
                for child_id in self._parent_to_children[cb_id]:
                    self.query_one(f"#{child_id}", Checkbox).value = event.value

        elif cb_id in self._child_to_parent:
            # Child toggled -> update parent (checked only when ALL children checked)
            parent_id = self._child_to_parent[cb_id]
            all_checked = all(self.query_one(f"#{cid}", Checkbox).value for cid in self._parent_to_children[parent_id])
            with self.prevent(Checkbox.Changed):
                self.query_one(f"#{parent_id}", Checkbox).value = all_checked

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            selected = [
                tool_name
                for cb_id, tool_name in self._id_to_tool.items()
                if self.query_one(f"#{cb_id}", Checkbox).value
            ]
            self.dismiss(selected)
        elif event.button.id == "cancel":
            self.dismiss(None)

    def on_mount(self) -> None:
        first_cb = self.query_one(Checkbox)
        self.set_focus(first_cb)


class RadioListModal(ModalScreen[str | None]):
    class SquareRadioButton(RadioButton):
        # BUTTON_INNER = '●'
        @property
        def _button(self) -> Content:
            button_style = self.get_visual_style("toggle--button")
            symbol = "☒" if self.value else "☐"
            return Content.assemble(
                (" ", button_style),
                (symbol, button_style),
                (" ", button_style),
            )

    CSS = """
    RadioListModal {
        align: center middle;
    }

    .dialog {
        width: 60%;
        max-width: 90%;
        height: 40%;
        padding: 1 2;
        background: $panel;
    }

    .title {
        text-align: center;
        margin-bottom: 1;
    }

    .radio-list {
        height: 1fr;
    }

    .btns {
        height: auto;
        width: 100%;
        margin-top: 1;
        padding: 0;
        content-align: center middle;
        align-horizontal: center;
    }

    .btns Button {
        padding: 0 1;
    }
    """

    def __init__(self, lines: list[str], category: str = "", input_string: str = "", default_index: int = 0) -> None:
        super().__init__()
        self.lines = lines
        self.category = category
        self.input_string = input_string
        self.default_index = default_index

    def compose(self) -> ComposeResult:
        dialog_msg = f"{self.category} '{self.input_string}' does not exist. Choose a suggestion:"
        with Vertical(classes="dialog"):
            yield Label(dialog_msg, classes="title")

            # One-select radio list
            with VerticalScroll(classes="radio-list"):
                with RadioSet(id="radio-set"):
                    for i, line in enumerate(self.lines):
                        yield self.SquareRadioButton(line, id=f"rb{i}", value=(i == self.default_index))

            # Buttons
            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_mount(self) -> None:
        first_rb = self.query_one(self.SquareRadioButton)
        self.set_focus(first_rb)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            radioset = self.query_one("#radio-set", RadioSet)
            selected = radioset.pressed_index
            if selected is not None:
                self.dismiss(selected)
        else:
            self.dismiss(None)
