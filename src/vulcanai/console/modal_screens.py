from textual.app import ComposeResult
from textual.widgets import Input, Checkbox, Button, Label, RadioSet, RadioButton
from textual import events
from textual.containers import VerticalScroll, Horizontal, Vertical, Container


# checkbox
from textual.screen import ModalScreen



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

    .btns {
        height: 3;            /* give buttons row a fixed height */
        padding-top: 1;
        content-align: right middle;
    }
    """

    def __init__(self, lines: list[str], active_tools_num: int = 0) -> None:
        super().__init__()
        self.lines = list(lines)
        self.active_tools_num = active_tools_num

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Label("Pick tools you want to enable", classes="title")

            # SCROLLABLE CHECKBOX LIST
            with VerticalScroll(classes="checkbox-list"):
                for i, line in enumerate(self.lines, start=1):
                    yield Checkbox(line, value=i <= self.active_tools_num, id=f"cb{i}")

            # Buttons
            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            boxes = list(self.query(Checkbox))
            selected = [self.lines[i] for i, cb in enumerate(boxes) if cb.value]
            self.dismiss(selected)
        elif event.button.id == "cancel":
            self.dismiss(None)

    def on_mount(self) -> None:
        first_cb = self.query_one(Checkbox)
        self.set_focus(first_cb)




class RadioListModal(ModalScreen[str | None]):

    CSS = """
    RadioListModal {
        align: center middle;
    }

    .dialog {
        width: 60%;
        max-width: 90%;
        height: 40%;
        border: round $accent;
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
        height: 3;
        padding-top: 1;
        content-align: right middle;
    }
    """

    def __init__(self, lines: list[str], default_index: int = 0) -> None:
        super().__init__()
        self.lines = lines
        self.default_index = default_index

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Label("Pick one option", classes="title")

            # One-select radio list
            with VerticalScroll(classes="radio-list"):
                with RadioSet(id="radio-set"):
                    for i, line in enumerate(self.lines):
                        yield RadioButton(
                            line,
                            id=f"rb{i}",
                            value=(i == self.default_index)
                        )

            # Buttons
            with Horizontal(classes="btns"):
                yield Button("Cancel", variant="default", id="cancel")
                yield Button("Submit", variant="primary", id="submit")

    def on_mount(self) -> None:
        first_rb = self.query_one(RadioButton)
        self.set_focus(first_rb)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "submit":
            radioset = self.query_one("#radio-set", RadioSet)
            selected = radioset.pressed_index
            if selected != None:
                self.dismiss(selected)
        else:
            self.dismiss(None)