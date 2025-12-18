from collections import defaultdict
import re
import pyperclip

from rich.style import Style
from textual.app import App, ComposeResult
from textual.widgets import TextArea


class CustomLogTextArea(TextArea):
    """
    TextArea-based Log Panel. Includes the following features:
     - Selectable text.
     - Color text.

    NOTE: Uses private TextArea internals (_highlights, _theme).
    This may break on future Textual versions.
    """

    BINDINGS = [
        ("f4", "copy_selection", "Copy selection"),
    ]

    # <tag>body</tag>
    TAG_RE = re.compile(r"<(?P<tag>[A-Za-z0-9_#]+)>(?P<body>.*?)</(?P=tag)>")

    def __init__(self, **kwargs):
        super().__init__(read_only=True, **kwargs)

        # Internal variable used by the father class (TextArea)
        #  to change the colors of the lines in the Log.
        # When the function self.refresh() is called, all the styles
        #  are refreshed (inlcudes: colors, bold, italic, ...)
        self._highlights = defaultdict(list)

        # Private variable of the number of lines
        #  that are currently being displayed
        self._line_count = 0

        # Private list with all the style for the lines
        #  A row with N styles # (e.g.: N = 2: two colors, one bold and color)
        #  will have N entries.
        self._lines_styles = []

    # region UTILS

    def _rebuild_highlights(self) -> None:
        """
        When a new line is inputed in the CustomLogTextArea,
        by default, it is the only line highlighted with a stlye.
        To keep the styles of the previous lines, a rebuild must
        be executed.
        """

        # Clears the previous
        self._highlights.clear()

        # Substring span styles
        for row, start, end, token in self._lines_styles:
            self._highlights[row].append((start, end, token))

    def _ensure_style_token(self, style: str) -> str:
        """
        Convert a style string like:
        "red bold italic"
        "#49E63B bold"
        into a registered TextArea style token.
        """
        style_list = style.split()

        color = None
        bold = False
        italic = False
        underline = False
        dim = False

        token_parts = []

        for st in style_list:
            if st.startswith("#"):
                color = st
                token_parts.append("hex_" + st[1:].upper())
            elif st in {"bold", "italic", "underline", "dim"}:
                token_parts.append(st)
                if st == "bold":
                    bold = True
                elif st == "italic":
                    italic = True
                elif st == "underline":
                    underline = True
                elif st == "dim":
                    dim = True
            else:
                # assume named color like "red", "yellow"
                color = st
                token_parts.append(st)

        token = "_".join(token_parts)

        # Register the style if it doesn't exist
        if token not in self._theme.syntax_styles:
            self._theme.syntax_styles[token] = Style(
                color=color,
                bold=bold,
                italic=italic,
                underline=underline,
                dim=dim,
            )

        return token

    # endregion

    # region LOG

    def append_line(self, text: str, style: str = "white") -> None:
        self.append_marked(f"<{style}>{text}</{style}>")

    def append_marked(self, marked: str) -> None:
        plain = ""
        spans = []  # (start_col, end_col, token)
        cursor = 0

        for m in self.TAG_RE.finditer(marked):
            plain += marked[cursor:m.start()]

            tag = m.group("tag")
            body = m.group("body")

            start = len(plain)
            plain += body
            end = len(plain)

            token = self._ensure_style_token(tag)
            spans.append((start, end, token))

            cursor = m.end()

        plain += marked[cursor:]

        # Append via document API to keep row tracking consistent
        insert_text = ("" if not self.document.text or self.document.text.endswith("\n") else "\n")
        insert_text += plain + "\n"
        self.insert(insert_text, location=self.document.end)

        # Record spans for THIS row
        row = self._line_count
        self._line_count += 1
        for start, end, token in spans:
            self._lines_styles.append((row, start, end, token))

        self._rebuild_highlights()
        self.refresh()

    # endregion

    def clear(self) -> None:
        """
        Clear the CustomLogTextArea
        """

        """# Clear using editor API
        self.delete((0, 0), self.document.end)"""

        self._highlights.clear()

        self._line_count = 0
        self._lines_styles.clear()

        self.refresh()

    def action_copy_selection(self) -> None:
        """
        Action: Copies the selected text of the CustomLogTextArea
        """

        # "self.selected_text" is an inherence variable
        #   from the father TextArea
        pyperclip.copy(self.selected_text)

        self.notify("Selected area copied to clipboard!")


class ConsoleDemo(App):

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
    }

    #logview {
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
        ("i", "log_info", "Log info"),
        ("w", "log_warn", "Log warn"),
        ("e", "log_error", "Log error"),
        ("ctrl+l", "clear", "Clear"),
    ]


    def __init__(self):
        super().__init__() # Textual lib

        self.left_pannel = None

    def compose(self) -> ComposeResult:
        from textual.containers import Horizontal, Vertical, VerticalScroll
        from textual.widgets import Static, Input
        from textual.widgets import Input, TextArea


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
                yield CustomLogTextArea(id="logcontent") # TODO. here
                yield Input(placeholder="> ", id="cmd")

            # RIGHT
            with Vertical(id="right"):
                yield Static(vulcanai_title_slant, id="history_title")
                yield Static(f" AI model: OpenAi\n K = 7\n history_depth = 10", id="variables")
                with VerticalScroll(id="history_scroll"):
                    # IMPORTANT: markup=True so [bold reverse] works
                    yield TextArea("", id="history", read_only=True) # TODO. here

    def on_mount(self) -> None:
        self.left_pannel = self.query_one("#logcontent", CustomLogTextArea)
        self.left_pannel.append_line("ready", style="green")

    def action_log_info(self) -> None:
        #self.left_pannel.append_line("INFO connected", style="#4AD428")
        self.left_pannel.append_marked("<green>INFO</green> <yellow>connected</yellow>")

    def action_log_warn(self) -> None:
        self.left_pannel.append_line("WARN retrying", style="yellow")
        self.left_pannel.append_line("(extra message) WARN retrying")

    def action_log_error(self) -> None:
        self.left_pannel.append_line("ERROR timeout", style="red")

    def action_clear(self) -> None:
        self.left_pannel.clear()

    def run_console(self) -> None:
        """
        Run function for the VulcanAI
        """

        self.run()


if __name__ == "__main__":
    console = ConsoleDemo()

    console.run_console()
