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
    TAG_RE = re.compile(r"<(?P<tag>[A-Za-z0-9_# ]+)>(?P<body>.*?)</(?P=tag)>")
    # join tags
    TAG_TOKEN_RE = re.compile(r"</?[^>]+>")

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
        self._lines_styles = {}

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

        for row, value_list in self._lines_styles.items():
            for tuple in value_list:
                self._highlights[row].append((tuple[0], tuple[1], tuple[2]))

    def join_nested_tags(self, marked: str) -> str:
        """
        Convert nested tags.

        example:
            <bold><green>INFO</green></bold>
            <bold green>INFO</bold green>
        """
        ret = []
        tag_stack = []

        pos = 0
        # Find all tags
        for m in self.TAG_TOKEN_RE.finditer(marked):
            # Emit text between tags
            text = marked[pos:m.start()]
            if text:
                if tag_stack:
                    combined = " ".join(tag_stack)
                    ret.append(f"<{combined}>{text}</{combined}>")
                else:
                    ret.append(text)

            token = m.group(0)
            is_close = token.startswith("</")
            name = token[2:-1].strip() if is_close else token[1:-1].strip()

            if is_close:
                # Remove closing tag
                # Remove last occurrence
                for i in range(len(tag_stack) - 1, -1, -1):
                    if tag_stack[i] == name:
                        del tag_stack[i]
                        break
            else:
                # Add opening tag
                tag_stack.append(name)

            pos = m.end()

        # Tail text after last tag
        tail = marked[pos:]
        if tail:
            if tag_stack:
                combined = " ".join(tag_stack)
                ret.append(f"<{combined}>{tail}</{combined}>")
            else:
                ret.append(tail)

        return "".join(ret)

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
                # gray color is not supported
                if st == "gray":
                    color = "#8D8D8D"
                    token_parts.append("hex_" + color[1:])
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

        marked = self.join_nested_tags(marked)

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
        # Only add a newline before the new line if there is already content
        insert_text = ("\n" if self.document.text else "") + plain
        self.insert(insert_text, location=self.document.end)

        row = self._line_count
        self._line_count += 1

        self._lines_styles[row] = []
        for start, end, token in spans:
            self._lines_styles[row].append((start, end, token))

        # Move the cursor at the end of the terminal
        # to apply autoscroll when writting a line
        self.move_cursor(self.document.end)

        self._rebuild_highlights()
        self.refresh()

    def replace_row(self, marked: str, row: int) -> None:
        """
        Function used to replace a specific row in the CustomLogTextArea.
        Only used in the updated() function of the query SpinnerHook.
        """
        plain = ""
        spans = []  # (start_col, end_col, token)
        cursor = 0

        if row == -1:
            row = self._line_count - 1

        marked = self.join_nested_tags(marked)

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

        if row < self._line_count - 1:
            # It is not the last line, replace up to (row+1, 0)
            self.replace(plain + "\n", start=(row, 0), end=(row + 1, 0))
        else:
            # If it is the last line, replace up to end of that line
            old = self.document.get_line(row)
            self.replace(plain, start=(row, 0), end=(row, len(old)))

        self._lines_styles[row] = []
        for start, end, token in spans:
            self._lines_styles[row].append((start, end, token))

        self._rebuild_highlights()
        self.refresh()

    def delete_last_row(self) -> None:
        """
        Function used to delete the last row in the CustomLogTextArea.

        Only used in the end() function of the query SpinnerHook.
        """

        if self._line_count == 0:
            return

        last_row = self._line_count - 1

        if last_row > 0:
            # Delete the newline before the last line + the line itself
            self.replace(
                "",
                start=(last_row - 1, len(self.document.get_line(last_row - 1))),
                end=(last_row, len(self.document.get_line(last_row))),
            )
        else:
            # Only one line
            self.replace(
                "",
                start=(0, 0),
                end=(0, len(self.document.get_line(0))),
            )

        self._line_count -= 1
        self._lines_styles.pop(last_row, None)

        self._rebuild_highlights()
        self.refresh()

    # endregion

    def clear_console(self) -> None:
        """
        Clear the CustomLogTextArea
        """

        """# Clear using editor API
        self.delete((0, 0), self.document.end)"""

        self._highlights.clear()

        self._line_count = 0
        self._lines_styles.clear()

        self.refresh()
        self.clear()

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
                yield CustomLogTextArea(id="logcontent")
                yield Input(placeholder="> ", id="cmd")

            # RIGHT
            with Vertical(id="right"):
                yield Static(vulcanai_title_slant, id="history_title")
                yield Static(f" AI model: OpenAi\n K = 7\n history_depth = 10", id="variables")
                #with VerticalScroll(id="history_scroll"):
                yield TextArea("", id="history", read_only=True)

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
        self.left_pannel.clear_console()

    def run_console(self) -> None:
        """
        Run function for the VulcanAI
        """

        self.run()


if __name__ == "__main__":
    console = ConsoleDemo()

    console.run_console()
