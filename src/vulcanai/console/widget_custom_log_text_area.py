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


from collections import defaultdict
import pyperclip
import re
import threading

from rich.style import Style
from textual.widgets import TextArea

from collections import deque


class CustomLogTextArea(TextArea):
    """
    TextArea-based Log Panel. Includes the following features:
     - Selectable text.
     - Color text.

    NOTE: Uses private TextArea internals (_highlights, _theme).
    This may break on future Textual versions.
    """

    BINDINGS = [
        ("f3", "copy_selection", "Copy selection"),
    ]

    # Maximum number of lines to keep in the log
    MAX_LINES = 500

    # <tag>body</tag>
    TAG_RE = re.compile(r"<(?P<tag>[A-Za-z0-9_# ]+)>(?P<body>.*?)</(?P=tag)>")
    # join tags
    TAG_TOKEN_RE = re.compile(r"</?[^>]+>")


    def __init__(self, **kwargs):
        super().__init__(read_only=True, **kwargs)

        # Lock used to avoid data races in 'self._lines_styles'
        #   when VulcanAI and ROS threads writes at the same time
        self._lock = threading.Lock()

        # Internal variable used by the father class (TextArea)
        #  to change the colors of the lines in the Log.
        # When the function self.refresh() is called, all the styles
        #  are refreshed (includes: colors, bold, italic, ...)
        self._highlights = defaultdict(list)

        # Private variable of the number of lines
        #  that are currently being displayed
        self._line_count = 0

        # Private dictionary (int, list()) with all the style for the lines
        #  A row with N styles # (e.g.: N = 2: two colors, one bold and color)
        #  will have N entries.
        self._lines_styles = deque(maxlen=self.MAX_LINES)

    # region UTILS

    def _trim_highlights(self) -> None:
        """
        Function used to trim the CustomLogTextArea to the maximum number of lines.

        Keep only the last MAX_LINES lines (text + styles).
        """
        if self._line_count <= self.MAX_LINES:
            # Less than max lines, nothing to do
            return

        # MAX_LINES exceeded, need to trim

        # Calculate how many lines to remove
        extra = self._line_count - self.MAX_LINES

        # 1) Remove the first 'extra' lines from the TextArea document.
        #    end=(extra, 0) means "start of line 'extra'", so it deletes lines [0..extra-1]
        self.replace("", start=(0, 0), end=(extra, 0))

        # 3) Update line count
        self._line_count -= extra

    def _rebuild_highlights(self) -> None:
        """
        Function used to rebuild the highlights of the TextArea based on the current
        _lines_styles mapping.

        Each time a new line is added, or a line is modified, this function should be called
        to update the _highlights variable used by the TextArea to render the styles.
        """

        # Clears the previous highlights
        self._highlights.clear()

        # Rebuild highlights from _lines_styles
        for row, value_list in enumerate(self._lines_styles):
            for tuple in value_list:
                self._highlights[row].append((tuple[0], tuple[1], tuple[2]))

    def join_nested_tags(self, input_text: str) -> str:
        """
        Function used to join nested tags into combined tags.

        E.g.:
        Input:  "<red>This is <bold>important</bold> text</red>"
        Output: "<red bold>This is important text</red bold>"
        """
        out: list[str] = []
        stack: list[str] = []

        pos = 0
        # Iterate over all tags in 'input_text'
        for m in self.TAG_TOKEN_RE.finditer(input_text):
            # Emit text between tags
            text = input_text[pos:m.start()]
            if text:
                if stack:
                    combined = " ".join(stack)
                    out.append(f"<{combined}>{text}</{combined}>")
                else:
                    out.append(text)

            # Process tag
            token = m.group(0)
            is_close = token.startswith("</")
            name = token[2:-1].strip() if is_close else token[1:-1].strip()

            # Check stack
            if is_close:
                # Pop matching tag (tolerant: remove last occurrence)
                for i in range(len(stack) - 1, -1, -1):
                    if stack[i] == name:
                        del stack[i]
                        break
            else:
                stack.append(name)

            pos = m.end()

        # Tail text after last tag
        tail = input_text[pos:]
        if tail:
            if stack:
                combined = " ".join(stack)
                out.append(f"<{combined}>{tail}</{combined}>")
            else:
                out.append(tail)

        return "".join(out)

    def _ensure_style_token(self, style: str) -> str:
        """
        Function used to ensure a style token exists in the theme.
        If the style does not exist, it is created.

        E.g.:
        style: style string (e.g., "red bold", "#FF0000 italic", "green underline", ...)
        """
        style_list = style.split()

        color = None
        bold = False
        italic = False
        underline = False
        dim = False

        token_parts = []

        # Iterate over style parts
        for st in style_list:
            if st.startswith("#"):
                # Hex color
                color = st
                token_parts.append("hex_" + st[1:].upper())
            elif st in {"bold", "italic", "underline", "dim"}:
                # Text style
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
                # Other colors

                # Gray color is not supported
                if st == "gray":
                    color = "#8D8D8D"
                    token_parts.append("hex_" + color[1:])
                else:
                    # Assume named color like "red", "yellow"
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

    def append_line(self, text: str) -> None:
        """
        Function used to append a new line to the CustomLogTextArea.

        The 'text' parameter can include tags to specify styles.
        E.g.:
        "<red>This is red text</red>"
        "<green bold>This is green bold text</green bold>"
        """

        # Check if the input text has a breakline
        if "\n" in text or "\r" in text:
            # Normalize line endings and split
            lines = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
            for line in lines:
                self.append_line(line)
            return

        # 'text' without tags
        plain = ""
        # Spans -> Multiple styles (start_col, end_col, token)
        spans = []
        # Position cursor in 'text'
        cursor = 0

        # First, join nested tags into combined tags
        # for easier parsing.
        text = self.join_nested_tags(text)

        # Parse tags and build plain text + 'spans'
        # TAG_RE.finditer finds all tags in 'text'
        for m in self.TAG_RE.finditer(text):
            # Append text before the current tag
            plain += text[cursor:m.start()]
            # Get tag and body
            tag = m.group("tag")
            body = m.group("body")
            # Record span
            start = len(plain)
            # Append body in plain text
            plain += body
            end = len(plain)
            # Get or create style token
            token = self._ensure_style_token(tag)
            spans.append((start, end, token))
            # Move cursor after the current tag
            cursor = m.end()

        # Append remaining text after last tag
        plain += text[cursor:]

        # Append the new line with styles
        # Use a lock to avoid race conditions when multiple threads
        #   try to write to the console at the same time.
        # This can happen when VulcanAI and ROS nodes are logging simultaneously.
        #
        # e.g., VulcanAI manager logging and ROS listener logging.
        # [EXECUTOR] Invoking 'move_turtle' with args: ...
        # [ROS] [INFO] Publishing message 1 to ...
        with self._lock:

            # Append via document API to keep row tracking consistent
            # Only add a newline before the new line if there is already content
            insert_text = ("\n" if self.document.text else "") + plain
            self.insert(insert_text, location=self.document.end)

            # Track styles for the new line (always at the end)
            row = self._line_count
            self._line_count += 1

            if (self._line_count > self.MAX_LINES):
                self._highlights.pop(self._line_count - self.MAX_LINES, None)

            # Store styles
            # Each line may have multiple styles (spans)
            current_line = []
            for start, end, token in spans:
                current_line.append((start, end, token))

            self._lines_styles.append(current_line)

            # Trim now
            self._trim_highlights()

            # Scroll to end
            self.scroll_end(animate=False)

            # Rebuild highlights and refresh
            self._rebuild_highlights()
            self.refresh()

    def delete_last_row(self) -> None:
        """
        Function used to delete the last line in the CustomLogTextArea.
        Used when a AI query is completed. To remove the "Querying..." line.
        """
        with self._lock:
            if self._line_count == 0:
                # No lines, does nothing.
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

            # Decrease line count and remove styles
            self._line_count -= 1
            self._lines_styles.pop()

            # Rebuild highlights and refresh
            self._rebuild_highlights()
            self.refresh()

    # endregion

    def clear_console(self) -> None:
        """
        Function used to clear the entire CustomLogTextArea.
        """

        with self._lock:
            # Clear internal structures
            self._highlights.clear()

            # Clear the document
            self._line_count = 0
            self._lines_styles.clear()

            # Refresh and clear the TextArea
            self.refresh()
            self.clear()

    def action_copy_selection(self) -> None:
        """
        Action: Copies the selected text of the CustomLogTextArea
        """

        # Check if there is selected text
        if self.selected_text == "":
            self.notify("No text selected to copy!")
            return

        # Copy to clipboard, using pyperclip library
        pyperclip.copy(self.selected_text)
        self.notify("Selected area copied to clipboard!")
