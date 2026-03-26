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

import os
import sys
import unittest
from unittest.mock import patch

from vulcanai.console.widget_custom_log_text_area import CustomLogTextArea

# Make src-layout importable
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class TestCustomLogTextArea(unittest.TestCase):
    def setUp(self):
        self.widget = CustomLogTextArea()

    def test_join_nested_tags(self):
        text = "<red>first <bold>second</bold> third</red>"
        expected = "<red>first </red><red bold>second</red bold><red> third</red>"
        self.assertEqual(self.widget.join_nested_tags(text), expected)

    def test_append_line_stores_plain_text_and_styles(self):
        result = self.widget.append_line("prefix <red>error</red> suffix")
        self.assertTrue(result)
        self.assertEqual(self.widget.document.text, "prefix error suffix")
        self.assertEqual(list(self.widget._lines_styles)[0], [(7, 12, "red")])
        self.assertEqual(self.widget._highlights[0], [(7, 12, "red")])

    def test_append_line_rejects_multiline_input(self):
        self.assertFalse(self.widget.append_line("line 1\nline 2"))
        self.assertEqual(self.widget.document.text, "")
        self.assertEqual(self.widget._line_count, 0)
        self.assertEqual(len(self.widget._lines_styles), 0)

    def test_append_line_trims_max_lines_and_reindexes_highlights(self):
        max_lines = self.widget.MAX_LINES
        extra_lines = 5

        for idx in range(max_lines + extra_lines):
            if idx % 100 == 0:
                self.widget.append_line(f"<red>line-{idx}</red>")
            else:
                self.widget.append_line(f"line-{idx}")

        self.assertEqual(self.widget._line_count, max_lines)
        self.assertEqual(len(self.widget.document.text.splitlines()), max_lines)
        self.assertEqual(self.widget.document.get_line(0), f"line-{extra_lines}")
        self.assertEqual(self.widget.document.get_line(max_lines - 1), f"line-{max_lines + extra_lines - 1}")

        kept_colored_indexes = [i for i in range(max_lines + extra_lines) if i % 100 == 0 and i >= extra_lines]
        expected_rows = [i - extra_lines for i in kept_colored_indexes]
        self.assertEqual(sorted(self.widget._highlights.keys()), expected_rows)

        for original_idx, row in zip(kept_colored_indexes, expected_rows):
            self.assertEqual(self.widget._highlights[row], [(0, len(f"line-{original_idx}"), "red")])

    def test_delete_last_row(self):
        self.widget.append_line("first")
        self.widget.append_line("second")

        self.widget.delete_last_row()
        self.assertEqual(self.widget.document.text, "first")
        self.assertEqual(self.widget._line_count, 1)

        self.widget.delete_last_row()
        self.assertEqual(self.widget.document.text, "")
        self.assertEqual(self.widget._line_count, 0)

        self.widget.delete_last_row()
        self.assertEqual(self.widget.document.text, "")
        self.assertEqual(self.widget._line_count, 0)

    def test_clear_console(self):
        self.widget.append_line("<red>first</red>")
        self.widget.append_line("second")

        self.widget.clear_console()
        self.assertEqual(self.widget.document.text, "")
        self.assertEqual(self.widget._line_count, 0)
        self.assertEqual(len(self.widget._lines_styles), 0)
        self.assertEqual(dict(self.widget._highlights), {})

    def test_action_copy_selection_no_selected_text(self):
        notifications = []
        self.widget.notify = notifications.append

        with patch("vulcanai.console.widget_custom_log_text_area.pyperclip.copy") as clipboard_copy:
            self.widget.action_copy_selection()

        clipboard_copy.assert_not_called()
        self.assertEqual(notifications, ["No text selected to copy!"])

    def test_action_copy_selection_with_selected_text(self):
        self.widget.append_line("copy this")
        self.widget.select_all()

        notifications = []
        self.widget.notify = notifications.append

        with patch("vulcanai.console.widget_custom_log_text_area.pyperclip.copy") as clipboard_copy:
            self.widget.action_copy_selection()

        clipboard_copy.assert_called_once_with("copy this")
        self.assertEqual(notifications, ["Selected area copied to clipboard!"])
