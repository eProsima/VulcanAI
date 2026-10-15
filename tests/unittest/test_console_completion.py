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

import unittest
from unittest.mock import Mock

from textual.app import App
from textual.widgets import Input

from vulcanai.console.console import VulcanConsole
from vulcanai.console.ros2_completion import Ros2Completer


class CompletionHarness(App):
    on_key = VulcanConsole.on_key
    _complete_ros2_input = VulcanConsole._complete_ros2_input

    def __init__(self):
        super().__init__()
        self.commands = {"/tools": None}
        self.tab_matches = []
        self.tab_index = 0
        self.logger = Mock()
        self.ros2_completer = Ros2Completer(lambda args: "/reset\n/spawn\n")

    def compose(self):
        yield Input(id="cmd")

    def on_mount(self):
        self.query_one(Input).focus()


class TestConsoleCompletion(unittest.IsolatedAsyncioTestCase):
    async def test_tab_lists_choices_with_user_echo_and_purple_color(self):
        app = CompletionHarness()
        async with app.run_test() as pilot:
            field = app.query_one(Input)
            field.value = "ros2 service call /"
            field.cursor_position = len(field.value)
            await pilot.press("tab")
            await app.workers.wait_for_complete()
            app.logger.log_user.assert_called_once_with("ros2 service call /")
            app.logger.log_console.assert_called_once_with("/reset\n/spawn", color="console")
            self.assertEqual(field.value, "ros2 service call /")
            self.assertIs(app.focused, field)

    async def test_tab_inserts_unique_match(self):
        app = CompletionHarness()
        async with app.run_test() as pilot:
            field = app.query_one(Input)
            field.value = "ros2 service call /res"
            field.cursor_position = len(field.value)
            await pilot.press("tab")
            await app.workers.wait_for_complete()
            self.assertEqual(field.value, "ros2 service call /reset ")

    async def test_stale_completion_does_not_overwrite_edits(self):
        app = CompletionHarness()
        async with app.run_test():
            field = app.query_one(Input)
            field.value = "ros2 service call /res"
            field.cursor_position = len(field.value)
            app._complete_ros2_input(field.value, field.cursor_position)
            field.value = "ros2 topic list"
            field.cursor_position = len(field.value)
            await app.workers.wait_for_complete()
            self.assertEqual(field.value, "ros2 topic list")
            app.logger.log_console.assert_not_called()
