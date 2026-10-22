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

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from textual.app import App
from textual.widgets import Input

from vulcanai.console.console import VulcanConsole


class ReloadHarness(App):
    cmd_reload_tools = VulcanConsole.cmd_reload_tools
    _reload_registered_tools = VulcanConsole._reload_registered_tools
    set_input_enabled = VulcanConsole.set_input_enabled

    def __init__(self):
        super().__init__()
        self.register_from_file = ["tools.py"]
        self.manager = SimpleNamespace(registry=Mock())
        self.logger = Mock()

    def compose(self):
        yield Input(id="cmd")

    def on_mount(self):
        self.query_one(Input).focus()

    def _is_stream_task_active(self):
        return False


class TestConsoleReload(unittest.IsolatedAsyncioTestCase):
    async def test_reload_restores_input_and_reports_success(self):
        app = ReloadHarness()
        app.manager.registry.reload_tools_from_files.return_value = {"files": 1, "updated": ["move"], "removed": []}
        async with app.run_test():
            app.cmd_reload_tools([])
            self.assertTrue(app.query_one(Input).disabled)
            await app.workers.wait_for_complete()
            self.assertFalse(app.query_one(Input).disabled)
            app.manager.registry.reload_tools_from_files.assert_called_once_with(["tools.py"])
            app.logger.log_console.assert_called_once_with("Reloaded 1 tools from 1 files.", color="console")

    async def test_failure_restores_input_and_reports_error(self):
        app = ReloadHarness()
        app.manager.registry.reload_tools_from_files.side_effect = SyntaxError("Invalid source")
        async with app.run_test():
            app.cmd_reload_tools([])
            await app.workers.wait_for_complete()
            self.assertFalse(app.query_one(Input).disabled)
            self.assertIn("registered tools kept", app.logger.log_msg.call_args.args[0])
