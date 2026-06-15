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

import asyncio
import importlib
import os
import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import Mock


class _DummySentenceTransformer:
    def __init__(self, *args, **kwargs):
        pass

    def encode(self, text, convert_to_numpy=True):
        return None

    def similarity(self, a, b):
        return None


sys.modules.setdefault("sentence_transformers", types.SimpleNamespace(SentenceTransformer=_DummySentenceTransformer))


CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class _CaptureSink:
    def __init__(self):
        self.messages = []

    def write(self, msg: str, color: str = "") -> None:
        self.messages.append((msg, color))


class _FakeInput:
    def __init__(self):
        self.id = "cmd"
        self.value = ""
        self.focused = False

    def focus(self):
        self.focused = True


class TestConsoleRegressions(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        cls.console_mod = importlib.import_module("vulcanai.console.console")
        cls.logger_mod = importlib.import_module("vulcanai.console.logger")
        cls.modal_screens_mod = importlib.import_module("vulcanai.console.modal_screens")

    def setUp(self):
        self._default_logger_instance = self.logger_mod.VulcanAILogger._default_instance
        self._rich_markup_enabled = self.logger_mod.VulcanAILogger._rich_markup

    def tearDown(self):
        self.logger_mod.VulcanAILogger._default_instance = self._default_logger_instance
        self.logger_mod.VulcanAILogger._rich_markup = self._rich_markup_enabled

    def test_tools_command_escapes_tool_summaries(self):
        """`/tools` should escape tool descriptions that contain markup-like tokens."""
        console = self.console_mod.VulcanConsole(default_tools=False)
        sink = _CaptureSink()
        console.logger = self.logger_mod.VulcanAILogger(sink=sink)

        topic_tool = SimpleNamespace(
            name="ros2_topic_type",
            tool_description="Equivalent to `ros2 topic type [topic_name]`.",
            description="fallback",
        )
        console.manager = SimpleNamespace(
            k=7,
            registry=SimpleNamespace(
                tools={"ros2_topic_type": topic_tool},
                group_tool_names=lambda names: [("ros2_topic", ["type"])],
            ),
        )
        console.commands = {"/tools": console.cmd_tools}

        console.handle_command("/tools")

        rendered = "\n".join(msg for msg, _ in sink.messages)
        self.assertNotIn("Error:", rendered)
        self.assertIn(r"\[topic_name]", rendered)

    async def test_on_input_submitted_removes_angle_brackets_from_queries(self):
        """User queries should be sanitized before echo/history/task scheduling."""
        console = self.console_mod.VulcanConsole(default_tools=False)
        console.is_ready = True
        console.logger = Mock()
        console._update_history_panel = Mock()
        console._is_stream_task_active = Mock(return_value=False)
        console._route_logs_to_stream_panel = 0

        input_widget = _FakeInput()
        console.query_one = Mock(return_value=input_widget)

        captured = {}

        async def fake_queriestrap(user_input):
            captured["query"] = user_input

        console.queriestrap = fake_queriestrap

        raw_query = "show topic </chatter> <>"
        event = SimpleNamespace(value=raw_query, input=input_widget)

        await console.on_input_submitted(event)
        await asyncio.sleep(0)

        self.assertIn("query", captured)
        self.assertNotIn("<", captured["query"])
        self.assertNotIn(">", captured["query"])
        self.assertEqual(console.history[-1], captured["query"])
        console.logger.log_user.assert_called_once_with(captured["query"])
        self.assertEqual(input_widget.value, "")
        self.assertTrue(input_widget.focused)

    async def test_modal_enter_binding(self):
        """Suggestion modal should accept the highlighted item when Enter is used."""
        modal = self.modal_screens_mod.RadioListModal(["alpha", "beta", "gamma"])
        modal.dismiss = Mock()
        modal.query_one = Mock(return_value=SimpleNamespace(pressed_index=0, _selected=2))

        modal.dismiss_selected()

        modal.dismiss.assert_called_once_with(2)
        bindings = {
            (binding.key, binding.action)
            for binding in self.modal_screens_mod.RadioListModal.SuggestionRadioSet.BINDINGS
        }
        self.assertIn(("enter", "submit_selection"), bindings)
        self.assertIn(("space", "toggle_button"), bindings)


if __name__ == "__main__":
    unittest.main()
