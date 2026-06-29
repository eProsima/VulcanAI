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
from unittest.mock import Mock, PropertyMock, patch

from textual.geometry import Offset


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
        cls.log_text_area_mod = importlib.import_module("vulcanai.console.widget_custom_log_text_area")

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

    def test_log_text_area_keeps_following_output_when_sticky_follow_is_enabled(self):
        """Appending output should stay anchored even if a one-shot end check would fail."""
        widget = self.log_text_area_mod.CustomLogTextArea()
        widget.is_near_vertical_scroll_end = Mock(return_value=False)
        widget.scroll_end = Mock()
        widget.scroll_to = Mock()
        widget.call_after_refresh = Mock()
        widget.refresh = Mock()

        widget.append_line("hello")

        self.assertEqual(widget.document.text, "hello")
        self.assertTrue(widget.should_follow_output())
        widget.scroll_end.assert_called_once_with(animate=False, immediate=True, x_axis=False)
        widget.scroll_to.assert_not_called()
        widget.call_after_refresh.assert_called_once_with(widget._scroll_to_output_end)

    def test_log_text_area_preserves_viewport_when_follow_is_disabled(self):
        """Appending output should not yank the viewport when the user scrolled up."""
        widget = self.log_text_area_mod.CustomLogTextArea()
        widget._follow_output = False
        widget.scroll_end = Mock()
        widget.scroll_to = Mock()
        widget.call_after_refresh = Mock()
        widget.refresh = Mock()

        with patch.object(type(widget), "scroll_offset", new_callable=PropertyMock, return_value=Offset(3, 7)):
            widget.append_line("hello")

        self.assertFalse(widget.should_follow_output())
        widget.scroll_end.assert_not_called()
        widget.scroll_to.assert_called_once_with(x=3, y=7, animate=False, immediate=True, force=True)
        widget.call_after_refresh.assert_called_once_with(widget._restore_scroll_position, 3, 7)

    def test_log_text_area_follow_state_updates_when_user_moves_viewport(self):
        """Returning to the end should resume sticky follow mode."""
        widget = self.log_text_area_mod.CustomLogTextArea()
        widget._follow_output = False
        widget.is_near_vertical_scroll_end = Mock(return_value=True)

        widget._update_follow_output_from_viewport()
        self.assertTrue(widget.should_follow_output())

        widget.is_near_vertical_scroll_end.return_value = False
        widget._update_follow_output_from_viewport()
        self.assertFalse(widget.should_follow_output())

    def test_cmd_edit_tools_passes_default_tool_subset_to_modal(self):
        """`/edit_tools` should expose built-in default tools to the checklist modal."""

        class DefaultTool:
            pass

        class CustomTool:
            pass

        DefaultTool.__module__ = "vulcanai.tools.default_tools"
        CustomTool.__module__ = "demo.custom_tools"

        default_active = DefaultTool()
        default_inactive = DefaultTool()
        custom_tool = CustomTool()

        tool_lookup = {
            "ros2_topic_list": default_active,
            "ros2_node_info": default_inactive,
            "custom_tool": custom_tool,
        }
        registry = SimpleNamespace(
            tools={"ros2_topic_list": default_active, "custom_tool": custom_tool},
            deactivated_tools={"ros2_node_info": default_inactive},
            group_tool_names=Mock(return_value=[("custom_tool", None)]),
            _get_tool_by_name=lambda name: tool_lookup[name],
        )

        console = self.console_mod.VulcanConsole(default_tools=False)
        console.manager = SimpleNamespace(registry=registry)
        console.open_checklist = Mock()

        console.cmd_edit_tools([])

        registry.group_tool_names.assert_called_once_with(["custom_tool", "ros2_node_info", "ros2_topic_list"])
        console.open_checklist.assert_called_once_with(
            [("custom_tool", None)],
            {"ros2_topic_list", "custom_tool"},
            {"ros2_topic_list", "ros2_node_info"},
        )

    def test_tool_toggle_log_message_marks_default_tools(self):
        """Activation logs should label built-in tools as default tools."""

        class DefaultTool:
            pass

        class CustomTool:
            pass

        DefaultTool.__module__ = "vulcanai.tools.default_tools"
        CustomTool.__module__ = "demo.custom_tools"

        registry = SimpleNamespace(
            _get_tool_by_name=lambda name: {
                "ros2_topic_list": DefaultTool(),
                "custom_tool": CustomTool(),
            }.get(name)
        )

        console = self.console_mod.VulcanConsole(default_tools=False)
        console.manager = SimpleNamespace(registry=registry)

        self.assertEqual(
            console._tool_toggle_log_message("ros2_topic_list", activated=True),
            "Activated default tool 'ros2_topic_list'",
        )
        self.assertEqual(
            console._tool_toggle_log_message("ros2_topic_list", activated=False),
            "Deactivated default tool 'ros2_topic_list'",
        )
        self.assertEqual(
            console._tool_toggle_log_message("custom_tool", activated=True),
            "Activated tool 'custom_tool'",
        )

    def test_add_line_uses_sticky_follow_state_for_main_log(self):
        """Main log writes should rely on the shared sticky follow state."""
        console = self.console_mod.VulcanConsole(default_tools=False)
        console.main_pannel = SimpleNamespace(
            should_follow_output=Mock(return_value=True),
            append_line=Mock(return_value=True),
        )
        console.stream_pannel = None
        console.logger = Mock()
        console._route_logs_to_stream_panel = 0
        console._stream_panel_visible = False
        console._is_stream_task_active = Mock(return_value=False)

        console.add_line("hello")

        console.main_pannel.should_follow_output.assert_called_once_with()
        console.main_pannel.append_line.assert_called_once_with("hello", force_follow_output=True)


if __name__ == "__main__":
    unittest.main()
