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

import importlib
import os
import sys
import types
import unittest
from unittest.mock import Mock, patch


class _DummySentenceTransformer:
    def __init__(self, *args, **kwargs):
        pass

    def encode(self, text, convert_to_numpy=True):
        return None

    def similarity(self, a, b):
        return None


sys.modules.setdefault("sentence_transformers", types.SimpleNamespace(SentenceTransformer=_DummySentenceTransformer))


def _install_rclpy_stubs():
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.ok = lambda: True
    rclpy_mod.init = lambda: None
    rclpy_mod.spin_until_future_complete = lambda *args, **kwargs: None
    rclpy_mod.spin_once = lambda *args, **kwargs: None

    node_mod = types.ModuleType("rclpy.node")

    class Node:
        def __init__(self, *args, **kwargs):
            pass

    node_mod.Node = Node

    task_mod = types.ModuleType("rclpy.task")

    class Future:
        def __init__(self):
            self._cancelled = False

        def cancelled(self):
            return self._cancelled

        def cancel(self):
            self._cancelled = True

    task_mod.Future = Future

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.task"] = task_mod


CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

_install_rclpy_stubs()


class _MockLogger:
    def log_tool(self, *args, **kwargs):
        pass

    def log_msg(self, *args, **kwargs):
        pass

    def log_console(self, *args, **kwargs):
        pass


class _MockConsole:
    def __init__(self):
        self.logger = _MockLogger()
        self.stream_task = None

    def set_stream_task(self, task):
        self.stream_task = task

    def call_from_thread(self, fn, *args, **kwargs):
        fn(*args, **kwargs)

    def show_subprocess_panel(self):
        pass

    def change_route_logs(self, value):
        pass

    def add_line(self, text):
        pass

    def add_subprocess_line(self, text):
        pass


class _FakeNodeLogger:
    def warn(self, *args, **kwargs):
        pass


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeNode:
    def __init__(self):
        self.publishers = []
        self.destroyed_publishers = []
        self.logger = _FakeNodeLogger()

    def create_publisher(self, msg_type, topic_name, qos_depth):
        publisher = _FakePublisher()
        publisher.msg_type = msg_type
        publisher.topic_name = topic_name
        publisher.qos_depth = qos_depth
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed_publishers.append(publisher)

    def get_logger(self):
        return self.logger


class _NeverCancelledFuture:
    def cancelled(self):
        return False


class _StringMessage:
    def __init__(self):
        self.data = ""


class _CustomMessage:
    def __init__(self):
        self.value = None


class _DefaultToolsOptionalArgsBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.default_tools = importlib.import_module("vulcanai.tools.default_tools")

    def setUp(self):
        self.console = _MockConsole()
        self.node = _FakeNode()

    def _make_tool(self, tool_cls_name: str):
        tool_cls = getattr(self.default_tools, tool_cls_name)
        tool = tool_cls()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool


# region TOPIC


class TestStreamingToolOptionalArgs(_DefaultToolsOptionalArgsBase):
    def _assert_topic_default(
        self,
        tool_cls_name: str,
        command: str,
        omitted_arg: str,
        explicit_arg: str,
        explicit_value,
        expected_default,
    ):
        tool = self._make_tool(tool_cls_name)

        with patch.object(self.default_tools, "_run_ros2_topic_command", return_value={"output": "ok"}) as runner:
            result = tool.run(topic_name="/demo_topic", **{explicit_arg: explicit_value})

        self.assertEqual(result, {"output": "ok"})
        self.assertEqual(runner.call_args.args[:3], (self.console, tool.name, command))
        self.assertEqual(runner.call_args.kwargs["topic_name"], "/demo_topic")
        self.assertEqual(runner.call_args.kwargs[explicit_arg], explicit_value)
        self.assertEqual(runner.call_args.kwargs[omitted_arg], expected_default)

    def _assert_service_default(
        self,
        omitted_arg: str,
        explicit_arg: str,
        explicit_value,
        expected_default,
    ):
        tool = self._make_tool("Ros2ServiceEchoTool")

        with patch.object(self.default_tools, "_run_ros2_service_command", return_value={"output": "ok"}) as runner:
            result = tool.run(service_name="/demo_service", **{explicit_arg: explicit_value})

        self.assertEqual(result, {"output": "ok"})
        self.assertEqual(runner.call_args.args[:3], (self.console, tool.name, "echo"))
        self.assertEqual(runner.call_args.kwargs["service_name"], "/demo_service")
        self.assertEqual(runner.call_args.kwargs[explicit_arg], explicit_value)
        self.assertEqual(runner.call_args.kwargs[omitted_arg], expected_default)

    def test_topic_bw_defaults_max_duration_when_omitted(self):
        self._assert_topic_default("Ros2TopicBwTool", "bw", "max_duration", "max_lines", 7, 60)

    def test_topic_bw_defaults_max_lines_when_omitted(self):
        self._assert_topic_default("Ros2TopicBwTool", "bw", "max_lines", "max_duration", 3.5, 100)

    def test_topic_delay_defaults_max_duration_when_omitted(self):
        self._assert_topic_default("Ros2TopicDelayTool", "delay", "max_duration", "max_lines", 8, 60)

    def test_topic_delay_defaults_max_lines_when_omitted(self):
        self._assert_topic_default("Ros2TopicDelayTool", "delay", "max_lines", "max_duration", 4.5, 100)

    def test_topic_hz_defaults_max_duration_when_omitted(self):
        self._assert_topic_default("Ros2TopicHzTool", "hz", "max_duration", "max_lines", 9, 60)

    def test_topic_hz_defaults_max_lines_when_omitted(self):
        self._assert_topic_default("Ros2TopicHzTool", "hz", "max_lines", "max_duration", 5.5, 100)

    def test_service_echo_defaults_max_duration_when_omitted(self):
        self._assert_service_default("max_duration", "max_lines", 6, 60)

    def test_service_echo_defaults_max_lines_when_omitted(self):
        self._assert_service_default("max_lines", "max_duration", 2.5, 100)


# endregion

# region PARAM


class TestParamListOptionalArgs(_DefaultToolsOptionalArgsBase):
    def test_param_list_passes_none_when_node_name_is_omitted(self):
        tool = self._make_tool("Ros2ParamListTool")

        with patch.object(self.default_tools, "_run_ros2_param_command", return_value={"output": "ok"}) as runner:
            result = tool.run()

        self.assertEqual(result, {"output": "ok"})
        self.assertEqual(runner.call_args.args[:3], (self.console, tool.name, "list"))
        self.assertIsNone(runner.call_args.kwargs["node_name"])


# endregion

# region PUBLISH


class TestPublishOptionalArgs(_DefaultToolsOptionalArgsBase):
    def _run_publish(
        self, tool, monotonic_values, message_cls=_StringMessage, future_cls=_NeverCancelledFuture, **kwargs
    ):
        imported_types = []
        sleep_mock = Mock()

        def fake_import_msg_type(type_str, node):
            imported_types.append(type_str)
            return message_cls

        with (
            patch.object(self.default_tools, "import_msg_type", side_effect=fake_import_msg_type),
            patch.object(self.default_tools, "log_tool_in_stream_and_main"),
            patch.object(self.default_tools, "print_tool_output"),
            patch.object(self.default_tools, "_get_node_spin_lock", return_value=None),
            patch.object(self.default_tools.rclpy, "spin_once", create=True),
            patch.object(self.default_tools, "Future", future_cls),
            patch.object(self.default_tools.time, "monotonic", side_effect=monotonic_values),
            patch.object(self.default_tools.time, "sleep", sleep_mock),
        ):
            result = tool.run(**kwargs)

        publisher = self.node.publishers[-1] if self.node.publishers else None
        return result, publisher, imported_types, sleep_mock

    def test_publish_defaults_msg_type_when_omitted(self):
        tool = self._make_tool("Ros2PublishTool")

        result, _, imported_types, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0],
            topic="/demo_publish",
            message_data="hello",
            max_lines=1,
            max_duration=10.0,
            period_sec=0.0,
        )

        self.assertEqual(result["published"], "True")
        self.assertEqual(imported_types, ["std_msgs/msg/String"])

    def test_publish_defaults_message_data_when_omitted(self):
        tool = self._make_tool("Ros2PublishTool")

        result, publisher, _, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0],
            topic="/demo_publish",
            max_lines=1,
            max_duration=10.0,
            period_sec=0.0,
        )

        self.assertEqual(result["published"], "True")
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(publisher.messages[0].data, "Hello from VulcanAI PublishTool!")

    def test_publish_defaults_max_lines_when_omitted(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "max_lines": 2}

        result, publisher, _, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0, 0.1],
            topic="/demo_publish",
            message_data="hello",
            max_duration=10.0,
            period_sec=0.0,
        )

        self.assertEqual(result["count"], 2)
        self.assertEqual(len(publisher.messages), 2)

    def test_publish_defaults_max_duration_when_omitted(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "max_duration": 0.001}

        result, publisher, _, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0, 0.1],
            topic="/demo_publish",
            message_data="hello",
            max_lines=50,
            period_sec=0.0,
        )

        self.assertEqual(result["count"], 1)
        self.assertEqual(len(publisher.messages), 1)

    def test_publish_defaults_period_sec_when_omitted(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "period_sec": 0.25}

        result, publisher, _, sleep_mock = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0, 0.0, 0.0],
            topic="/demo_publish",
            message_data="hello",
            max_lines=1,
            max_duration=10.0,
        )

        self.assertEqual(result["count"], 1)
        self.assertEqual(len(publisher.messages), 1)
        sleep_mock.assert_not_called()

    def test_publish_invalid_max_duration_falls_back_to_default(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "max_duration": 0.001}

        result, publisher, _, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0, 0.1],
            topic="/demo_publish",
            message_data="hello",
            max_lines=50,
            max_duration="not_a_number",
            period_sec=0.0,
        )

        self.assertEqual(result["count"], 1)
        self.assertEqual(len(publisher.messages), 1)

    def test_publish_invalid_max_lines_falls_back_to_default(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "max_lines": 2}

        result, publisher, _, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0, 0.1],
            topic="/demo_publish",
            message_data="hello",
            max_lines="not_an_int",
            max_duration=10.0,
            period_sec=0.0,
        )

        self.assertEqual(result["count"], 2)
        self.assertEqual(len(publisher.messages), 2)

    def test_publish_negative_period_sec_falls_back_to_default(self):
        tool = self._make_tool("Ros2PublishTool")
        tool.input_defaults = {**tool.input_defaults, "period_sec": 0.25}

        result, publisher, _, sleep_mock = self._run_publish(
            tool,
            monotonic_values=[0.0] * 10,
            topic="/demo_publish",
            message_data="hello",
            max_lines=2,
            max_duration=10.0,
            period_sec=-1.0,
        )

        self.assertEqual(result["count"], 2)
        self.assertEqual(len(publisher.messages), 2)
        sleep_mock.assert_called()

    def test_publish_valid_custom_message_json_populates_message(self):
        tool = self._make_tool("Ros2PublishTool")

        result, publisher, imported_types, _ = self._run_publish(
            tool,
            monotonic_values=[0.0, 0.0],
            message_cls=_CustomMessage,
            topic="/demo_publish",
            msg_type="demo_msgs/msg/Custom",
            message_data='{"value": 42}',
            max_lines=1,
            max_duration=10.0,
            period_sec=0.0,
        )

        self.assertEqual(result["published"], "True")
        self.assertEqual(imported_types, ["demo_msgs/msg/Custom"])
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(publisher.messages[0].value, 42)


# endregion

# region SUBSCRIBE


class TestSubscribeOptionalArgs(_DefaultToolsOptionalArgsBase):
    def test_subscribe_defaults_max_duration_when_omitted(self):
        tool = self._make_tool("Ros2SubscribeTool")

        with (
            patch.object(self.default_tools, "execute_subprocess", return_value="hello\nworld") as execute,
            patch.object(self.default_tools, "print_tool_output"),
        ):
            result = tool.run(topic="/demo_topic", max_lines=2)

        self.assertEqual(result["subscribed"], "True")
        self.assertEqual(
            execute.call_args.args[:3],
            (self.console, tool.name, ["ros2", "topic", "echo", "/demo_topic", "--field", "data", "--no-arr"]),
        )
        self.assertEqual(execute.call_args.args[3], 60)
        self.assertEqual(execute.call_args.args[4], 2)

    def test_subscribe_defaults_max_lines_when_omitted(self):
        tool = self._make_tool("Ros2SubscribeTool")

        with (
            patch.object(self.default_tools, "execute_subprocess", return_value="hello") as execute,
            patch.object(self.default_tools, "print_tool_output"),
        ):
            result = tool.run(topic="/demo_topic", max_duration=2.5)

        self.assertEqual(result["subscribed"], "True")
        self.assertEqual(
            execute.call_args.args[:3],
            (self.console, tool.name, ["ros2", "topic", "echo", "/demo_topic", "--field", "data", "--no-arr"]),
        )
        self.assertEqual(execute.call_args.args[3], 2.5)
        self.assertEqual(execute.call_args.args[4], 100)

    def test_subscribe_invalid_limits_fall_back_to_defaults(self):
        tool = self._make_tool("Ros2SubscribeTool")

        with (
            patch.object(self.default_tools, "execute_subprocess", return_value="hello") as execute,
            patch.object(self.default_tools, "print_tool_output"),
        ):
            result = tool.run(topic="/demo_topic", max_duration="invalid", max_lines="invalid")

        self.assertEqual(result["subscribed"], "True")
        self.assertEqual(execute.call_args.args[3], 60)
        self.assertEqual(execute.call_args.args[4], 100)

    def test_subscribe_truncates_output_to_max_lines(self):
        tool = self._make_tool("Ros2SubscribeTool")

        with (
            patch.object(self.default_tools, "execute_subprocess", return_value="one\ntwo\nthree") as execute,
            patch.object(self.default_tools, "print_tool_output"),
        ):
            result = tool.run(topic="/demo_topic", max_duration=2.0, max_lines=2)

        self.assertEqual(result["subscribed"], "True")
        self.assertEqual(result["count"], 2)
        self.assertEqual(result["output"], "one\ntwo")
        self.assertEqual(execute.call_args.args[4], 2)

    def test_subscribe_none_output_keeps_default_unsubscribed_result(self):
        tool = self._make_tool("Ros2SubscribeTool")

        with (
            patch.object(self.default_tools, "execute_subprocess", return_value=None),
            patch.object(self.default_tools, "print_tool_output"),
        ):
            result = tool.run(topic="/demo_topic", max_duration=1.0, max_lines=1)

        self.assertEqual(result["subscribed"], "False")
        self.assertEqual(result["count"], "0")
        self.assertEqual(result["topic"], "")
        self.assertEqual(result["output"], "")


# endregion

if __name__ == "__main__":
    unittest.main()
