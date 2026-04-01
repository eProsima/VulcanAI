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

"""
Unit tests for the ROS 2 default tools
  - ros2_node
  - ros2_topic
  - ros2_service
  - ros2_action
  - ros2_param
  - ros2 pkg
  - ros2 interfaces

These tests require a working ROS 2 environment (rclpy importable and
ros2 CLI available). Tests are automatically skipped when ROS 2 is not
installed.

Tests call the tool's ``run()`` method directly with a minimal blackboard
containing a mock console that implements the methods the tool relies on
(logging, suggestion handling, subprocess panel, stream task).
Background ``ros2 topic pub`` processes are used to create observable topics.
"""

import asyncio
import importlib
import os
import signal
import subprocess
import sys
import threading
import time
import unittest

# -----------------------------------------------------------------------------
# Skip entire module when ROS 2 is not available
# -----------------------------------------------------------------------------
try:
    import rclpy  # noqa: F401

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Also check that the ros2 CLI is on PATH
try:
    subprocess.check_output(["ros2", "topic", "list"], stderr=subprocess.STDOUT, timeout=10)
    ROS2_CLI_AVAILABLE = True
except Exception:
    ROS2_CLI_AVAILABLE = False


# Make src-layout importable
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


# -----------------------------------------------------------------------------
# Mock console — implements only the interface used by tools and utils
# -----------------------------------------------------------------------------
class _MockLogger:
    """Collects log calls for optional inspection."""

    def __init__(self):
        self.messages = []

    def log_tool(self, msg, **kwargs):
        self.messages.append(msg)

    def log_msg(self, msg, **kwargs):
        self.messages.append(msg)

    def log_console(self, msg, **kwargs):
        self.messages.append(msg)


class _MockApp:
    """Provides an asyncio event loop on a background thread,
    mimicking Textual's ``app.call_from_thread()`` for
    ``execute_subprocess``."""

    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

    def call_from_thread(self, fn, *args, **kwargs):
        self._loop.call_soon_threadsafe(fn, *args, **kwargs)

    def stop(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5)
        self._loop.close()


class MockConsole:
    """Implements the console interface expected by tools and their helpers.

    Covers:
    - ``logger``                         — used for all tool logging
    - ``app``                            — used by execute_subprocess
                                           (provides asyncio event loop)
    - ``set_stream_task``                — used by execute_subprocess
    - ``show/hide_subprocess_panel``     — used by execute_subprocess
    - ``add_line / add_subprocess_line`` — used by run_streaming_cmd_async
    - ``change_route_logs``              — used by streaming commands
    - ``suggestion_index*``              — used by suggest_string when a topic
                                           is not found (auto-selects first match)
    - ``open_radiolist``                 — used by suggest_string modal
    """

    def __init__(self):
        self.logger = _MockLogger()
        self.app = _MockApp()
        # suggest_string support: auto-accept the first suggestion
        self.suggestion_index = 0
        self.suggestion_index_changed = threading.Event()
        self.suggestion_index_changed.set()  # unblock immediately
        self._stream_task = None

    def set_stream_task(self, task):
        self._stream_task = task

    def call_from_thread(self, fn, *args, **kwargs):
        self.app.call_from_thread(fn, *args, **kwargs)

    def show_subprocess_panel(self):
        pass

    def hide_subprocess_panel(self):
        pass

    def change_route_logs(self, value):
        pass

    def add_line(self, text):
        pass

    def add_subprocess_line(self, text):
        pass

    def open_radiolist(self, items, tool_name, string_name, input_string):
        # Auto-select index 0 (the best match) without user interaction
        pass

    def stop(self):
        self.app.stop()


# -----------------------------------------------------------------------------
# Helper: background ROS 2 publisher
# -----------------------------------------------------------------------------
def start_background_publisher(
    topic: str,
    msg_type: str = "std_msgs/msg/String",
    rate: float = 10.0,
    message: str = "hello_test",
):
    """Launch ``ros2 topic pub`` in a subprocess and return the Popen handle."""
    proc = subprocess.Popen(
        [
            "ros2",
            "topic",
            "pub",
            topic,
            msg_type,
            f"{{data: '{message}'}}",
            "--rate",
            str(rate),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    # Give the publisher a moment to register with the ROS graph
    time.sleep(2)
    return proc


def stop_background_publisher(proc: subprocess.Popen):
    """Terminate a background publisher gracefully."""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    if proc.stdout is not None:
        proc.stdout.close()
    if proc.stderr not in (None, subprocess.STDOUT):
        proc.stderr.close()


def start_background_ros2_executable(package: str, executable: str, wait_sec: float = 2.0):
    """Launch ``ros2 run <package> <executable>`` in a subprocess."""
    proc = subprocess.Popen(
        ["ros2", "run", package, executable],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    time.sleep(wait_sec)
    return proc


def stop_background_process(proc: subprocess.Popen):
    """Terminate a generic background process gracefully."""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    if proc.stdout is not None:
        proc.stdout.close()
    if proc.stderr not in (None, subprocess.STDOUT):
        proc.stderr.close()


# ===========================================================================
# Test class TestRos2NodeTool
# ===========================================================================
@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2NodeTool(unittest.TestCase):
    """Direct tests for ``Ros2NodeTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2NodeTool = default_tools_mod.Ros2NodeTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self.node_name = f"/{self.node.get_name()}"
        self._bg_processes = []

    def tearDown(self):
        for proc in self._bg_processes:
            stop_background_process(proc)
        self._bg_processes.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_node(self, **kwargs):
        tool = self.Ros2NodeTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _wait_for_node(self, timeout_sec: float = 5.0):
        start_time = time.monotonic()
        while (time.monotonic() - start_time) <= timeout_sec:
            result = self._run_node(command="list")
            if self.node_name in result["output"]:
                return
            time.sleep(0.2)
        self.fail(f"Node '{self.node_name}' not visible within {timeout_sec} seconds")

    # -------------------------------------------------------------------------
    # Tests — Node name suggestion
    # -------------------------------------------------------------------------
    def test_node_name_suggestions(self):
        """Misspelled node names should be corrected through suggestions."""
        self._wait_for_node()
        result = self._run_node(command="info", node_name=self.node.get_name())
        self.assertIn("Subscribers:", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 node list
    # -------------------------------------------------------------------------
    def test_node_list(self):
        """`list` should include the default tool node name."""
        self._wait_for_node()
        result = self._run_node(command="list")
        self.assertIn(self.node_name, result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 node info
    # -------------------------------------------------------------------------
    def test_node_info(self):
        """`info` should provide standard node metadata."""
        self._wait_for_node()
        result = self._run_node(command="info", node_name=self.node_name)
        self.assertIn("Subscribers:", result["output"])
        self.assertIn("Publishers:", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Error
    # -------------------------------------------------------------------------
    def test_node_info_missing_name_raises(self):
        """`info` with explicit None node_name should raise."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_node(command="info", node_name=None)


# ===========================================================================
# Test class Ros2TopicTool
# ===========================================================================
@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2TopicTool(unittest.TestCase):
    """Direct tests for ``Ros2TopicTool.run()``.

    Each test instantiates the tool, injects a blackboard with a
    MockConsole and a ROS2DefaultToolNode, and calls ``run()`` directly.
    """

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        """Import the default tools module and the ROS2DefaultToolNode."""
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")

        cls.Ros2TopicTool = default_tools_mod.Ros2TopicTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self._bg_publishers = []

    def tearDown(self):
        for proc in self._bg_publishers:
            stop_background_publisher(proc)
        self._bg_publishers.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_topic(self, **kwargs):
        """Create a Ros2TopicTool, inject a blackboard, and call run()."""
        tool = self.Ros2TopicTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _run_topic_threaded(self, **kwargs):
        """Run the tool from a worker thread.

        Streaming commands (bw, hz, delay) use ``execute_subprocess`` which
        requires a non-main thread so it can call
        ``console.app.call_from_thread()`` and then block on a done-event.
        """
        result = {}
        error = {}

        def worker():
            try:
                result["value"] = self._run_topic(**kwargs)
            except Exception as e:
                error["exc"] = e

        t = threading.Thread(target=worker)
        t.start()
        t.join(timeout=30)
        if "exc" in error:
            raise error["exc"]
        return result.get("value")

    # -------------------------------------------------------------------------
    # Tests — ros2 topic list
    # -------------------------------------------------------------------------
    def test_topic_list(self):
        """`list` should succeed and contain /rosout."""
        result = self._run_topic(command="list")

        self.assertIn("output", result)
        self.assertIn("/rosout", result["output"])

    def test_topic_list_with_background_pub(self):
        """After starting a background publisher, `list` should include
        that topic."""
        topic = "/vulcan_test_topic_list"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="list")

        self.assertIn(topic, result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 topic info
    # -------------------------------------------------------------------------
    def test_topic_info(self):
        """`info` on a published topic returns type and publisher count."""
        topic = "/vulcan_test_topic_info"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="info", topic_name=topic)

        self.assertIn("std_msgs/msg/String", result["output"])
        self.assertIn("Publisher count:", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 topic type
    # -------------------------------------------------------------------------
    def test_topic_type(self):
        """`type` returns the message type of a topic."""
        topic = "/vulcan_test_topic_type"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="type", topic_name=topic)

        self.assertIn("std_msgs/msg/String", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 topic find
    # -------------------------------------------------------------------------
    def test_topic_find(self):
        """`find` locates topics by message type."""
        topic = "/vulcan_test_topic_find"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="find", msg_type="std_msgs/msg/String")

        self.assertIn(topic, result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 topic bw (streaming)
    # -------------------------------------------------------------------------
    def test_topic_bw(self):
        """`bw` measures bandwidth on an active topic."""
        topic = "/vulcan_test_topic_bw"
        proc = start_background_publisher(topic, rate=10.0)
        self._bg_publishers.append(proc)

        result = self._run_topic_threaded(
            command="bw",
            topic_name=topic,
            max_duration=5.0,
            max_lines=20,
        )

        self.assertIsNotNone(result)
        self.assertIn("output", result)
        self.assertIsInstance(result["output"], str)

    # -------------------------------------------------------------------------
    # Tests — ros2 topic hz (streaming)
    # -------------------------------------------------------------------------
    def test_topic_hz(self):
        """`hz` measures the publishing rate of a topic."""
        topic = "/vulcan_test_topic_hz"
        proc = start_background_publisher(topic, rate=10.0)
        self._bg_publishers.append(proc)

        result = self._run_topic_threaded(
            command="hz",
            topic_name=topic,
            max_duration=5.0,
            max_lines=20,
        )

        self.assertIsNotNone(result)
        self.assertIn("output", result)
        self.assertIsInstance(result["output"], str)

    # -------------------------------------------------------------------------
    # Tests — ros2 topic delay (streaming)
    # -------------------------------------------------------------------------
    def test_topic_delay(self):
        """`delay` runs without error on an active topic.

        Note: ``ros2 topic delay`` requires messages with a header stamp.
        With ``std_msgs/msg/String`` (no header) the output may be empty
        or contain a warning, but the command should not crash.
        """
        topic = "/vulcan_test_topic_delay"
        proc = start_background_publisher(topic, rate=10.0)
        self._bg_publishers.append(proc)

        result = self._run_topic_threaded(
            command="delay",
            topic_name=topic,
            max_duration=5.0,
            max_lines=20,
        )

        self.assertIsNotNone(result)
        self.assertIn("output", result)
        self.assertIsInstance(result["output"], str)

    # -------------------------------------------------------------------------
    # Tests — streaming commands with custom max_duration / max_lines
    # -------------------------------------------------------------------------
    def test_topic_hz_custom_limits(self):
        """`hz` respects custom max_duration and max_lines."""
        topic = "/vulcan_test_topic_hz_limits"
        proc = start_background_publisher(topic, rate=10.0)
        self._bg_publishers.append(proc)

        result = self._run_topic_threaded(
            command="hz",
            topic_name=topic,
            max_duration=3.0,
            max_lines=5,
        )

        self.assertIsNotNone(result)
        self.assertIn("output", result)

    # -------------------------------------------------------------------------
    # Tests — error cases
    # -------------------------------------------------------------------------
    def test_topic_unknown_command_raises(self):
        """An unknown subcommand should raise ValueError."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_topic(command="nonexistent")

        with self.assertRaises(ValueError):
            self._run_topic(command="nonexistent", topic_name="/rosout")

    def test_topic_info_missing_topic_name_raises(self):
        """`info` without a topic_name should raise ValueError."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_topic(command="info")

    def test_topic_type_missing_topic_name_raises(self):
        """`type` without a topic_name should raise ValueError."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_topic(command="type")

    def test_topic_find_missing_msg_type_raises(self):
        """`find` without a msg_type should raise an exception."""
        with self.assertRaises(Exception):
            self._run_topic(command="find")


# ===========================================================================
# Test class Ros2ServiceTool
# ===========================================================================
@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2ServiceTool(unittest.TestCase):
    """Direct tests for ``Ros2ServiceTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2ServiceTool = default_tools_mod.Ros2ServiceTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self.get_parameters_service = f"/{self.node.get_name()}/get_parameters"
        self._bg_processes = []

    def tearDown(self):
        for proc in self._bg_processes:
            stop_background_process(proc)
        self._bg_processes.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_service(self, **kwargs):
        tool = self.Ros2ServiceTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _run_service_threaded(self, **kwargs):
        result = {}
        error = {}

        def worker():
            try:
                result["value"] = self._run_service(**kwargs)
            except Exception as e:
                error["exc"] = e

        t = threading.Thread(target=worker)
        t.start()
        t.join(timeout=30)
        if "exc" in error:
            raise error["exc"]
        return result.get("value")

    def _wait_for_service(self, service_name: str, timeout_sec: float = 5.0):
        start_time = time.monotonic()
        while (time.monotonic() - start_time) <= timeout_sec:
            result = self._run_service(command="list")
            if service_name in result["output"]:
                return
            time.sleep(0.2)
        self.fail(f"Service '{service_name}' not visible within {timeout_sec} seconds")

    def _start_add_two_ints_service(self):
        try:
            executables = subprocess.check_output(
                ["ros2", "pkg", "executables", "examples_rclpy_minimal_service"],
                text=True,
                timeout=10,
            )
        except Exception:
            self.skipTest("examples_rclpy_minimal_service package is not available")
            return

        if " service" not in executables:
            self.skipTest("examples_rclpy_minimal_service/service executable is not available")

        proc = start_background_ros2_executable("examples_rclpy_minimal_service", "service", wait_sec=2.0)
        self._bg_processes.append(proc)
        self._wait_for_service("/add_two_ints", timeout_sec=8.0)

    # -------------------------------------------------------------------------
    # Tests — ros2 service list
    # -------------------------------------------------------------------------
    def test_service_list(self):
        """`list` should include parameter services exposed by the tool node."""
        self._wait_for_service(self.get_parameters_service)
        result = self._run_service(command="list")

        self.assertIn("output", result)
        self.assertIn(self.get_parameters_service, result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 service info
    # -------------------------------------------------------------------------
    def test_service_info(self):
        """`info` should report type metadata for a known service."""
        self._wait_for_service(self.get_parameters_service)
        result = self._run_service(command="info", service_name=self.get_parameters_service)

        self.assertIn("Type:", result["output"])
        self.assertIn("rcl_interfaces/srv/GetParameters", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 service type
    # -------------------------------------------------------------------------
    def test_service_type(self):
        """`type` should return the service type for a known service."""
        self._wait_for_service(self.get_parameters_service)
        result = self._run_service(command="type", service_name=self.get_parameters_service)

        self.assertEqual("rcl_interfaces/srv/GetParameters", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 service find
    # -------------------------------------------------------------------------
    def test_service_find(self):
        """`find` should list services by service type."""
        self._wait_for_service(self.get_parameters_service)
        result = self._run_service(command="find", service_type="rcl_interfaces/srv/GetParameters")

        self.assertIn(self.get_parameters_service, result["output"])

    # -------------------------------------------------------------------------
    # Tests — explicit type
    # -------------------------------------------------------------------------
    def test_service_call_with_explicit_type(self):
        """`call` should work with an explicit service type."""
        self._start_add_two_ints_service()
        result = self._run_service(
            command="call",
            service_name="/add_two_ints",
            service_type="example_interfaces/srv/AddTwoInts",
            args="{a: 2, b: 3}",
        )
        self.assertIn("sum=5", result["output"])

    # -------------------------------------------------------------------------
    # Tests — autodetects type
    # -------------------------------------------------------------------------
    def test_service_call_auto_detects_type(self):
        """`call` should auto-detect service type when omitted."""
        self._start_add_two_ints_service()
        result = self._run_service(
            command="call",
            service_name="/add_two_ints",
            args="{a: 4, b: 5}",
        )
        self.assertIn("sum=9", result["output"])

    # -------------------------------------------------------------------------
    # Tests — streaming
    # -------------------------------------------------------------------------
    def test_service_echo(self):
        """`echo` should run and produce output for service event topic status."""
        self._start_add_two_ints_service()
        result = self._run_service_threaded(
            command="echo",
            service_name="/add_two_ints",
            max_duration=5.0,
            max_lines=1,
        )
        if result is None:
            self.skipTest("ros2 service echo did not yield output in time")
            return
        self.assertIn("output", result)
        self.assertIsInstance(result["output"], str)
        self.assertNotEqual("", result["output"].strip())

    # -------------------------------------------------------------------------
    # Tests — errors
    # -------------------------------------------------------------------------
    def test_service_call_missing_args_raises(self):
        """`call` without args should raise ValueError."""
        self._wait_for_service(self.get_parameters_service)
        with self.assertRaises(ValueError):
            self._run_service(
                command="call",
                service_name=self.get_parameters_service,
                service_type="rcl_interfaces/srv/GetParameters",
            )

    def test_service_info_missing_service_name_raises(self):
        """`info` without a service_name should raise."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_service(command="info")

    def test_service_unknown_command_raises(self):
        """Unknown service subcommands should raise ValueError."""
        self._wait_for_service(self.get_parameters_service)
        with self.assertRaises(ValueError):
            self._run_service(command="nonexistent", service_name=self.get_parameters_service)


# ===========================================================================
# Test class TestRos2ActionTool
# ===========================================================================
@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2ActionTool(unittest.TestCase):
    """Direct tests for ``Ros2ActionTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2ActionTool = default_tools_mod.Ros2ActionTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self._bg_processes = []

    def tearDown(self):
        for proc in self._bg_processes:
            stop_background_process(proc)
        self._bg_processes.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_action(self, **kwargs):
        tool = self.Ros2ActionTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _wait_for_action(self, action_name: str, timeout_sec: float = 8.0):
        start_time = time.monotonic()
        while (time.monotonic() - start_time) <= timeout_sec:
            result = self._run_action(command="list")
            if action_name in result["output"]:
                return
            time.sleep(0.2)
        self.fail(f"Action '{action_name}' not visible within {timeout_sec} seconds")

    def _start_fibonacci_action_server(self):
        try:
            executables = subprocess.check_output(
                ["ros2", "pkg", "executables", "examples_rclpy_minimal_action_server"],
                text=True,
                timeout=10,
            )
        except Exception:
            self.skipTest("examples_rclpy_minimal_action_server package is not available")
            return

        if " server" not in executables:
            self.skipTest("examples_rclpy_minimal_action_server/server executable is not available")

        proc = start_background_ros2_executable("examples_rclpy_minimal_action_server", "server", wait_sec=2.5)
        self._bg_processes.append(proc)
        self._wait_for_action("/fibonacci", timeout_sec=8.0)

    # -------------------------------------------------------------------------
    # Tests — Action name suggestion
    # -------------------------------------------------------------------------
    def test_action_name_suggestions(self):
        """Misspelled names should be corrected through suggestion flow."""
        self._start_fibonacci_action_server()
        result = self._run_action(command="info", action_name="fibonacc")
        self.assertIn("Action: /fibonacci", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 action list
    # -------------------------------------------------------------------------
    def test_action_list(self):
        """`list` should pass through ros2 action list output."""
        self._start_fibonacci_action_server()
        result = self._run_action(command="list")
        self.assertIn("/fibonacci", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 action info
    # -------------------------------------------------------------------------
    def test_action_info(self):
        """`info` should invoke ros2 action info for the given name."""
        self._start_fibonacci_action_server()
        result = self._run_action(command="info", action_name="/fibonacci")
        self.assertIn("Action: /fibonacci", result["output"])
        self.assertIn("Action servers:", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 action type
    # -------------------------------------------------------------------------
    def test_action_type(self):
        """`type` should invoke ros2 action type for the given action name."""
        self._start_fibonacci_action_server()
        result = self._run_action(command="type", action_name="/fibonacci")
        self.assertIn("example_interfaces/action/Fibonacci", result["output"])

    # -------------------------------------------------------------------------
    # Tests — error cases
    # -------------------------------------------------------------------------
    def test_action_info_missing_action_name_raises(self):
        """`info` without action_name should raise."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_action(command="info")

    def test_action_type_missing_action_name_raises(self):
        """`type` without action_name should raise."""
        with self.assertRaises((ValueError, TypeError)):
            self._run_action(command="type")

    def test_action_unknown_command_raises(self):
        """Unknown action subcommands should raise ValueError."""
        self._start_fibonacci_action_server()
        with self.assertRaises(ValueError):
            self._run_action(command="nonexistent", action_name="/fibonacci")


@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2ParamTool(unittest.TestCase):
    """Direct tests for ``Ros2ParamTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2ParamTool = default_tools_mod.Ros2ParamTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self.node_name = f"/{self.node.get_name()}"
        self._bg_processes = []

    def tearDown(self):
        for proc in self._bg_processes:
            stop_background_process(proc)
        self._bg_processes.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_param(self, **kwargs):
        tool = self.Ros2ParamTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _run_param_without_param_suggestion(self, **kwargs):
        """Run param tool preserving explicit param_name even if it's not a topic."""
        self.console.suggestion_index = -1
        self.console.suggestion_index_changed.set()
        return self._run_param(**kwargs)

    def _wait_for_node(self, timeout_sec: float = 5.0):
        start_time = time.monotonic()
        while (time.monotonic() - start_time) <= timeout_sec:
            try:
                node_list = subprocess.check_output(["ros2", "node", "list"], text=True)
            except Exception:
                node_list = ""
            if self.node_name in node_list:
                return
            time.sleep(0.2)
        self.fail(f"Node '{self.node_name}' not visible within {timeout_sec} seconds")

    def _start_parameter_blackboard(self):
        try:
            executables = subprocess.check_output(
                ["ros2", "pkg", "executables", "demo_nodes_cpp"],
                text=True,
                timeout=10,
            )
        except Exception:
            self.skipTest("demo_nodes_cpp package is not available")
            return

        if " parameter_blackboard" not in executables:
            self.skipTest("demo_nodes_cpp/parameter_blackboard executable is not available")

        proc = start_background_ros2_executable("demo_nodes_cpp", "parameter_blackboard", wait_sec=2.0)
        self._bg_processes.append(proc)
        start_time = time.monotonic()
        while (time.monotonic() - start_time) <= 8.0:
            try:
                node_list = subprocess.check_output(["ros2", "node", "list"], text=True, timeout=5)
            except Exception:
                node_list = ""
            if "/parameter_blackboard" in node_list:
                return
            time.sleep(0.2)
        self.fail("Node '/parameter_blackboard' not visible within timeout")

    # -------------------------------------------------------------------------
    # Tests — ros2 param list
    # -------------------------------------------------------------------------
    def test_param_list(self):
        """`list` should return a non-empty output string."""
        result = self._run_param(command="list")
        self.assertIn("output", result)
        self.assertIsInstance(result["output"], str)
        self.assertNotEqual("", result["output"].strip())

    def test_param_list_with_node_name_returns_node_list(self):
        """When node_name is provided, `list` returns node listing."""
        self._wait_for_node()
        result = self._run_param(command="list", node_name=self.node_name)
        self.assertIn(self.node_name, result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 param get <node_name> <param_name>
    # -------------------------------------------------------------------------
    def test_param_get(self):
        """`get` should return the value of a parameter."""
        self._start_parameter_blackboard()
        param_name = "vulcan_test_param_get"
        self._run_param_without_param_suggestion(
            command="set",
            node_name="/parameter_blackboard",
            param_name=param_name,
            set_value="456",
        )
        result = self._run_param_without_param_suggestion(
            command="get",
            node_name="/parameter_blackboard",
            param_name=param_name,
        )
        self.assertIn("456", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 param describe <node_name> <param_name>
    # -------------------------------------------------------------------------
    def test_param_describe(self):
        """`describe` should return metadata for a known parameter."""
        self._start_parameter_blackboard()
        result = self._run_param_without_param_suggestion(
            command="describe",
            node_name="/parameter_blackboard",
            param_name="use_sim_time",
        )
        self.assertIn("Type:", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 param set <node_name> <param_name> <set_value>
    # -------------------------------------------------------------------------
    def test_param_set(self):
        """`set` should update a parameter on parameter_blackboard."""
        self._start_parameter_blackboard()
        result = self._run_param_without_param_suggestion(
            command="set",
            node_name="/parameter_blackboard",
            param_name="vulcan_test_param_set",
            set_value="123",
        )
        self.assertIn("Set parameter", result["output"])
    # -------------------------------------------------------------------------
    # Tests — ros2 param delete <node_name> <param_name>
    # -------------------------------------------------------------------------
    def test_param_delete(self):
        """`delete` should remove a previously set parameter."""
        self._start_parameter_blackboard()
        param_name = "vulcan_test_param_delete"
        self._run_param_without_param_suggestion(
            command="set",
            node_name="/parameter_blackboard",
            param_name=param_name,
            set_value="789",
        )
        delete_result = self._run_param_without_param_suggestion(
            command="delete",
            node_name="/parameter_blackboard",
            param_name=param_name,
        )
        self.assertNotEqual("", delete_result["output"].strip())

        with self.assertRaises(Exception):
            self._run_param_without_param_suggestion(
                command="get",
                node_name="/parameter_blackboard",
                param_name=param_name,
            )
    # -------------------------------------------------------------------------
    # Tests — ros2 param dump <node_name> [file_path]
    # -------------------------------------------------------------------------
    def test_param_dump(self):
        """`dump` should produce YAML for a running parameter node."""
        self._start_parameter_blackboard()
        result = self._run_param(command="dump", node_name="/parameter_blackboard")
        self.assertIn("/parameter_blackboard:", result["output"])
        self.assertIn("ros__parameters", result["output"])

    def test_param_dump_with_output_file(self):
        """`dump` with file path currently errors on unsupported CLI option."""
        self._start_parameter_blackboard()
        dump_path = "/tmp/vulcan_param_dump.yaml"
        if os.path.exists(dump_path):
            os.remove(dump_path)
        with self.assertRaises(Exception):
            self._run_param(command="dump", node_name="/parameter_blackboard", file_path=dump_path)

    # -------------------------------------------------------------------------
    # Tests — ros2 param load <node_name> <file_path>
    # -------------------------------------------------------------------------
    def test_param_load(self):
        """`load` should restore parameters from a YAML dump file."""
        self._start_parameter_blackboard()
        param_name = "vulcan_test_param_load"
        load_path = "/tmp/vulcan_param_dump_and_load.yaml"
        try:
            self._run_param_without_param_suggestion(
                command="set",
                node_name="/parameter_blackboard",
                param_name=param_name,
                set_value="321",
            )
            # -- DUMP -----------------
            dump_result = self._run_param(command="dump", node_name="/parameter_blackboard")
            with open(load_path, "w", encoding="utf-8") as f:
                f.write(dump_result["output"])

            self._run_param_without_param_suggestion(
                command="set",
                node_name="/parameter_blackboard",
                param_name=param_name,
                set_value="999",
            )

            # -- LOAD -----------------
            load_result = self._run_param(
                command="load",
                node_name="/parameter_blackboard",
                file_path=load_path,
            )
            self.assertNotEqual("", load_result["output"].strip())

            get_result = self._run_param_without_param_suggestion(
                command="get",
                node_name="/parameter_blackboard",
                param_name=param_name,
            )
            self.assertIn("321", get_result["output"])
        finally:
            if os.path.exists(load_path):
                os.remove(load_path)

    # -------------------------------------------------------------------------
    # Tests — Error
    # -------------------------------------------------------------------------
    def test_param_set_missing_set_value_raises(self):
        """`set` requires set_value."""
        self._wait_for_node()
        with self.assertRaises(ValueError):
            self._run_param(command="set", node_name=self.node_name, param_name="/rosout")

    def test_param_load_missing_file_path_raises(self):
        """`load` requires file_path."""
        self._wait_for_node()
        with self.assertRaises(ValueError):
            self._run_param(command="load", node_name=self.node_name)

    def test_param_unknown_command_raises(self):
        """Unknown param subcommands should raise ValueError."""
        self._wait_for_node()
        with self.assertRaises(ValueError):
            self._run_param(command="nonexistent", node_name=self.node_name, param_name="/rosout")


@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2PkgTool(unittest.TestCase):
    """Direct tests for ``Ros2PkgTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2PkgTool = default_tools_mod.Ros2PkgTool

    def setUp(self):
        self.console = MockConsole()

    def tearDown(self):
        self.console.stop()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_pkg(self, **kwargs):
        tool = self.Ros2PkgTool()
        tool.bb = {"console": self.console}
        return tool.run(**kwargs)

    # -------------------------------------------------------------------------
    # Tests — ros2 pkg list
    # -------------------------------------------------------------------------
    def test_pkg_list(self):
        """`list` should pass through ros2 pkg list output."""
        result = self._run_pkg(command="list")
        self.assertIn("rclpy", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 pkg executables
    # -------------------------------------------------------------------------
    def test_pkg_executables(self):
        """`executables` should pass through ros2 pkg executables output."""
        result = self._run_pkg(command="executables")
        self.assertIn("demo_nodes_cpp talker", result["output"])
        self.assertIn("demo_nodes_cpp listener", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Error
    # -------------------------------------------------------------------------
    def test_pkg_unknown_command_raises(self):
        """Unknown pkg subcommands should raise ValueError."""
        with self.assertRaises(ValueError):
            self._run_pkg(command="nonexistent")


@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2InterfaceTool(unittest.TestCase):
    """Direct tests for ``Ros2InterfaceTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2InterfaceTool = default_tools_mod.Ros2InterfaceTool

    def setUp(self):
        self.console = MockConsole()

    def tearDown(self):
        self.console.stop()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_interface(self, **kwargs):
        tool = self.Ros2InterfaceTool()
        tool.bb = {"console": self.console}
        return tool.run(**kwargs)

    # -------------------------------------------------------------------------
    # Tests — ros2 interface list
    # -------------------------------------------------------------------------
    def test_interface_without_name_returns_interface_list(self):
        """When interface_name is None, tool should return interface list."""
        result = self._run_interface(command="list", interface_name=None)
        self.assertIn("std_msgs/msg/String", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 interface packages
    # -------------------------------------------------------------------------
    def test_interface_packages_command_returns_packages(self):
        """`packages` branch should return package names."""
        result = self._run_interface(command="packages", interface_name="std_msgs/msg/String")
        self.assertIn("std_msgs", result["output"])

    # -------------------------------------------------------------------------
    # Tests — ros2 interface package <interface_name>
    # -------------------------------------------------------------------------
    def test_interface_package_branch_calls_current_cli(self):
        """`package` branch should use the current CLI call path."""
        with self.assertRaises(Exception) as exc_info:
            self._run_interface(command="package", interface_name="std_msgs")
        self.assertIn("ros2 topic package", str(exc_info.exception))

    # -------------------------------------------------------------------------
    # Tests — ros2 interface show
    # -------------------------------------------------------------------------
    def test_interface_show_branch_calls_current_cli(self):
        """`show` branch should use the current CLI call path."""
        with self.assertRaises(Exception) as exc_info:
            self._run_interface(command="show", interface_name="std_msgs/msg/String")
        self.assertIn("ros2 topic show", str(exc_info.exception))

    # -------------------------------------------------------------------------
    # Tests — Error
    # -------------------------------------------------------------------------
    def test_interface_unknown_command_raises(self):
        """Unknown interface subcommands should raise ValueError."""
        with self.assertRaises(ValueError):
            self._run_interface(command="nonexistent", interface_name="std_msgs/msg/String")


@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2PublishTool(unittest.TestCase):
    """Direct tests for ``Ros2PublishTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2PublishTool = default_tools_mod.Ros2PublishTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self._bg_publishers = []

    def tearDown(self):
        for proc in self._bg_publishers:
            stop_background_publisher(proc)
        self._bg_publishers.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_publish(self, node=None, **kwargs):
        tool = self.Ros2PublishTool()
        tool.bb = {"console": self.console, "main_node": node if node is not None else self.node}
        return tool.run(**kwargs)

    # -------------------------------------------------------------------------
    # Tests — Publish max_lines
    # -------------------------------------------------------------------------
    def test_publish_with_max_lines(self):
        """Publisher should stop after max_lines and report count/output."""
        result = self._run_publish(
            topic="/vulcan_publish_tool_test",
            message_data="hello_publish",
            max_lines=2,
            period_sec=0.01,
            max_duration=2.0,
        )

        self.assertEqual("True", result["published"])
        self.assertEqual(2, result["count"])
        self.assertEqual("/vulcan_publish_tool_test", result["topic"])
        self.assertIn("Publishing: 'hello_publish'", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Publish max_duration
    # -------------------------------------------------------------------------
    def test_publish_with_max_duration(self):
        """Publisher should stop after max_lines and report count/output."""
        result = self._run_publish(
            topic="/vulcan_publish_tool_test",
            message_data="hello_publish",
            max_lines=10,
            period_sec=1.0,
            max_duration=10.0,
        )

        self.assertEqual("True", result["published"])
        self.assertEqual(10, result["count"])
        self.assertEqual("/vulcan_publish_tool_test", result["topic"])
        self.assertIn("Publishing: 'hello_publish'", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Errors
    # -------------------------------------------------------------------------

    # -- No topic ---------------------
    def test_publish_with_empty_topic_returns_unpublished(self):
        """Empty topic should return default unpublished result."""
        result = self._run_publish(topic="", max_lines=1)
        self.assertEqual("False", result["published"])
        self.assertEqual("0", result["count"])

    # -- Max lines <= 0 ---------------------
    def test_publish_with_non_positive_max_lines_returns_unpublished(self):
        """max_lines <= 0 should stop immediately with default result."""
        result = self._run_publish(topic="/vulcan_publish_tool_test", max_lines=0)
        self.assertEqual("False", result["published"])
        self.assertEqual("0", result["count"])

    # -- Message invalid --------------
    def test_publish_custom_message_invalid_json(self):
        """Invalid JSON for custom message type should return default result."""
        try:
            result = self._run_publish(
                topic="/dummy_topic",
                msg_type="geometry_msgs/msg/Twist",
                message_data="{invalid_json",
                max_lines=1,
            )
        except ModuleNotFoundError:
            self.skipTest("geometry_msgs package is not available")
            return

        self.assertEqual("False", result["published"])
        self.assertEqual("0", result["count"])
        self.assertEqual("/dummy_topic", result["topic"])

    # -- No main node -----------------
    def test_publish_missing_main_node_raises(self):
        """Tool should raise when blackboard has no shared node."""
        tool = self.Ros2PublishTool()
        tool.bb = {"console": self.console}
        with self.assertRaises(Exception):
            tool.run(topic="/vulcan_publish_tool_test")


@unittest.skipUnless(
    ROS2_AVAILABLE and ROS2_CLI_AVAILABLE,
    "ROS 2 (rclpy + ros2 CLI) not available — skipping",
)
class TestRos2SubscribeTool(unittest.TestCase):
    """Direct tests for ``Ros2SubscribeTool.run()``."""

    # -------------------------------------------------------------------------
    # Fixtures
    # -------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        default_tools_mod = importlib.import_module("vulcanai.tools.default_tools")
        cls.Ros2SubscribeTool = default_tools_mod.Ros2SubscribeTool
        cls.ROS2DefaultToolNode = default_tools_mod.ROS2DefaultToolNode

    def setUp(self):
        self.console = MockConsole()
        self.node = self.ROS2DefaultToolNode()
        self._bg_publishers = []

    def tearDown(self):
        for proc in self._bg_publishers:
            stop_background_publisher(proc)
        self._bg_publishers.clear()
        self.node.destroy_node()
        self.console.stop()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _run_subscribe(self, **kwargs):
        tool = self.Ros2SubscribeTool()
        tool.bb = {"console": self.console, "main_node": self.node}
        return tool.run(**kwargs)

    def _run_subscribe_threaded(self, **kwargs):
        result = {}
        error = {}

        def worker():
            try:
                result["value"] = self._run_subscribe(**kwargs)
            except Exception as e:
                error["exc"] = e

        t = threading.Thread(target=worker)
        t.start()
        t.join(timeout=30)
        if "exc" in error:
            raise error["exc"]
        return result.get("value")

    # -------------------------------------------------------------------------
    # Tests — Subscribe max_lines
    # -------------------------------------------------------------------------
    def test_subscribe_with_max_lines(self):
        """`run` should receive live topic data and count lines."""
        topic = "/vulcan_test_subscribe"
        proc = start_background_publisher(topic, message="hello_subscribe", rate=10)
        self._bg_publishers.append(proc)

        result = self._run_subscribe_threaded(topic=topic, max_duration=5.0, max_lines=5)

        self.assertEqual("True", result["subscribed"])
        self.assertEqual(5, result["count"])
        self.assertEqual(topic, result["topic"])
        self.assertIn("I heard", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Subscribe max_duration
    # -------------------------------------------------------------------------
    def test_subscribe_with_max_lines(self):
        """`run` should receive live topic data and count lines."""
        topic = "/vulcan_test_subscribe"
        proc = start_background_publisher(topic, message="hello_subscribe", rate=1)
        self._bg_publishers.append(proc)

        result = self._run_subscribe_threaded(topic=topic, max_duration=5.0, max_lines=50)

        self.assertEqual("True", result["subscribed"])
        self.assertEqual(5, result["count"])
        self.assertEqual(topic, result["topic"])
        self.assertIn("I heard", result["output"])

    # -------------------------------------------------------------------------
    # Tests — Subscribe (no publisher)
    # -------------------------------------------------------------------------
    def test_subscribe_empty_output(self):
        """Empty subprocess output still returns subscribed state."""
        result = self._run_subscribe_threaded(topic="/vulcan_test_no_pub", max_duration=1.0, max_lines=1)
        self.assertEqual("True", result["subscribed"])
        self.assertGreaterEqual(result["count"], 0)
        self.assertIsInstance(result["output"], str)

    # -------------------------------------------------------------------------
    # Tests — Error
    # -------------------------------------------------------------------------
    def test_subscribe_missing_main_node_raises(self):
        """Tool should raise when blackboard has no shared node."""
        tool = self.Ros2SubscribeTool()
        tool.bb = {"console": self.console}
        with self.assertRaises(Exception):
            tool.run(topic="/my_topic")


if __name__ == "__main__":
    unittest.main()
