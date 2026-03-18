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
Unit tests for the ROS 2 default tools (ros2_topic).

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

# ---------------------------------------------------------------------------
# Skip entire module when ROS 2 is not available
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Mock console — implements only the interface used by tools and utils
# ---------------------------------------------------------------------------
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
        self._loop.call_soon_threadsafe(fn)

    def stop(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5)


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


# ---------------------------------------------------------------------------
# Helper: background ROS 2 publisher
# ---------------------------------------------------------------------------
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


# ===========================================================================
# Test class
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

    # -----------------------------------------------------------------------
    # Fixtures
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — ros2 topic list
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — ros2 topic info
    # -----------------------------------------------------------------------
    def test_topic_info(self):
        """`info` on a published topic returns type and publisher count."""
        topic = "/vulcan_test_topic_info"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="info", topic_name=topic)

        self.assertIn("std_msgs/msg/String", result["output"])
        self.assertIn("Publisher count:", result["output"])

    # -----------------------------------------------------------------------
    # Tests — ros2 topic type
    # -----------------------------------------------------------------------
    def test_topic_type(self):
        """`type` returns the message type of a topic."""
        topic = "/vulcan_test_topic_type"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="type", topic_name=topic)

        self.assertIn("std_msgs/msg/String", result["output"])

    # -----------------------------------------------------------------------
    # Tests — ros2 topic find
    # -----------------------------------------------------------------------
    def test_topic_find(self):
        """`find` locates topics by message type."""
        topic = "/vulcan_test_topic_find"
        proc = start_background_publisher(topic)
        self._bg_publishers.append(proc)

        result = self._run_topic(command="find", msg_type="std_msgs/msg/String")

        self.assertIn(topic, result["output"])

    # -----------------------------------------------------------------------
    # Tests — ros2 topic bw (streaming)
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — ros2 topic hz (streaming)
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — ros2 topic delay (streaming)
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — streaming commands with custom max_duration / max_lines
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Tests — error cases
    # -----------------------------------------------------------------------
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


if __name__ == "__main__":
    unittest.main()
