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

import importlib.util
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

PATH = Path(__file__).resolve().parents[2] / "src/vulcanai/console/ros2_completion.py"
spec = importlib.util.spec_from_file_location("ros2_completion", PATH)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


class CompletionTests(unittest.TestCase):
    def setUp(self):
        outputs = {
            ("--help",): "Commands:\n  bag  Record bags\n  service  Services\n",
            ("service", "list"): "/reset\n/spawn\n/turtle1/set_pen\n",
            ("service", "type", "/reset"): "std_srvs/srv/Empty\n",
            ("topic", "list"): "/turtle1/pose\n/turtle1/cmd_vel\n",
            ("topic", "type", "/turtle1/pose"): "turtlesim/msg/Pose\n",
            ("node", "list"): "/turtlesim\n/vulcanai\n",
            ("action", "list"): "/turtle1/rotate_absolute\n",
            ("param", "list", "/turtlesim"): "  background_b\n  background_g\n",
            (
                "interface",
                "list",
            ): (
                "Messages:\n  turtlesim/msg/Pose\nServices:\n  std_srvs/srv/Empty\n"
                "Actions:\n  turtlesim/action/RotateAbsolute\n"
            ),
            ("pkg", "list"): "turtlesim\nstd_srvs\n",
            ("pkg", "executables", "turtlesim"): "turtlesim turtlesim_node\nturtlesim turtle_teleop_key\n",
        }
        self.runner = Mock(side_effect=lambda args: outputs.get(tuple(args[1:]), ""))
        self.c = m.Ros2Completer(self.runner)

    def complete(self, value, cursor=None):
        return self.c.complete(value, len(value) if cursor is None else cursor)

    def test_all_default_groups_and_installed_commands(self):
        matches = self.complete("ros2 ").matches
        self.assertTrue(set(m.COMMANDS) <= set(matches))
        self.assertIn("bag", matches)

    def test_group_verbs_and_prefix(self):
        self.assertEqual(self.complete("ros2 service ca").matches, ["call"])
        for group, verbs in m.COMMANDS.items():
            self.assertTrue(set(verbs) <= set(self.complete("ros2 " + group + " ").matches))

    def test_live_service_names(self):
        self.assertEqual(self.complete("ros2 service call /").matches, ["/reset", "/spawn", "/turtle1/set_pen"])
        self.assertEqual(
            self.complete("ros2 service call /res").apply("ros2 service call /res")[0], "ros2 service call /reset "
        )

    def test_service_type_and_empty_payload(self):
        self.assertEqual(self.complete("ros2 service call /reset ").matches, ["std_srvs/srv/Empty"])
        self.assertEqual(self.complete("ros2 service call /reset std_srvs/srv/Empty ").matches, ["{}"])

    def test_entities_and_parameters(self):
        self.assertEqual(self.complete("ros2 topic hz /turtle1/p").matches, ["/turtle1/pose"])
        self.assertEqual(self.complete("ros2 node info /t").matches, ["/turtlesim"])
        self.assertEqual(self.complete("ros2 action send_goal /t").matches, ["/turtle1/rotate_absolute"])
        self.assertEqual(
            self.complete("ros2 param get /turtlesim background_").matches, ["background_b", "background_g"]
        )

    def test_interface_kind_filter(self):
        self.assertEqual(self.complete("ros2 service find ").matches, ["std_srvs/srv/Empty"])
        self.assertEqual(self.complete("ros2 topic find ").matches, ["turtlesim/msg/Pose"])

    def test_run_package_and_executable(self):
        self.assertEqual(self.complete("ros2 run tur").matches, ["turtlesim"])
        self.assertEqual(self.complete("ros2 run turtlesim turtlesim_").matches, ["turtlesim_node"])

    def test_cursor_preserves_remaining_arguments(self):
        value = "  ros2 service call /res std_srvs/srv/Empty {}"
        cursor = value.index(" std_srvs")
        completed, position = self.complete(value, cursor).apply(value)
        self.assertEqual(completed, "  ros2 service call /reset std_srvs/srv/Empty {}")
        self.assertEqual(position, len("  ros2 service call /reset"))

    def test_natural_language_is_unchanged(self):
        self.assertEqual(self.complete("draw a pentagon").matches, [])

    def test_queries_are_cached_and_timeouts_are_safe(self):
        self.complete("ros2 service call /")
        self.complete("ros2 service call /r")
        self.runner.assert_called_once_with(["ros2", "service", "list"])
        c = m.Ros2Completer(Mock(side_effect=subprocess.TimeoutExpired("ros2", 3)))
        self.assertEqual(c.complete("ros2 service call /", len("ros2 service call /")).matches, [])


if __name__ == "__main__":
    unittest.main()
