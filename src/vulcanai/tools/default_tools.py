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
This file contains the default tools given by VulcanAI.
It contains atomic tools used to call ROS2 CLI.
"""

import importlib
import json
import threading
import time
from concurrent.futures import Future

from vulcanai import AtomicTool, vulcanai_tool
from vulcanai.tools.utils import (
    execute_subprocess,
    log_tool_in_stream_and_main,
    print_tool_output,
    run_oneshot_cmd,
    suggest_string,
)

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.task import Future
except ImportError:
    raise ImportError("Unable to load default tools because no ROS 2 installation was found.")


# This class contains a ROS 2 node that will be loaded if none is provided to launch ROS 2 default tools
class ROS2DefaultToolNode(Node):
    def __init__(self, name: str = "vulcanai_ros2_default_tools_node"):
        if not rclpy.ok():
            rclpy.init()
        # This helper node is not spun continuously, so exposing parameter services
        # would make `ros2 param list` hang while waiting on unanswered requests.
        super().__init__(name, start_parameter_services=False)
        # Dictionary to store created clients
        self._vulcan_clients = {}
        # Dictionary to store created publishers
        self._vulcan_publishers = {}

        # Ensure entities creation is thread-safe.
        self.node_lock = threading.Lock()

    def get_client(self, srv_type, srv_name):
        """
        Get a cached client for the specified service type and name or
        create a new one if it doesn't exist.
        """
        key = (srv_type, srv_name)
        with self.node_lock:
            if key not in self._vulcan_clients:
                client = self.create_client(srv_type, srv_name)
                self._vulcan_clients[key] = client
                self.get_logger().info(f"Created new client for {srv_name}")
            return self._vulcan_clients[key]

    def get_publisher(self, msg_type, topic_name):
        """
        Get a cached publisher for the specified message type and topic name or
        create a new one if it doesn't exist.
        """
        key = (msg_type, topic_name)
        with self.node_lock:
            if key not in self._vulcan_publishers:
                publisher = self.create_publisher(msg_type, topic_name, 10)
                self._vulcan_publishers[key] = publisher
                self.get_logger().info(f"Created new publisher for {topic_name}")
            return self._vulcan_publishers[key]

    def wait_for_message(self, msg_type, topic: str, timeout_sec: float = None):
        """
        Block until a message is received or timeout expires.
        Subscriptions are created on demand and destroyed after use to avoid
        handling spins and callbacks in a separate thread.
        """
        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(msg_type, topic, callback, 10)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        return None


"""
Available ROS 2 CLI commands that can be run with the tools in this file:

- ros2 node
    Commands:
        info  Output information about a node
        list  Output a list of available nodes

- ros2 topic
    Commands:
        bw     Display bandwidth used by topic
        delay  Display delay of topic from timestamp in header
        echo   Output messages from a topic
        find   Output a list of available topics of a given type
        hz     Print the average receiving rate to screen
        info   Print information about a topic
        list   Output a list of available topics
        pub    Publish a message to a topic
        type   Print a topic's type

- ros2 service
    Commands:
        call  Call a service
        echo  Echo a service
        find  Output a list of available services of a given type
        info  Print information about a service
        list  Output a list of available services
        type  Output a service's type

- ros2 action
    Commands:
        info       Print information about an action
        list       Output a list of action names
        send_goal  Send an action goal
        type       Print a action's type
        echo       Echo a action
        find       Find actions from type


- ros2 param
    Commands:
        delete    Delete parameter
        describe  Show descriptive information about declared parameters
        dump      Show all of the parameters of a node in a YAML file format
        get       Get parameter
        list      Output a list of available parameters
        load      Load parameter file for a node
        set       Set parameter


- ros2 pkg
    Commands:
        executables  Output a list of package specific executables
        list         Output a list of available packages
        prefix       Output the prefix path of a package
        xml          Output the XML of the package manifest or a specific ta


- ros2 interfaces
    Commands:
        list      List all interface types available
        package   Output a list of available interface types within one package
        packages  Output a list of packages that provide interfaces
"""


def _require_console(bb):
    console = bb.get("console", None)
    if console is None:
        raise Exception("Could not find console, aborting...")
    return console


def _normalize_command(command):
    if command is None:
        raise ValueError("`command` is required.")
    return str(command).lower()


def _run_ros2_node_command(console, tool_name: str, command: str, node_name: str = None):
    command = _normalize_command(command)
    result = {"output": ""}

    node_name_list_str = run_oneshot_cmd(["ros2", "node", "list"])
    node_name_list = node_name_list_str.splitlines()

    if command == "list":
        result["output"] = node_name_list_str
    elif command == "info":
        suggested_topic = suggest_string(console, tool_name, "Node", node_name, node_name_list)
        if suggested_topic is not None:
            node_name = suggested_topic

        if not node_name:
            raise ValueError("`command='{}'` requires `node_name`.".format("info"))

        result["output"] = run_oneshot_cmd(["ros2", "node", "info", node_name])
    else:
        raise ValueError(f"Unknown command '{command}'. Expected one of: list, info.")

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_topic_command(
    console,
    tool_name: str,
    command: str,
    topic_name: str = None,
    msg_type: str = None,
    max_duration: float = None,
    max_lines: int = None,
):
    command = _normalize_command(command)
    result = {"output": ""}

    topic_name_list_str = run_oneshot_cmd(["ros2", "topic", "list"])
    topic_name_list = topic_name_list_str.splitlines()

    if command == "find":
        if not msg_type:
            raise ValueError("`command='find'` requires `msg_type`.")
    elif command != "list":
        suggested_topic_name = suggest_string(console, tool_name, "Topic", topic_name, topic_name_list)
        if suggested_topic_name is not None:
            topic_name = suggested_topic_name

        if not topic_name:
            raise ValueError("`command='{}'` requires `topic_name`.".format(command))

    if command == "list":
        result["output"] = topic_name_list_str
    elif command == "info":
        result["output"] = run_oneshot_cmd(["ros2", "topic", "info", topic_name])
    elif command == "find":
        find_output = run_oneshot_cmd(["ros2", "topic", "find", msg_type])
        find_topics = [line.strip() for line in find_output.splitlines() if line.strip()]
        result["output"] = ", ".join(find_topics)
    elif command == "type":
        result["output"] = run_oneshot_cmd(["ros2", "topic", "type", topic_name])
    elif command == "bw":
        result["output"] = execute_subprocess(
            console,
            tool_name,
            ["ros2", "topic", "bw", topic_name],
            max_duration,
            max_lines,
        )
    elif command == "delay":
        result["output"] = execute_subprocess(
            console,
            tool_name,
            ["ros2", "topic", "delay", topic_name],
            max_duration,
            max_lines,
        )
    elif command == "hz":
        result["output"] = execute_subprocess(
            console,
            tool_name,
            ["ros2", "topic", "hz", topic_name],
            max_duration,
            max_lines,
        )
    else:
        raise ValueError(
            f"Unknown command '{command}'. Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
        )

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_service_command(
    console,
    tool_name: str,
    command: str,
    service_name: str = None,
    service_type: str = None,
    call_args: str = None,
    max_duration: float = None,
    max_lines: int = None,
):
    command = _normalize_command(command)
    result = {"output": ""}

    service_name_list_str = run_oneshot_cmd(["ros2", "service", "list"])
    service_name_list = service_name_list_str.splitlines()

    if command == "find":
        if not service_type:
            raise ValueError("`command='find'` requires `service_type`.")
    elif command != "list":
        suggested_service_name = suggest_string(console, tool_name, "Service", service_name, service_name_list)
        if suggested_service_name is not None:
            service_name = suggested_service_name

        if not service_name:
            raise ValueError("`command='{}'` requires `service_name`.".format(command))

    if command == "list":
        result["output"] = service_name_list_str
    elif command == "info":
        result["output"] = run_oneshot_cmd(["ros2", "service", "info", service_name])
    elif command == "type":
        result["output"] = run_oneshot_cmd(["ros2", "service", "type", service_name]).strip()
    elif command == "find":
        result["output"] = run_oneshot_cmd(["ros2", "service", "find", service_type])
    elif command == "call":
        if call_args is None:
            raise ValueError("`command='call'` requires `args`.")
        if not service_type:
            service_type = run_oneshot_cmd(["ros2", "service", "type", service_name]).strip()
        result["output"] = run_oneshot_cmd(["ros2", "service", "call", service_name, service_type, call_args])
    elif command == "echo":
        result["output"] = execute_subprocess(
            console,
            tool_name,
            ["ros2", "service", "echo", service_name],
            max_duration,
            max_lines,
        )
    else:
        raise ValueError(f"Unknown command '{command}'. Expected one of: list, info, type, call, echo, find.")

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_action_command(
    console,
    tool_name: str,
    command: str,
    action_name: str = None,
    action_type: str = None,
    goal_args: str = None,
):
    command = _normalize_command(command)
    result = {"output": ""}

    action_name_list_str = run_oneshot_cmd(["ros2", "action", "list"])
    action_name_list = action_name_list_str.splitlines()

    if command != "list":
        suggested_action_name = suggest_string(console, tool_name, "Action", action_name, action_name_list)
        if suggested_action_name is not None:
            action_name = suggested_action_name

        if not action_name:
            raise ValueError("`command='{}'` requires `action_name`.".format(command))

    if command == "list":
        result["output"] = action_name_list_str
    elif command == "info":
        result["output"] = run_oneshot_cmd(["ros2", "action", "info", action_name])
    elif command == "type":
        result["output"] = run_oneshot_cmd(["ros2", "action", "type", action_name])
    elif command == "send_goal":
        if not action_type:
            action_type = run_oneshot_cmd(["ros2", "action", "type", action_name]).strip()
        args_list = ["ros2", "action", "send_goal", action_name, action_type]
        if goal_args is not None:
            args_list.append(goal_args)
        result["output"] = run_oneshot_cmd(args_list)
    else:
        raise ValueError(f"Unknown command '{command}'. Expected one of: list, info, type, send_goal.")

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_param_command(
    console,
    tool_name: str,
    command: str,
    node_name: str = None,
    param_name: str = None,
    set_value: str = None,
    file_path: str = None,
):
    command = _normalize_command(command)
    result = {"output": ""}

    param_name_list_str = run_oneshot_cmd(["ros2", "param", "list"])
    node_name_list_str = run_oneshot_cmd(["ros2", "node", "list"])

    param_name_list = param_name_list_str.splitlines()
    node_name_list = node_name_list_str.splitlines()

    if command != "list":
        if command not in ["dump", "load"]:
            suggested_param_name = suggest_string(console, tool_name, "Param", param_name, param_name_list)
            if suggested_param_name is not None:
                param_name = suggested_param_name
            if not param_name:
                raise ValueError("`command='{}'` requires `param_name`.".format(command))

        suggested_node_name = suggest_string(console, tool_name, "Node", node_name, node_name_list)
        if suggested_node_name is not None:
            node_name = suggested_node_name
        if not node_name:
            raise ValueError("`command='{}'` requires `node_name`.".format(command))

    if command == "list":
        if node_name:
            result["output"] = node_name_list_str
        else:
            result["output"] = param_name_list_str
    elif command == "get":
        get_output = run_oneshot_cmd(["ros2", "param", "get", node_name, param_name])
        if "parameter not set" in get_output.lower():
            raise Exception(f"Parameter '{param_name}' is not set on node '{node_name}'.")
        result["output"] = get_output
    elif command == "describe":
        result["output"] = run_oneshot_cmd(["ros2", "param", "describe", node_name, param_name])
    elif command == "set":
        if set_value is None:
            raise ValueError("`command='set'` requires `set_value`.")
        result["output"] = run_oneshot_cmd(["ros2", "param", "set", node_name, param_name, set_value])
    elif command == "delete":
        result["output"] = run_oneshot_cmd(["ros2", "param", "delete", node_name, param_name])
    elif command == "dump":
        if file_path:
            dump_output = run_oneshot_cmd(["ros2", "param", "dump", node_name, "--output-file", file_path])
            result["output"] = dump_output or f"Dumped parameters to {file_path}"
        else:
            result["output"] = run_oneshot_cmd(["ros2", "param", "dump", node_name])
    elif command == "load":
        if not file_path:
            raise ValueError("`command='load'` `file_path`.")
        result["output"] = run_oneshot_cmd(["ros2", "param", "load", node_name, file_path])
    else:
        raise ValueError(f"Unknown command '{command}'. Expected one of: list, get, describe, set, delete, dump, load.")

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_pkg_command(console, tool_name: str, command: str):
    command = _normalize_command(command)
    result = {"output": ""}

    if command == "list":
        result["output"] = run_oneshot_cmd(["ros2", "pkg", "list"])
    elif command == "executables":
        result["output"] = run_oneshot_cmd(["ros2", "pkg", "executables"])
    else:
        raise ValueError(f"Unknown command '{command}'. Expected one of: list, executables, prefix, xml")

    print_tool_output(console, result["output"], tool_name)
    return result


def _run_ros2_interface_command(console, tool_name: str, command: str, interface_name: str = None):
    command = _normalize_command(command)
    result = {"output": ""}

    interface_name_list_str = run_oneshot_cmd(["ros2", "interface", "list"])
    interface_name_list = interface_name_list_str.splitlines()

    package_name_list_str = run_oneshot_cmd(["ros2", "interface", "packages"])
    package_name_list = package_name_list_str.splitlines()

    if command == "list":
        result["output"] = interface_name_list_str
    elif command == "packages":
        result["output"] = package_name_list_str
    elif command == "package":
        package_name = interface_name
        suggested_package_name = suggest_string(console, tool_name, "Interface", package_name, package_name_list)
        if suggested_package_name is not None:
            package_name = suggested_package_name
        if not interface_name:
            raise ValueError("`command='{}'` requires `interface_name`.".format(command))
        # Keep existing command behavior for compatibility.
        result["output"] = run_oneshot_cmd(["ros2", "topic", "package", package_name])
    elif command == "show":
        suggested_interface_name = suggest_string(console, tool_name, "Interface", interface_name, interface_name_list)
        if suggested_interface_name is not None:
            interface_name = suggested_interface_name
        if not interface_name:
            raise ValueError("`command='{}'` requires `interface_name`.".format(command))
        # Keep existing command behavior for compatibility.
        result["output"] = run_oneshot_cmd(["ros2", "topic", "show", interface_name])
    else:
        raise ValueError(
            f"Unknown command '{command}'. Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
        )

    print_tool_output(console, result["output"], tool_name)
    return result


# ---------------------------------------------------------------------------
# Strict per-subcommand ROS 2 tools (planner-facing)
# ---------------------------------------------------------------------------


@vulcanai_tool
class Ros2NodeListTool(AtomicTool):
    name = "ros2_node_list"
    description = (
        "List all currently available ROS 2 nodes. "
        "Equivalent to `ros2 node list`. "
        "Use when the user wants to list, show, display, or print ROS 2 nodes."
    )
    tags = [
        "ros2",
        "ros 2",
        "nodes",
        "list",
        "node list",
        "ros2 node list",
        "list ros2 nodes",
        "show ros2 nodes",
        "print ros2 nodes",
        "available nodes",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_node_command(console, self.name, "list")


@vulcanai_tool
class Ros2NodeInfoTool(AtomicTool):
    name = "ros2_node_info"
    description = (
        "Show details for a specific ROS 2 node. "
        "Equivalent to `ros2 node info <name>`."
        "Use when the user wants to list, show, display, or print the information of a ROS 2 node."
        )
    tags = [
        "ros2",
        "ros 2",
        "info",
        "node info",
        "ros2 node info",
        "inspect node",
        "show node details"
    ]
    input_schema = [("node_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_node_command(console, self.name, "info", node_name=kwargs.get("node_name"))


@vulcanai_tool
class Ros2TopicListTool(AtomicTool):
    name = "ros2_topic_list"
    description = (
        "List all currently available ROS 2 topics. "
        "Equivalent to `ros2 topic list`. "
        "Use when the user wants to list, show, display, or print ROS 2 topics."
    )
    tags = [
        "ros2",
        "ros 2",
        "topics",
        "list",
        "topic list",
        "ros2 topic list",
        "list ros2 topics",
        "show ros2 topics",
        "print ros2 topics",
        "available topics",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(console, self.name, "list")


@vulcanai_tool
class Ros2TopicInfoTool(AtomicTool):
    name = "ros2_topic_info"
    description = (
        "Show details for a specific ROS 2 topic. "
        "Equivalent to `ros2 topic info`. "
        "Use when the user wants to list, show, display, or print the information of a ROS 2 topic."
    )
    tags = [
        "ros2",
        "ros 2",
        "info",
        "topic info",
        "ros2 topic info",
        "inspect topic",
        "show topic details"
    ]
    input_schema = [("topic_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(console, self.name, "info", topic_name=kwargs.get("topic_name"))


@vulcanai_tool
class Ros2TopicFindTool(AtomicTool):
    name = "ros2_topic_find"
    description = (
        "Find ROS 2 topics by message type. "
        "Equivalent to `ros2 topic find <msg_type>`."
        "Use when the user wants to find, show, display, or print the topic with a given ROS 2 topic type."
    )
    tags = [
        "ros2",
        "ros 2",
        "find",
        "topic find",
        "ros2 topic find",
        "find topics by type",
        "topics by message type",
        "find ros2 topics",
        "message type",
    ]
    input_schema = [("msg_type", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(console, self.name, "find", msg_type=kwargs.get("msg_type"))


@vulcanai_tool
class Ros2TopicTypeTool(AtomicTool):
    name = "ros2_topic_type"
    description = (
        "Show the message type used by a ROS 2 topic. "
        "Equivalent to `ros2 topic type <topic_name>`."
        "Use when the user wants to get, show, display, or print the type of a ROS 2 topic."
    )
    tags = [
        "ros2",
        "ros 2",
        "type",
        "topic type",
        "ros2 topic type",
        "topic message type",
        "show topic type",
        "get topic type",
        "topic datatype",
    ]
    input_schema = [("topic_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(console, self.name, "type", topic_name=kwargs.get("topic_name"))


@vulcanai_tool
class Ros2TopicBwTool(AtomicTool):
    name = "ros2_topic_bw"
    description = (
        "Stream and observe ROS 2 topic bandwidth. "
        "Equivalent to `ros2 topic bw`. "
        "Use when the user wants to show, display, or print the bandwidth of a ROS 2 topic that is publishing. "
        "If optional limits are omitted, it defaults to 60 seconds and 100 lines."
    )
    tags = [
        "ros2",
        "ros 2",
        "bw",
        "bandwidth",
        "topic bandwidth",
        "ros2 topic bandwidth",
        "inspect topic bandwidth",
        "show topic bandwidth"
    ]

    input_schema = [
        ("topic_name", "string"),
        ("max_duration", "float?"),
        ("max_lines", "int?")
    ]
    input_defaults = {"max_duration": 60, "max_lines": 100}
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)

        # Streaming commands variables
        max_duration = kwargs.get("max_duration")
        if max_duration is None:
            max_duration = 60

        max_lines = kwargs.get("max_lines")
        if max_lines is None:
            max_lines = 100

        return _run_ros2_topic_command(
            console,
            self.name,
            "bw",
            topic_name=kwargs.get("topic_name"),
            max_duration=max_duration,
            max_lines=max_lines,
        )


@vulcanai_tool
class Ros2TopicDelayTool(AtomicTool):
    name = "ros2_topic_delay"
    description = (
        "Stream and observe ROS 2 topic delay. "
        "Equivalent to `ros2 topic delay`. "
        "Use when the user wants to show, display, or print the delay of a ROS 2 topic that is publishing."
    )
    tags = [
        "ros2",
        "ros 2",
        "delay",
        "topic delay",
        "ros2 topic delay",
        "inspect topic delay",
        "show topic delay"
    ]
    input_schema = [("topic_name", "string"), ("max_duration", "float"), ("max_lines", "int")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(
            console,
            self.name,
            "delay",
            topic_name=kwargs.get("topic_name"),
            max_duration=kwargs.get("max_duration"),
            max_lines=kwargs.get("max_lines"),
        )


@vulcanai_tool
class Ros2TopicHzTool(AtomicTool):
    name = "ros2_topic_hz"
    description = (
        "Stream and observe ROS 2 topic average receiving rate. "
        "Equivalent to `ros2 topic hz`. "
        "Use when the user wants to show, display, or print the average receiving rate of a ROS 2 topic that is publishing."
    )
    tags = [
        "ros2",
        "ros 2",
        "hz"
        "rate"
        "receiving rate"
        "average receiving rate"
        "topic hz",
        "ros2 topic hz",
        "inspect topic hz",
        "show topic hz"
    ]
    input_schema = [("topic_name", "string"), ("max_duration", "float"), ("max_lines", "int")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_topic_command(
            console,
            self.name,
            "hz",
            topic_name=kwargs.get("topic_name"),
            max_duration=kwargs.get("max_duration"),
            max_lines=kwargs.get("max_lines"),
        )


@vulcanai_tool
class Ros2ServiceListTool(AtomicTool):
    name = "ros2_service_list"
    description = (
        "List all currently available ROS 2 services. "
        "Equivalent to `ros2 service list`. "
        "Use when the user wants to list, show, display, or print ROS 2 services."
    )
    tags = [
        "ros2",
        "ros 2",
        "services",
        "list",
        "service list",
        "ros2 service list",
        "list ros2 services",
        "show ros2 services",
        "print ros2 services",
        "available services",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(console, self.name, "list")


@vulcanai_tool
class Ros2ServiceInfoTool(AtomicTool):
    name = "ros2_service_info"
    description = (
        "Show details for a specific ROS 2 service. "
        "Equivalent to `ros2 service info`. "
        "Use when the user wants to list, show, display, or print the information of a ROS 2 service."
    )
    tags = [
        "ros2",
        "ros 2",
        "info",
        "information",
        "service info",
        "ros2 service info",
        "inspect service",
        "show service details"
    ]
    input_schema = [("service_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(console, self.name, "info", service_name=kwargs.get("service_name"))


@vulcanai_tool
class Ros2ServiceTypeTool(AtomicTool):
    name = "ros2_service_type"
    description = (
        "Show the message type used by a ROS 2 service. "
        "Equivalent to `ros2 service type <service_name>`."
        "Use when the user wants to get, show, display, or print the type of a ROS 2 service."
    )
    tags = [
        "ros2",
        "ros 2",
        "type",
        "service type",
        "ros2 service type",
        "service message type",
        "show service type",
        "get service type",
        "service datatype",
    ]
    input_schema = [("service_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(console, self.name, "type", service_name=kwargs.get("service_name"))


@vulcanai_tool
class Ros2ServiceFindTool(AtomicTool):
    name = "ros2_service_find"
    description = (
        "Find ROS 2 services by message type. "
        "Equivalent to `ros2 service find <msg_type>`."
        "Use when the user wants to find, show, display, or print the service with a given ROS 2 sevice type."
    )
    tags = [
        "ros2",
        "ros 2",
        "service find",
        "ros2 service find",
        "find services by type",
        "services by message type",
        "find ros2 services",
        "message type",
    ]
    input_schema = [("service_type", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(console, self.name, "find", service_type=kwargs.get("service_type"))


@vulcanai_tool
class Ros2ServiceCallTool(AtomicTool):
    name = "ros2_service_call"
    description = (
        "Call a ROS 2 service. "
        "Equivalent to `ros2 service call <args>`."
        "Use when the user wants to call a ROS 2 service ."
    )
    tags = [
        "ros2",
        "ros 2",
        "call",
        "ros2 service call",
        "call service",
        "call ros2 service",
    ]
    input_schema = [("service_name", "string"), ("service_type", "string"), ("args", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(
            console,
            self.name,
            "call",
            service_name=kwargs.get("service_name"),
            service_type=kwargs.get("service_type"),
            call_args=kwargs.get("args"),
        )


@vulcanai_tool
class Ros2ServiceEchoTool(AtomicTool):
    name = "ros2_service_echo"
    description = (
        "Stream and observe ROS 2 service traffic over time. "
        "Equivalent to `ros2 service echo <service_name>`."
        "Use when the user wants to show, display, or print the information a ROS 2 service is publishing."
    )
    tags = [
        "ros2",
        "ros 2",
        "service echo",
        "ros2 service echo",
        "show service traffic",
        "stream service traffic",
        "observe service",
        "echo service",
    ]
    input_schema = [("service_name", "string"), ("max_duration", "float"), ("max_lines", "int")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_service_command(
            console,
            self.name,
            "echo",
            service_name=kwargs.get("service_name"),
            max_duration=kwargs.get("max_duration"),
            max_lines=kwargs.get("max_lines"),
        )


@vulcanai_tool
class Ros2ActionListTool(AtomicTool):
    name = "ros2_action_list"
    description = (
        "List all currently available ROS 2 actions. "
        "Equivalent to `ros2 action list`. "
        "Use when the user wants to list, show, display, or print ROS 2 actions."
    )
    tags = [
        "ros2",
        "ros 2",
        "actions",
        "action list",
        "ros2 action list",
        "list ros2 actions",
        "show ros2 actions",
        "print ros2 actions",
        "available actions",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_action_command(console, self.name, "list")


@vulcanai_tool
class Ros2ActionInfoTool(AtomicTool):
    name = "ros2_action_info"
    description = (
        "Show details for a specific ROS 2 action. "
        "Equivalent to `ros2 action info`. "
        "Use when the user wants to list, show, display, or print the information of a ROS 2 action."
    )
    tags = [
        "ros2",
        "ros 2",
        "action info",
        "ros2 action info",
        "inspect action",
        "show action details"
    ]
    input_schema = [("action_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_action_command(console, self.name, "info", action_name=kwargs.get("action_name"))


@vulcanai_tool
class Ros2ActionTypeTool(AtomicTool):
    name = "ros2_action_type"
    description = (
        "Show the message type used by a ROS 2 action. "
        "Equivalent to `ros2 action type <action_name>`."
        "Use when the user wants to get, show, display, or print the type of a ROS 2 action."
    )
    tags = [
        "ros2",
        "ros 2",
        "action type",
        "ros2 action type",
        "action message type",
        "show action type",
        "get action type",
        "action datatype",
    ]
    input_schema = [("action_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_action_command(console, self.name, "type", action_name=kwargs.get("action_name"))


@vulcanai_tool
class Ros2ActionSendGoalTool(AtomicTool):
    name = "ros2_action_send_goal"
    description = (
        "Send a goal to a ROS 2 action server. "
        "Equivalent to `ros2 action send_goal <action_name> <action_type>`."
        "Use when the user wants to send a ROS 2 action goal"
    )
    tags = [
        "ros2",
        "ros 2",
        "goal",
        "send goal",
        "action send goal",
        "ros2 action send_goal",
        "send action goal",
        "call action",
        "trigger action"
    ]
    input_schema = [("action_name", "string"), ("action_type", "string"), ("goal_args", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_action_command(
            console,
            self.name,
            "send_goal",
            action_name=kwargs.get("action_name"),
            action_type=kwargs.get("action_type"),
            goal_args=kwargs.get("goal_args"),
        )


@vulcanai_tool
class Ros2ParamListTool(AtomicTool):
    name = "ros2_param_list"
    description = (
        "List all currently available ROS 2 params. "
        "Equivalent to `ros2 param list`. "
        "Use when the user wants to list, show, display, or print ROS 2 params."
    )
    tags = [
        "ros2",
        "ros 2",
        "params",
        "param list",
        "ros2 param list",
        "list ros2 params",
        "show ros2 params",
        "print ros2 params",
        "available params",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(console, self.name, "list")


@vulcanai_tool
class Ros2ParamGetTool(AtomicTool):
    name = "ros2_param_get"
    description = (
        "Get the value of a parameter from a ROS 2 node. "
        "Equivalent to `ros2 param get <node_name> <param_name>`."
        "Use when the user want to get a Ros 2 parameter"
    )
    tags = [
        "ros2",
        "ros 2",
        "get",
        "param get",
        "ros2 param get",
        "get parameter",
        "read parameter",
        "show parameter value"
    ]
    input_schema = [("node_name", "string"), ("param_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(
            console,
            self.name,
            "get",
            node_name=kwargs.get("node_name"),
            param_name=kwargs.get("param_name"),
        )


@vulcanai_tool
class Ros2ParamDescribeTool(AtomicTool):
    name = "ros2_param_describe"
    description = (
        "Show the definition and metadata of a ROS 2 parameter. "
        "Equivalent to `ros2 param describe <node_name> <param_name>`."
        "Use when the user want to show, display or print a desciptive information about a ROS 2 parameter"
    )
    tags = [
        "ros2",
        "ros 2",
        "describe",
        "param describe",
        "ros2 param describe",
        "describe parameter",
        "parameter metadata",
        "parameter details",
    ]
    input_schema = [("node_name", "string"), ("param_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(
            console,
            self.name,
            "describe",
            node_name=kwargs.get("node_name"),
            param_name=kwargs.get("param_name"),
        )


@vulcanai_tool
class Ros2ParamSetTool(AtomicTool):
    name = "ros2_param_set"
    description = (
        "Set the value of a parameter on a ROS 2 node. "
        "Equivalent to `ros2 param set <node_name> <param_name> <value>`."
        "Use when the user want to set a ROS 2 parameter"
    )
    tags = [
        "ros2",
        "ros 2",
        "set",
        "param set",
        "ros2 param set",
        "set parameter",
        "update parameter",
        "change parameter value"
    ]
    input_schema = [("node_name", "string"), ("param_name", "string"), ("set_value", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(
            console,
            self.name,
            "set",
            node_name=kwargs.get("node_name"),
            param_name=kwargs.get("param_name"),
            set_value=kwargs.get("set_value"),
        )


@vulcanai_tool
class Ros2ParamDeleteTool(AtomicTool):
    name = "ros2_param_delete"
    description = (
        "Delete a parameter from a ROS 2 node. "
        "Equivalent to `ros2 param delete <node_name> <param_name>`."
        "Used when the user want to delete a ROS 2 parameter"
    )
    tags = [
        "ros2",
        "ros 2",
        "delete",
        "remove",
        "param delete",
        "ros2 param delete",
        "delete parameter",
        "remove parameter",
        "unset parameter"
    ]
    input_schema = [("node_name", "string"), ("param_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(
            console,
            self.name,
            "delete",
            node_name=kwargs.get("node_name"),
            param_name=kwargs.get("param_name"),
        )


@vulcanai_tool
class Ros2ParamDumpTool(AtomicTool):
    name = "ros2_param_dump"
    description = (
        "Export all parameters from a ROS 2 node. "
        "Equivalent to `ros2 param dump <node_name>`."
        "Used when the user want to show, display or print all the parameters of a ROS 2 node"
    )
    tags = [
        "ros2",
        "ros 2",
        "dump",
        "yaml",
        "param dump",
        "ros2 param dump",
        "export parameters",
        "dump parameters",
        "save node parameters"
    ]
    input_schema = [("node_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(console, self.name, "dump", node_name=kwargs.get("node_name"))


@vulcanai_tool
class Ros2ParamLoadTool(AtomicTool):
    name = "ros2_param_load"
    description = (
        "Load parameters into a ROS 2 node from a file."
        "Equivalent to `ros2 param load <node_name> <file_path>`."
        "Used when the user want to load a parameter filer for a ROS 2 node"
    )
    tags = [
        "ros2",
        "ros 2",
        "load",
        "file",
        "param load",
        "ros2 param load",
        "load parameters",
        "import parameters",
        "load params from file"
    ]
    input_schema = [("node_name", "string"), ("file_path", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_param_command(
            console,
            self.name,
            "load",
            node_name=kwargs.get("node_name"),
            file_path=kwargs.get("file_path"),
        )


@vulcanai_tool
class Ros2PkgListTool(AtomicTool):
    name = "ros2_pkg_list"
    description = (
        "List all currently available ROS 2 pkgs. "
        "Equivalent to `ros2 pkg list`. "
        "Use when the user wants to list, show, display, or print ROS 2 packages."
    )
    tags = [
        "ros2",
        "ros 2",
        "pkg",
        "pkg list",
        "ros2 pkg list",
        "list ros2 pkgs",
        "show ros2 pkgs",
        "print ros2 pkgs",
        "available pkgs",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_pkg_command(console, self.name, "list")


@vulcanai_tool
class Ros2PkgExecutablesTool(AtomicTool):
    name = "ros2_pkg_executables"
    description = (
        "List executable entry points provided by ROS 2 packages. "
        "Equivalent to `ros2 pkg executables`."
        "Used when the user show, display or print a list of ROS 2 package specific executables"
    )
    tags = [
        "ros2",
        "ros 2",
        "exec",
        "executables",
        "pkg executables",
        "ros2 pkg executables",
        "list package executables",
        "show executables",
        "package binaries"
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_pkg_command(console, self.name, "executables")


@vulcanai_tool
class Ros2InterfaceListTool(AtomicTool):
    name = "ros2_interface_list"
    description = (
        "List all currently available ROS 2 interfaces. "
        "Equivalent to `ros2 interface list`. "
        "Use when the user wants to list, show, display, or print ROS 2 interfaces."
    )
    tags = [
        "ros2",
        "ros 2",
        "interfaces",
        "interface list",
        "ros2 serviinterfacece list",
        "list ros2 interfaces",
        "show ros2 interfaces",
        "print ros2 interfaces",
        "available interfaces",
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_interface_command(console, self.name, "list")


@vulcanai_tool
class Ros2InterfacePackagesTool(AtomicTool):
    name = "ros2_interface_packages"
    description = (
        "List ROS 2 packages that provide interfaces."
        "Equivalent to `ros2 interface packages`."
        "Used when the user wants to list, show, display or print the packages of all ROS 2 interfaces"
    )
    tags = [
        "ros2",
        "ros 2",
        "list",
        "pkgs",
        "packages",
        "interface packages",
        "ros2 interface packages",
        "list interface packages",
        "packages with interfaces"
    ]
    input_schema = []
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_interface_command(console, self.name, "packages")


@vulcanai_tool
class Ros2InterfacePackageTool(AtomicTool):
    name = "ros2_interface_package"
    description = (
        "List the interfaces provided by a ROS 2 package. "
        "Equivalent to `ros2 interface package <package_name>`."
        "Used when the user wants to list, show, display or print the ROS 2 interfaces of a package"
    )
    tags = [
        "ros2",
        "ros 2",
        "interfaces"
        "interface package",
        "ros2 interface package",
        "list package interfaces",
        "show package interfaces",
        "package interfaces"
    ]
    input_schema = [("interface_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_interface_command(console, self.name, "package", interface_name=kwargs.get("interface_name"))


@vulcanai_tool
class Ros2InterfaceShowTool(AtomicTool):
    name = "ros2_interface_show"
    description = (
        "Show the definition of a ROS 2 interface. "
        "Equivalent to `ros2 interface show <interface_name>`."
        "Used when the user wants show, display or print the ROS 2 interface information"
    )
    tags = [
        "ros2",
        "ros 2",
        "info",
        "information",
        "interface show",
        "ros2 interface show",
        "show interface",
        "interface definition",
        "display interface"
    ]
    input_schema = [("interface_name", "string")]
    output_schema = {"output": "string"}

    def run(self, **kwargs):
        console = _require_console(self.bb)
        return _run_ros2_interface_command(console, self.name, "show", interface_name=kwargs.get("interface_name"))


# ---------------------------------------------------------------------------
# Backward-compatible legacy wrappers (hidden from planner)
# ---------------------------------------------------------------------------


# @vulcanai_tool
# class Ros2NodeTool(AtomicTool):
#     name = "ros2_node"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 node commands. "
#         "Prefer strict tools: ros2_node_list and ros2_node_info."
#     )
#     tags = ["ros2", "nodes", "cli", "info", "diagnostics"]
#     input_schema = [("command", "string"), ("node_name", "string?")]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_node_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             node_name=kwargs.get("node_name", ""),
#         )


# @vulcanai_tool
# class Ros2TopicTool(AtomicTool):
#     name = "ros2_topic"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 topic commands. "
#         "Prefer strict tools: ros2_topic_*."
#     )
#     tags = ["ros2", "topics", "cli", "info"]
#     input_schema = [
#         ("command", "string"),
#         ("topic_name", "string?"),
#         ("msg_type", "string?"),
#         ("max_duration", "number?"),
#         ("max_lines", "int?"),
#     ]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_topic_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             topic_name=kwargs.get("topic_name"),
#             msg_type=kwargs.get("msg_type"),
#             max_duration=kwargs.get("max_duration"),
#             max_lines=kwargs.get("max_lines"),
#         )


# @vulcanai_tool
# class Ros2ServiceTool(AtomicTool):
#     name = "ros2_service"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 service commands. "
#         "Prefer strict tools: ros2_service_*."
#     )
#     tags = ["ros2", "services", "cli", "info", "call"]
#     input_schema = [
#         ("command", "string"),
#         ("service_name", "string?"),
#         ("service_type", "string?"),
#         ("call", "bool?"),
#         ("args", "string?"),
#         ("max_duration", "number?"),
#         ("max_lines", "int?"),
#     ]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_service_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             service_name=kwargs.get("service_name"),
#             service_type=kwargs.get("service_type"),
#             call_args=kwargs.get("args"),
#             max_duration=kwargs.get("max_duration"),
#             max_lines=kwargs.get("max_lines"),
#         )


# @vulcanai_tool
# class Ros2ActionTool(AtomicTool):
#     name = "ros2_action"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 action commands. "
#         "Prefer strict tools: ros2_action_*."
#     )
#     tags = ["ros2", "actions", "cli", "info", "goal"]
#     input_schema = [
#         ("command", "string"),
#         ("action_name", "string?"),
#         ("action_type", "string?"),
#         ("send_goal", "bool?"),
#         ("goal_args", "string?"),
#     ]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_action_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             action_name=kwargs.get("action_name"),
#             action_type=kwargs.get("action_type"),
#             goal_args=kwargs.get("goal_args"),
#         )


# @vulcanai_tool
# class Ros2ParamTool(AtomicTool):
#     name = "ros2_param"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 param commands. "
#         "Prefer strict tools: ros2_param_*."
#     )
#     tags = ["ros2", "param", "parameters", "cli"]
#     input_schema = [
#         ("command", "string"),
#         ("param_name", "string?"),
#         ("node_name", "string?"),
#         ("set_value", "string?"),
#         ("file_path", "string?"),
#     ]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_param_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             node_name=kwargs.get("node_name"),
#             param_name=kwargs.get("param_name"),
#             set_value=kwargs.get("set_value"),
#             file_path=kwargs.get("file_path"),
#         )


# @vulcanai_tool
# class Ros2PkgTool(AtomicTool):
#     name = "ros2_pkg"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 pkg commands. "
#         "Prefer strict tools: ros2_pkg_list and ros2_pkg_executables."
#     )
#     tags = ["ros2", "pkg", "packages", "cli", "introspection"]
#     input_schema = [("command", "string")]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_pkg_command(console, self.name, kwargs.get("command"))


# @vulcanai_tool
# class Ros2InterfaceTool(AtomicTool):
#     name = "ros2_interface"
#     planner_visible = False
#     description = (
#         "[DEPRECATED] Backward-compatible wrapper for ROS2 interface commands. "
#         "Prefer strict tools: ros2_interface_*."
#     )
#     tags = ["ros2", "interface", "msg", "srv", "cli", "introspection"]
#     input_schema = [("command", "string"), ("interface_name", "string?")]
#     output_schema = {"output": "string"}

#     def run(self, **kwargs):
#         console = _require_console(self.bb)
#         return _run_ros2_interface_command(
#             console,
#             self.name,
#             kwargs.get("command"),
#             interface_name=kwargs.get("interface_name"),
#         )


def import_msg_type(type_str: str, node):
    """
    Dynamically import a ROS 2 message type from its string identifier.

    This function resolves a ROS 2 message type expressed as a string
    (e.g. `"std_msgs/msg/String"`) into the corresponding Python message class
    (`std_msgs.msg.String`).
    """
    info_list = type_str.split("/")

    if len(info_list) != 3:
        pkg = "std_msgs"
        msg_name = info_list[-1]
        node.get_logger().warn(
            f"Cannot import ROS message type '{type_str}'. " + "Adding default pkg 'std_msgs' instead."
        )
    else:
        pkg, _, msg_name = info_list

    module = importlib.import_module(f"{pkg}.msg")

    return getattr(module, msg_name)


# @vulcanai_tool
# class Ros2PublishTool(AtomicTool):
#     name = "ros_publish"
#     description = (
#         "Publish one or more messages to a given ROS 2 topic [topic_name]. "
#         "Or execute 'ros2 topic pub [topic_name]'. "
#         "Supports both simple string messages (for std_msgs/msg/String) and custom message types. "
#         "For custom types, pass message_data as a JSON object with field names and values. "
#         "By default it keeps publishing messages until Ctrl+C is pressed. "
#         "with type 'std_msgs/msg/String' in topic '/chatter' "
#         "with 0.1 seconds of delay between messages to publish"
#         'Example for custom type: msg_type=\'my_pkg/msg/MyMessage\', message_data=\'{"index": 1, "message": "Hello"}\''
#     )
#     tags = ["ros2", "publish", "message", "std_msgs"]

#     input_schema = [
#         ("topic", "string"),  # e.g. "/chatter"
#         ("message_data", "string?"),  # (optional) payload - string for std_msgs/String or JSON for custom types
#         ("msg_type", "string?"),  # (optional) e.g. "std_msgs/msg/String" or "my_pkg/msg/CustomMsg"
#         ("max_lines", "int?"),  # (optional) number of messages to publish
#         ("max_duration", "int?"),  # (optional) stop after this seconds
#         ("period_sec", "float?"),  # (optional) delay between publishes (in seconds)
#         ("message", "string?"),  # (deprecated) use message_data instead
#     ]

#     output_schema = {
#         "published": "bool",
#         "count": "int",
#         "topic": "string",
#         "output": "string",
#     }

#     def msg_from_dict(self, msg, values: dict):
#         """
#         Populate a ROS 2 message instance from a Python dictionary.

#         This function recursively assigns values from a dictionary to the
#         corresponding fields of a ROS 2 message instance.

#         Supports:
#         - Primitive fields (int, float, bool, string)
#         - Nested ROS 2 messages

#         """
#         for field, value in values.items():
#             attr = getattr(msg, field)
#             if hasattr(attr, "__slots__"):
#                 self.msg_from_dict(attr, value)
#             else:
#                 setattr(msg, field, value)

#     def run(self, **kwargs):
#         # Ros2 node to create the Publisher and print the log information
#         node = self.bb.get("main_node", None)
#         if node is None:
#             raise Exception("Could not find shared node, aborting...")
#         # Optional console handle to route logs to the subprocess panel.
#         console = self.bb.get("console", None)

#         result = {
#             "published": "False",
#             "count": "0",
#             "topic": "",
#             "output": "",
#         }

#         panel_enabled = console is not None and hasattr(console, "show_subprocess_panel")
#         if panel_enabled:
#             console.call_from_thread(console.show_subprocess_panel)
#             if hasattr(console, "change_route_logs"):
#                 console.call_from_thread(console.change_route_logs, True)

#         topic_name = kwargs.get("topic", "/chatter")
#         # Support both 'message_data' (new) and 'message' (deprecated)
#         message_data = kwargs.get("message_data", kwargs.get("message", "Hello from VulcanAI PublishTool!"))
#         msg_type_str = kwargs.get("msg_type", "std_msgs/msg/String")

#         max_duration = kwargs.get("max_duration", None)
#         if not isinstance(max_duration, (int, float)) or max_duration <= 0:
#             max_duration = None

#         max_lines = kwargs.get("max_lines", None)
#         if max_lines is not None and not isinstance(max_lines, int):
#             max_lines = None

#         period_sec = kwargs.get("period_sec", 0.1)

#         qos_depth = 10

#         if console is None:
#             print("[ERROR] Console not is None")

#             return result

#         published_msgs = []
#         output_lines = []
#         publisher = None
#         cancel_token = None

#         try:
#             if not topic_name:
#                 console.call_from_thread(console.logger.log_msg, "<gray>[ROS] [ERROR] No topic provided.</gray>")
#                 return result

#             result["topic"] = topic_name

#             if max_lines is not None and max_lines <= 0:
#                 # No messages to publish
#                 console.call_from_thread(
#                     console.logger.log_msg, "<gray>[ROS] [WARN] max_lines <= 0, nothing to publish.</gray>"
#                 )
#                 return result

#             MsgType = import_msg_type(msg_type_str, node)
#             publisher = node.create_publisher(MsgType, topic_name, qos_depth)
#             cancel_token = Future()
#             console.set_stream_task(cancel_token)
#             log_tool_in_stream_and_main(
#                 console,
#                 "[tool]Publisher created![tool]",
#                 tool_name=self.name,
#             )

#             start_time = time.monotonic()
#             published_count = 0

#             while True:
#                 if cancel_token.cancelled():
#                     console.logger.log_tool("[tool]Ctrl+C received:[/tool] stopping publish...", tool_name=self.name)
#                     break
#                 if max_lines is not None and published_count >= max_lines:
#                     console.logger.log_tool(f"[tool]Stopping:[/tool] Reached max_lines = {max_lines}", tool_name=self.name)
#                     break
#                 if max_duration is not None and (time.monotonic() - start_time) >= max_duration:
#                     console.logger.log_tool(
#                         f"[tool]Stopping:[/tool] Exceeded max_duration = {max_duration}s",
#                         tool_name=self.name,
#                     )
#                     break

#                 msg = MsgType()

#                 # Try to populate message based on message type
#                 if hasattr(msg, "data"):
#                     # Standard message type with a 'data' field (e.g., std_msgs/msg/String)
#                     msg.data = message_data
#                 else:
#                     # Custom message type - parse message_data as JSON
#                     try:
#                         payload = json.loads(message_data)
#                         self.msg_from_dict(msg, payload)
#                     except json.JSONDecodeError as e:
#                         console.call_from_thread(
#                             console.logger.log_msg,
#                             "<gray>[ROS] [ERROR] Failed to parse message_data as JSON for custom type"
#                             + f"'{msg_type_str}': {e}</gray>",
#                         )
#                         return result

#                 if hasattr(msg, "data"):
#                     publish_line = f"[ROS] [INFO] Publishing: '{msg.data}'"
#                     console.call_from_thread(console.logger.log_msg, f"<gray>{publish_line}</gray>")
#                 else:
#                     publish_line = f"[ROS] [INFO] Publishing custom message to '{topic_name}'"
#                     console.call_from_thread(
#                         console.logger.log_msg,
#                         f"<gray>{publish_line}</gray>",
#                     )
#                 output_lines.append(publish_line)
#                 publisher.publish(msg)
#                 published_msgs.append(msg.data if hasattr(msg, "data") else str(msg))
#                 published_count += 1

#                 rclpy.spin_once(node, timeout_sec=0.05)

#                 if period_sec and period_sec > 0.0:
#                     time.sleep(period_sec)

#         finally:
#             console.set_stream_task(None)
#             if panel_enabled:
#                 if hasattr(console, "change_route_logs"):
#                     console.call_from_thread(console.change_route_logs, False)
#             if publisher is not None:
#                 try:
#                     node.destroy_publisher(publisher)
#                 except Exception:
#                     pass

#         result["output"] = "\n".join(output_lines)

#         if published_msgs is not None:
#             result["published"] = "True"
#             result["count"] = len(published_msgs)

#         print_tool_output(console, result["output"], self.name)

#         # if panel_enabled:
#         #     console.logger.log_msg(result["output"], color="gray")
#         #     console.logger.log_msg("\n")
#         # else:
#         #     print_tool_output(console, result["output"], self.name)
#         return result


# @vulcanai_tool
# class Ros2SubscribeTool(AtomicTool):
#     name = "ros_subscribe"
#     description = (
#         "Subscribe to a topic [topic] or execute 'ros2 topic echo [topic]' "
#         "and stop after receiving N messages or max duration."
#     )
#     tags = ["ros2", "subscribe", "topic", "std_msgs"]

#     input_schema = [
#         ("topic", "string"),  # topic name
#         ("max_lines", "int?"),  # (optional) stop after this number of messages
#         ("max_duration", "int?"),  # (optional) stop after this seconds
#     ]

#     output_schema = {
#         "subscribed": "bool",
#         "count": "int",
#         "topic": "string",
#         "output": "string",
#     }

#     def run(self, **kwargs):
#         # Ros2 node to create the Publisher and print the log information
#         node = self.bb.get("main_node", None)
#         if node is None:
#             raise Exception("Could not find shared node, aborting...")
#         # Optional console handle to support Ctrl+C cancellation.
#         console = self.bb.get("console", None)

#         result = {
#             "subscribed": "False",
#             "count": "0",
#             "topic": "",
#             "output": "",
#         }

#         topic_name = kwargs.get("topic", None)
#         max_duration = kwargs.get("max_duration", None)
#         if max_duration is not None and not isinstance(max_duration, (int, float)):
#             max_duration = None

#         max_lines = kwargs.get("max_lines", None)
#         if max_lines is not None and not isinstance(max_lines, int):
#             max_lines = None

#         # Start line (stream panel/main panel with tool color)
#         console.logger.log_tool("[tool]Subscriber created![/tool]", tool_name=self.name)

#         # "--field data" prints only the data field from each message
#         # instead of the full YAML message
#         # "--no-arr" do not print array fields of messages
#         base_args = ["ros2", "topic", "echo", topic_name, "--field", "data", "--no-arr"]
#         ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines, log_created=False)

#         ret_lines = ret.splitlines() if isinstance(ret, str) and ret else []

#         result["output"] = "\n".join(ret_lines)

#         if ret is not None:
#             result["subscribed"] = "True"
#             result["count"] = len(ret_lines)
#             result["topic"] = topic_name

#         print_tool_output(console, result["output"], self.name)

#         return result
