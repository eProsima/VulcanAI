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

from vulcanai import AtomicTool, vulcanai_tool
from vulcanai.tools.utils import execute_subprocess, run_oneshot_cmd

import importlib
import json
import rclpy
import subprocess # TODO. danip
from std_msgs.msg import String
import time


"""topics = topic_name_list.splitlines()

# TODO. in all commands
# Will be updated in the TUI Migration PR.
# The PR adds a modalscreen to select the most similar string),
# this applies to all ros cli commands. Though, not implemented
# in the rest commands from this PR
if topic_name not in topics:
    topic_similar = search_similar(topics, topic_name)
    topic_name = topic_similar"""

"""
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


@vulcanai_tool
class Ros2NodeTool(AtomicTool):
    name = "ros2_node"
    description = (
        "Wrapper for `ros2 node` CLI."
        "Run any subcommand: 'list', 'info'"
        "With an optional argument 'node_name' for 'info' subcommand."
    )
    description = "List ROS2 nodes and optionally get detailed info for a specific node."
    tags = ["ros2", "nodes", "cli", "info", "diagnostics"]

    input_schema = [
        ("node_name", "string?") # (optional) Name of the ros2 node.
                                 # if the node is not provided the command is `ros2 node list`.
                                 # otherwise `ros2 node info <node_name>`
    ]

    output_schema = {
        "ros2": "bool",     # ros2 flag for pretty printing.
        "output": "string", # list of ros2 nodes or info of a node.
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        # Get the node name if provided by the query
        node_name = kwargs.get("node_name", None)

        result = {
            "ros2": True,
            "output": "",
        }

        # -- Run `ros2 node list` ---------------------------------------------
        if node_name == None:
            node_name_list = run_oneshot_cmd(["ros2", "node", "list"])
            result["output"] = node_name_list

        # -- Run `ros2 node info <node>` --------------------------------------
        else:
            if not node_name:
                raise ValueError("`command='info'` requires `node_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "node", "info", node_name]
            )
            result["output"] = info_output

        return result


@vulcanai_tool
class Ros2TopicTool(AtomicTool):
    name = "ros2_topic"
    description = (
        "Wrapper for `ros2 topic` CLI."
        "Run any subcommand: 'list', 'info', 'find', 'type', 'echo', 'bw', 'delay', 'hz', 'pub'."
        "With optional arguments like 'topic_name', 'message_type', 'max_duration' or 'max_lines'"
    )
    tags = ["ros2", "topics", "cli", "info"]

    # - `command` lets you pick a single subcommand (echo/bw/hz/delay/find/pub/type).
    input_schema = [
        ("command", "string"),       # Command
        ("topic_name", "string?"),   # (optional) Topic name. (info/echo/bw/delay/hz/type/pub)
        ("msg_type", "string?"),     # (optional) Message type (`find` <type>, `pub` <message> <type>)
        ("max_duration", "number?"), # (optional) Seconds for streaming commands (echo/bw/hz/delay)
        ("max_lines", "int?"),       # (optional) Cap number of lines for streaming commands
    ]

    output_schema = {
        "ros2": "bool",            # ros2 flag for pretty printing
        "output": "string",        # output
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)  # optional explicit subcommand
        topic_name = kwargs.get("topic_name", None)
        msg_type = kwargs.get("msg_type", None)
        # Streaming commands variables
        max_duration = kwargs.get("max_duration", 60.0)
        max_lines = kwargs.get("max_lines", 1000)

        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        topic_name_list = run_oneshot_cmd(["ros2", "topic", "list"])

        # -- ros2 topic list --------------------------------------------------
        if command == "list":
            result["output"] = topic_name_list

        # -- ros2 topic info <topic_name> -------------------------------------
        elif command == "info":
            if not topic_name:
                raise ValueError("`command='info'` requires `topic_name`.")

            """topics = topic_name_list.splitlines()

            # TODO. Will be updated in the TUI Migration PR.
            # The PR adds a modalscreen to select the most similar string),
            # this applies to all ros cli commands. Though, not implemented
            # in the rest commands from this PR
            if topic_name not in topics:
                topic_similar = search_similar(topics, topic_name)
                topic_name = topic_similar"""

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "info", topic_name]
            )
            result["output"] = info_output

        # -- ros2 topic find <type> -------------------------------------------
        elif command == "find":
            if not msg_type:
                raise ValueError("`command='find'` requires `msg_type` (ROS type).")

            find_output = run_oneshot_cmd(
                ["ros2", "topic", "find", msg_type]
            )
            find_topics = [
                line.strip() for line in find_output.splitlines() if line.strip()
            ]
            result["output"] = ', '.join(find_topics)

        # -- ros2 topic type <topic_name> -------------------------------------
        elif command == "type":
            if not topic_name:
                raise ValueError("`command='type'` requires `topic_name`.")

            type_output = run_oneshot_cmd(
                ["ros2", "topic", "type", topic_name]
            )
            result["output"] = type_output

        # -- ros2 topic echo <topic_name> -------------------------------------
        elif command == "echo":
            if not topic_name:
                raise ValueError("`command='echo'` requires `topic_name`.")

            base_args = ["ros2", "topic", "echo", topic_name]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- ros2 topic bw <topic_name> ---------------------------------------
        elif command == "bw":
            base_args = ["ros2", "topic", "bw", topic_name]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- ros2 topic delay <topic_name> ------------------------------------
        elif command == "delay":
            if not topic_name:
                raise ValueError("`command='delay'` requires `topic_name`.")

            base_args = ["ros2", "topic", "delay", topic_name]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- ros2 topic hz <topic_name> ---------------------------------------
        elif command == "hz":
            if not topic_name:
                raise ValueError("`command='hz'` requires `topic_name`.")

            base_args = ["ros2", "topic", "hz", topic_name]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- publisher --------------------------------------------------------
        elif command == "pub":
            # One-shot publish using `-1`
            # ros2 topic pub -1 <topic> <msg_type> "<data>"
            # ros2 topic pub -1 /rosout2 std_msgs/msg/String "{data: 'Hello'}"
            if not topic_name:
                raise ValueError("`command='pub'` requires `topic_name`.")
            if not msg_type:
                raise ValueError("`command='pub'` requires `msg_type`.")

            """# only send 1
            pub_output = run_oneshot_cmd(
                ["ros2", "topic", "pub", "-1", topic_name, msg_type]
            )
            result["output"] = pub_output"""

            # TODO. expand publisher options?

            base_args = ["ros2", "topic", "pub", topic_name, msg_type]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
            )

        return result


@vulcanai_tool
class Ros2ServiceTool(AtomicTool):
    name = "ros2_service"
    description = (
        "Wrapper for `ros2 service` CLI."
        "Run any subcommand: 'list', 'info', 'type', 'find', 'call', 'echo'."
        "With optional arguments like 'service_name', 'service_type', "
        "'call', 'args', 'max_duration' or 'max_lines'"
    )
    tags = ["ros2", "services", "cli", "info", "call"]

    # - `command` = "list", "info", "type", "call", "echo", "find"
    input_schema = [
        ("command", "string"),         # Command
        ("service_name", "string?"),   # (optional) Service name. "info", "type", "call", "echo"
        ("service_type", "string?"),   # (optional) Service type. "find", "call"
        ("call", "bool?"),             # (optional) backwards-compatible call flag
        ("args", "string?"),           # (optional) YAML/JSON-like request data for `call`
        ("max_duration", "number?"),   # (optional) Maximum duration
        ("max_lines", "int?"),         # (optional) Maximum lines
    ]

    output_schema = {
        "ros2": "bool",                # ros2 flag for pretty printing
        "output": "string",            # `ros2 service list`
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        service_name = kwargs.get("service_name", None)
        service_type = kwargs.get("service_type", None)
        call_args = kwargs.get("args", None)
        # Streaming commands variables
        max_duration = kwargs.get("max_duration", 2.0) # default for echo
        max_lines = kwargs.get("max_lines", 50)

        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        service_name_list = run_oneshot_cmd(["ros2", "service", "list"])

        # -- ros2 service list ------------------------------------------------
        if command == "list":
            result["output"] = service_name_list

        # -- ros2 service info <service_name> ---------------------------------
        elif command == "info":
            if not service_name:
                raise ValueError("`command='info'` requires `service_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "service", "info", service_name]
            )
            result["output"] = info_output

        # -- ros2 service type <service_name> ---------------------------------
        elif command == "type":
            if not service_name:
                raise ValueError("`command='type'` requires `service_name`.")

            type_output = run_oneshot_cmd(
                ["ros2", "service", "type", service_name]
            )
            result["output"] = type_output.strip()

        # -- ros2 service find <type> -----------------------------------------
        elif command == "find":
            if not service_type:
                raise ValueError("`command='find'` requires `service_type`.")

            find_output = run_oneshot_cmd(
                ["ros2", "service", "find", service_type]
            )
            result["output"] = find_output

        # -- ros2 service call service_name service_type ----------------------
        elif command == "call":
            if not service_name:
                raise ValueError("`command='call'` requires `service_name`.")
            if call_args is None:
                raise ValueError("`command='call'` requires `args`.")

            # If service_type not given, detect it
            if not service_type:
                type_output = run_oneshot_cmd(
                    ["ros2", "service", "type", service_name]
                )
                service_type = type_output.strip()

            call_output = run_oneshot_cmd(
                ["ros2", "service", "call", service_name, service_type, call_args]
            )
            result["output"] = call_output

        # -- ros2 service echo service_name -----------------------------------
        elif command == "echo":
            if not service_name:
                raise ValueError("`command='echo'` requires `service_name`.")

            base_args = ["ros2", "service", "echo", service_name]
            execute_subprocess(console, base_args, max_duration, max_lines)

            result["output"] = "True"
            result["ros2"] = True

        # -- unknown ------------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, type, call, echo, find."
            )

        return result


@vulcanai_tool
class Ros2ActionTool(AtomicTool):
    name = "ros2_action"
    description = (
        "Wrapper for `ros2 action` CLI."
        "Run any subcommand: 'list', 'info', 'type', 'send_goal'."
        "With optional arguments like 'action_name', 'action_type', "
        "'goal_args'"
    )
    tags = ["ros2", "actions", "cli", "info", "goal"]

    # - `command`: "list", "info", "type", "send_goal"
    input_schema = [
        ("command", "string"),        # Command
        ("action_name", "string?"),   # (optional) Action name
        ("action_type", "string?"),   # (optional) Action type. "find"
        ("send_goal", "bool?"),       # (optional) legacy flag (backwards compatible)
        ("goal_args", "string?"),          # (optional) goal YAML, e.g. '{order: 5}'
    ]

    output_schema = {
        "ros2": "bool",      # ros2 flag for pretty printing
        "output": "string",  # `ros2 action list`
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        command = kwargs.get("command", None)
        action_name = kwargs.get("action_name", None)
        action_type = kwargs.get("action_type", None)
        goal_args = kwargs.get("goal_args", None)

        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        action_name_list = run_oneshot_cmd(["ros2", "action", "list"])

        # -- ros2 action list -------------------------------------------------
        if command == "list":
            result["output"] = action_name_list

        # -- ros2 action info <action_name> -----------------------------------
        elif command == "info":
            if not action_name:
                raise ValueError("`command='info'` requires `action_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "action", "info", action_name]
            )
            result["output"] = info_output

        # -- ros2 action type <type_name ---------------------------------------------------------------
        elif command == "type":
            if not action_name:
                raise ValueError("`command='type'` requires `action_name`.")

            type_output = run_oneshot_cmd(
                ["ros2", "action", "type", action_name]
            )
            result["output"] = type_output

        # send_goal -----------------------------------------------------------
        elif command == "send_goal":
            if not action_name:
                raise ValueError("`command='send_goal'` requires `action_name`.")

            # Use explicit type if provided, otherwise detect it
            if not action_type:
                type_output = run_oneshot_cmd(
                    ["ros2", "action", "type", action_name]
                )
                action_type = type_output.strip()

            args_list = ["ros2", "action", "send_goal", action_name, action_type]
            if goal_args is not None:
                args_list.extend(goal_args)

            goal_output = run_oneshot_cmd(args_list)
            result["output"] = goal_output

        # -- unknown ------------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, type, send_goal."
            )

        return result


@vulcanai_tool
class Ros2ParamTool(AtomicTool):
    name = "ros2_param"
    description = (
        "Wrapper for `ros2 param` CLI."
        "Run any subcommand: 'list', 'get', 'describe', 'set', 'delete', 'dump', 'load'."
        "With optional arguments like 'param_name', 'node_name', "
        "'set_value', 'file_path'"
    )
    tags = ["ros2", "param", "parameters", "cli"]

    # - `command`: "list", "get", "describe", "set", "delete", "dump", "load"
    input_schema = [
        ("command", "string"),     # Command
        ("param_name", "string?"), # (optional) Parameter name
        ("node_name", "string?"),  # (optional) Target node
        ("set_value", "string?"),  # (optional) value for set
        ("file_path", "string?"),  # (optional) for dump/load YAML file
    ]

    output_schema = {
        "ros2": "bool",     # ros2 flag for pretty printing
        "output": "string",
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        command = kwargs.get("command", None)
        node = kwargs.get("node_name", None)
        param = kwargs.get("param_name", None)
        set_value = kwargs.get("set_value", None)
        file_path = kwargs.get("file_path", None)

        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        # -- ros2 param list` -------------------------------------------------
        if command == "list":
            try:
                if node:
                    list_cmd = ["ros2", "param", "list", node]
                else:
                    list_cmd = ["ros2", "param", "list"]

                list_output = run_oneshot_cmd(list_cmd)
                result["output"] = list_output

            except Exception as e:
                raise Exception(str(e))

        # -- ros2 param get <node> <param> ------------------------------------
        elif command == "get":
            if not node or not param:
                raise ValueError("`command='get'` requires `node_name` and `param_name`.")

            get_output = run_oneshot_cmd(
                ["ros2", "param", "get", node, param]
            )
            result["output"] = get_output

        # -- ros2 param describe <node> <param> -------------------------------
        elif command == "describe":
            if not node or not param:
                raise ValueError("`command='describe'` requires `node_name` and `param_name`.")

            describe_output = run_oneshot_cmd(
                ["ros2", "param", "describe", node, param]
            )
            result["output"] = describe_output

        # -- ros2 param set <node> <param> <set_value> ------------------------
        elif command == "set":
            if not node or not param:
                raise ValueError("`command='set'` requires `node_name` and `param_name`.")
            if set_value is None:
                raise ValueError("`command='set'` requires `set_value`.")

            set_output = run_oneshot_cmd(
                ["ros2", "param", "set", node, param, set_value]
            )
            result["output"] = set_output

        # -- ros2 param delete <node> <parm> ----------------------------------
        elif command == "delete":
            if not node or not param:
                raise ValueError("`command='delete'` requires `node_name` and `param_name`.")

            delete_output = run_oneshot_cmd(
                ["ros2", "param", "delete", node, param]
            )
            result["output"] = delete_output

        # -- ros2 param dump <node> [file_path] -------------------------------
        elif command == "dump":
            if not node:
                raise ValueError("`command='dump'` requires `node_name`.")

            # Two modes:
            # - If file_path given, write to file with --output-file
            # - Otherwise, capture YAML from stdout
            if file_path:
                dump_output = run_oneshot_cmd(
                    ["ros2", "param", "dump", node, "--output-file", file_path]
                )
                # CLI usually prints a line like "Saved parameters to file..."
                # so we just expose that.
                result["output"] = dump_output or f"Dumped parameters to {file_path}"
            else:
                dump_output = run_oneshot_cmd(
                    ["ros2", "param", "dump", node]
                )
                result["output"] = dump_output

        # -- ros2 param load <node> <file_path> -------------------------------
        elif command == "load":
            if not node or not file_path:
                raise ValueError("`command='load'` requires `node_name` and `file_path`.")

            load_output = run_oneshot_cmd(
                ["ros2", "param", "load", node, file_path]
            )
            result["output"] = load_output

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, get, describe, set, delete, dump, load."
            )

        return result


@vulcanai_tool
class Ros2PkgTool(AtomicTool):
    name = "ros2_pkg"
    description = (
        "Wrapper for `ros2 pkg` CLI."
        "Run any subcommand: 'list', 'executables'."
    )
    tags = ["ros2", "pkg", "packages", "cli", "introspection"]

    # If package_name is not provided, runs: `ros2 pkg list`
    # If provided, runs: `ros2 pkg executables <package_name>`
    input_schema = [
        ("command", "string"),  # Command
    ]

    output_schema = {
        "ros2": "bool",     # ros2 flag for pretty printing.
        "output": "string", # list of packages or list of executables for a package.
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        # Get the package name if provided by the query
        command = kwargs.get("command", None)
        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        # -- Run `ros2 pkg list` ----------------------------------------------
        if command == "list":
            pkg_name_list = run_oneshot_cmd(["ros2", "pkg", "list"])
            result["output"] = pkg_name_list

        # -- Run `ros2 pkg executables` ---------------------------------------
        elif command == "executables":
            pkg_name_list = run_oneshot_cmd(["ros2", "pkg", "executables"])
            result["output"] = pkg_name_list

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, executables, prefix, xml"
            )

        return result


@vulcanai_tool
class Ros2InterfaceTool(AtomicTool):
    name = "ros2_interface"
    description = (
        "Wrapper for `ros2 interface` CLI."
        "Run any subcommand: 'list', 'packages', 'package', 'show'."
        "With optional arguments like 'interface_name'."
    )
    tags = ["ros2", "interface", "msg", "srv", "cli", "introspection"]

    # - `command` lets you pick a single subcommand (list/packages/package).
    input_schema = [
        ("command", "string"),         # Command
        ("interface_name", "string?"), # (optional) Name of the interface, e.g. "std_msgs/msg/String".
                                       # If not provided, the command is `ros2 interface list`.
                                       # Otherwise `ros2 interface show <interface_name>`.
    ]

    output_schema = {
        "ros2": "bool",     # ros2 flag for pretty printing.
        "output": "string", # list of interfaces (as list of strings) or full interface definition.
    }

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        # Get the interface name if provided by the query
        command = kwargs.get("command", None)
        interface_name = kwargs.get("interface_name", None)

        result = {
            "ros2": True,
            "output": "",
        }

        command = command.lower()

        # -- ros2 interface list ----------------------------------------------
        if interface_name is None:
            interface_name_list = run_oneshot_cmd(["ros2", "interface", "list"])
            result["output"] = interface_name_list

        # -- ros2 interface packages ------------------------------------------
        elif command == "packages":
            interface_pkg_name_list = run_oneshot_cmd(["ros2", "interface", "packages"])
            result["output"] = interface_pkg_name_list

        # -- ros2 interface package <interface_name> --------------------------------
        elif command == "package":
            if not interface_name:
                raise ValueError("`command='package'` requires `interface_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "package", interface_name]
            )
            result["output"] = info_output

        # -- ros2 interface show <interface_name> --------------------------------
        elif command == "show":
            if not interface_name:
                raise ValueError("`command='package'` requires `interface_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "show", interface_name]
            )
            result["output"] = info_output

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
            )

        return result


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
        node.get_logger().warn(f"Cannot import ROS message type '{type_str}'. " + \
                                "Adding default pkg 'std_msgs' instead.")
    else:
        pkg, _, msg_name = info_list

    module = importlib.import_module(f"{pkg}.msg")

    return getattr(module, msg_name)


@vulcanai_tool
class Ros2PublishTool(AtomicTool):
    name = "ros_publish"
    description = (
        "Publish one or more messages to a given ROS 2 topic. "
        "Supports both simple string messages (for std_msgs/msg/String) and custom message types. "
        "For custom types, pass message_data as a JSON object with field names and values. "
        "By default 10 messages 'Hello from VulcanAI PublishTool!' "
        "with type 'std_msgs/msg/String' in topic '/chatter' "
        "with 0.1 seconds of delay between messages to publish with a qos_depth of 10. "
        "Example for custom type: msg_type='my_pkg/msg/MyMessage', message_data='{\"index\": 1, \"message\": \"Hello\"}'"
    )
    tags = ["ros2", "publish", "message", "std_msgs"]

    input_schema = [
        ("topic", "string"),         # e.g. "/chatter"
        ("message_data", "string?"), # (optional) payload - string for std_msgs/String or JSON for custom types
        ("msg_type", "string?"),     # (optional) e.g. "std_msgs/msg/String" or "my_pkg/msg/CustomMsg"
        ("count", "int?"),           # (optional) number of messages to publish
        ("period_sec", "float?"),    # (optional) delay between publishes (in seconds)
        ("qos_depth", "int?"),       # (optional) publisher queue depth
        ("message", "string?"),      # (deprecated) use message_data instead
    ]

    output_schema = {
        "published": "bool",
        "count": "int",
        "topic": "string"
    }

    def msg_from_dict(self, msg, values: dict):
        """
        Populate a ROS 2 message instance from a Python dictionary.

        This function recursively assigns values from a dictionary to the
        corresponding fields of a ROS 2 message instance.

        Supports:
        - Primitive fields (int, float, bool, string)
        - Nested ROS 2 messages

        """
        for field, value in values.items():
            attr = getattr(msg, field)
            if hasattr(attr, "__slots__"):
                self.msg_from_dict(attr, value)
            else:
                setattr(msg, field, value)

    def run(self, **kwargs):

        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        topic = kwargs.get("topic", "/chatter")
        # Support both 'message_data' (new) and 'message' (deprecated)
        message_data = kwargs.get("message_data", kwargs.get("message", "Hello from VulcanAI PublishTool!"))
        msg_type_str = kwargs.get("msg_type", "std_msgs/msg/String")
        count = kwargs.get("count", 10)
        period_sec = kwargs.get("period_sec", 0.1)
        qos_depth = kwargs.get("qos_depth", 10.0)


        if not topic:
            node.get_logger().error("No topic provided.")
            return {"published": False, "count": 0, "topic": topic}

        if count <= 0:
            node.get_logger().warn("Count <= 0, nothing to publish.")
            return {"published": True, "count": 0, "topic": topic}


        MsgType = import_msg_type(msg_type_str, node)
        publisher = node.create_publisher(MsgType, topic, qos_depth)

        published_count = 0
        for i in range(count):
            msg = MsgType()

            # TODO. danip Check custom messages implementation
            """
            E.G.:
            PlanNode 1: kind=SEQUENCE
	        Step 1: ros_publish(topic=/custom_topic, msg_type=my_pkg/msg/CustomMsg, message_data={"id":1,"text":"Custom message 1"}, count=1, period_sec=0.1, qos_depth=10)

            [EXECUTOR] Invoking 'ros_publish' with args:'{'topic': '/custom_topic', 'msg_type': 'my_pkg/msg/CustomMsg', 'message_data': '{"id":1,"text":"Custom message 1"}', 'count': '1', 'period_sec': '0.1', 'qos_depth': '10'}'
            [EXECUTOR] Execution failed for 'ros_publish': No module named 'my_pkg'
            [EXECUTOR] Step 'ros_publish' attempt 1/1 failed
            [EXECUTOR] [ERROR] PlanNode SEQUENCE failed on attempt 1/1
            """

            # Try to populate message based on message type
            if hasattr(msg, "data"):
                # Standard message type with a 'data' field (e.g., std_msgs/msg/String)
                msg.data = message_data
            else:
                # Custom message type - parse message_data as JSON
                try:
                    payload = json.loads(message_data)
                    self.msg_from_dict(msg, payload)
                except json.JSONDecodeError as e:
                    node.get_logger().error(
                        f"Failed to parse message_data as JSON for custom type '{msg_type_str}': {e}"
                    )
                    return {"published": False, "count": 0, "topic": topic}

            with node.node_lock:
                if hasattr(msg, "data"):
                    node.get_logger().info(f"Publishing: '{msg.data}'")
                else:
                    node.get_logger().info(f"Publishing custom message to '{topic}'")
                publisher.publish(msg)

            published_count += 1

            rclpy.spin_once(node, timeout_sec=0.05)

            if period_sec and period_sec > 0.0:
                time.sleep(period_sec)

        return {"published": True, "count": published_count, "topic": topic}


@vulcanai_tool
class Ros2SubscribeTool(AtomicTool):
    name = "ros_subscribe"
    description = "Subscribe to a topic and stop after receiving N messages or a timeout."
    description = (
        "Subscribe to a given ROS 2 topic and stop after receiven N messages or a timeout."
        "By default 100 messages and 300 seconds duration and a qos_depth of 10"
    )
    tags = ["ros2", "subscribe", "topic", "std_msgs"]

    input_schema = [
        ("topic", "string"),            # topic name
        ("msg_type", "string"),         # e.g. "std_msgs/msg/String"
        ("output_format", "string"),    # "data" | "dict"
        ("max_messages", "int?"),       # (optional) stop after this number of messages
        ("timeout_sec", "float?"),      # (optional) stop after this seconds
        ("qos_depth", "int?"),          # (optional) subscription queue depth
    ]

    output_schema = {
        "success": "bool",
        "received": "int",
        "messages": "list",
        "reason": "string",
        "topic": "string",
    }


    def msg_to_dict(self, msg):
        """
        Convert a ROS 2 message instance into a Python dictionary.

        This function recursively converts a ROS 2 message into a dictionary
        using ROS 2 Python introspection (`__slots__`).

        Supports:
        - Primitive fields
        - Nested ROS 2 messages
        """
        out = {}
        for field in getattr(msg, "__slots__", []):
            key = field.lstrip("_")
            val = getattr(msg, field)
            if hasattr(val, "__slots__"):
                out[key] = self.msg_to_dict(val)
            elif isinstance(val, (list, tuple)):
                out[key] = [self.msg_to_dict(v) if hasattr(v, "__slots__") else v for v in val]
            else:
                out[key] = val
        return out


    def run(self, **kwargs):

        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        topic = kwargs.get("topic", None)
        msg_type_str = kwargs.get("msg_type", "std_msgs/msg/String")
        max_messages = kwargs.get("max_messages", 100)
        timeout_sec = kwargs.get("timeout_sec", 300.0)
        qos_depth = kwargs.get("qos_depth", 10.0)
        output_format = kwargs.get("output_format", "data")


        if not topic:
            return {"success": False, "received": 0, "messages": [], "reason": "no_topic", "topic": topic}

        if max_messages <= 0:
            return {"success": True, "received": 0, "messages": [], "reason": "max_messages<=0", "topic": topic}

        if not msg_type_str:
            msg_type_str = "std_msgs/msg/String"

        received_msgs = []

        def callback(msg: String):
            # received_msgs.append(msg.data)
            # node.get_logger().info(f"I heard: [{msg.data}]")
            if output_format == "data" and hasattr(msg, "data"):
                received_msgs.append(msg.data)
                node.get_logger().info(f"I heard: [{msg.data}]")
            else:
                d = self.msg_to_dict(msg)
                received_msgs.append(d["data"] if "data" in d else d)
                node.get_logger().info(f"I heard: [{d["data"]}]")

        MsgType = import_msg_type(msg_type_str, node)
        sub = node.create_subscription(MsgType, topic, callback, qos_depth)

        start = time.monotonic()
        reason = "timeout"

        try:
            while rclpy.ok():
                # Stop conditions
                if len(received_msgs) >= max_messages:
                    reason = "max_messages"
                    break
                if (time.monotonic() - start) >= timeout_sec:
                    reason = "timeout"
                    break

                rclpy.spin_once(node, timeout_sec=0.1)

        finally:
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass

        return {
            "success": True,
            "received": len(received_msgs),
            "messages": received_msgs,
            "reason": reason,
            "topic": topic,
        }



