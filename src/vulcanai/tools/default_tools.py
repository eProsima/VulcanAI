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
import time
from concurrent.futures import Future

from vulcanai import AtomicTool, vulcanai_tool
from vulcanai.tools.utils import execute_subprocess, last_output_lines, run_oneshot_cmd, suggest_string

# ROS2 imports
try:
    import rclpy
except ImportError:
    raise ImportError("Unable to load default tools because no ROS 2 installation was found.")

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
    tags = ["ros2", "nodes", "cli", "info", "diagnostics"]

    # - `command` lets you pick a single subcommand (list/info).
    input_schema = [
        ("command", "string"),  # Command
        ("topic_name", "string?"),  # (optional) Topic name. (info/bw/delay/hz/type/pub)
    ]

    output_schema = {
        "output": "string",  # list of ros2 nodes or info of a node.
    }

    def run(self, **kwargs):
        # Used in the suggestion string
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)  # optional explicit subcommand
        node_name = kwargs.get("node_name", "")

        result = {
            "output": "",
        }

        command = command.lower()

        # -- Node name suggestions --
        node_name_list_str = run_oneshot_cmd(["ros2", "node", "list"])
        node_name_list = node_name_list_str.splitlines()

        # -- Run `ros2 node list` ---------------------------------------------
        if command == "list":
            result["output"] = node_name_list_str

        # -- Run `ros2 node info <node>` --------------------------------------
        else:
            # Check if the topic is not available ros2 topic list
            # if it is not create a window for the user to choose a correct topic name
            suggested_topic = suggest_string(console, self.name, "Node", node_name, node_name_list)
            if suggested_topic is not None:
                node_name = suggested_topic

            if not node_name:
                raise ValueError("`command='{}'` requires `node_name`.".format("info"))

            info_output = run_oneshot_cmd(["ros2", "node", "info", node_name])
            result["output"] = info_output

        return result


@vulcanai_tool
class Ros2TopicTool(AtomicTool):
    name = "ros2_topic"
    description = (
        "Wrapper for `ros2 topic` CLI."
        "Run any subcommand: 'list', 'info', 'find', 'type', 'bw', 'delay', 'hz', 'pub'."
        "With optional arguments like 'topic_name', 'message_type', 'max_duration' or 'max_lines'"
    )
    tags = ["ros2", "topics", "cli", "info"]

    # - `command` lets you pick a single subcommand (bw/hz/delay/find/pub/type).
    input_schema = [
        ("command", "string"),  # Command
        ("topic_name", "string?"),  # (optional) Topic name. (info/bw/delay/hz/type/pub)
        ("msg_type", "string?"),  # (optional) Message type (`find` <type>, `pub` <message> <type>)
        ("max_duration", "number?"),  # (optional) Seconds for streaming commands (bw/hz/delay)
        ("max_lines", "int?"),  # (optional) Cap number of lines for streaming commands
    ]

    output_schema = {
        "output": "string",  # output
    }

    def run(self, **kwargs):
        # Used in the suggestion string
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
            "output": "",
        }

        command = command.lower()

        topic_name_list_str = run_oneshot_cmd(["ros2", "topic", "list"])
        topic_name_list = topic_name_list_str.splitlines()

        # -- Topic name suggestions --
        if command == "find":
            # TODO?
            """suggested_type = suggest_string(console, self.name, "Topic", msg_type, topic_name_list)
            if suggested_type is not None:
                msg_type = suggested_type"""
        elif command != "list":
            # Check if the topic is not available ros2 topic list
            # if it is not create a window for the user to choose a correct topic name
            suggested_topic_name = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic_name is not None:
                topic_name = suggested_topic_name

            # Check if the topic_name is null (suggest_string() failed)
            if not topic_name:
                raise ValueError("`command='{}'` requires `topic_name`.".format(command))

        # -- Commands --
        # -- ros2 topic list --------------------------------------------------
        if command == "list":
            result["output"] = topic_name_list_str

        # -- ros2 topic info <topic_name> -------------------------------------
        elif command == "info":
            info_output = run_oneshot_cmd(["ros2", "topic", "info", topic_name])
            result["output"] = info_output

        # -- ros2 topic find <type> -------------------------------------------
        elif command == "find":
            find_output = run_oneshot_cmd(["ros2", "topic", "find", msg_type])
            find_topics = [line.strip() for line in find_output.splitlines() if line.strip()]
            result["output"] = ", ".join(find_topics)

        # -- ros2 topic type <topic_name> -------------------------------------
        elif command == "type":
            type_output = run_oneshot_cmd(["ros2", "topic", "type", topic_name])
            result["output"] = type_output

        # -- ros2 topic bw <topic_name> ---------------------------------------
        elif command == "bw":
            base_args = ["ros2", "topic", "bw", topic_name]
            ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines)
            result["output"] = last_output_lines(console, self.name, ret, max_lines=10)

        # -- ros2 topic delay <topic_name> ------------------------------------
        elif command == "delay":
            base_args = ["ros2", "topic", "delay", topic_name]
            ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines)
            result["output"] = last_output_lines(console, self.name, ret, max_lines=10)

        # -- ros2 topic hz <topic_name> ---------------------------------------
        elif command == "hz":
            base_args = ["ros2", "topic", "hz", topic_name]
            ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines)
            result["output"] = last_output_lines(console, self.name, ret, max_lines=10)

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
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
        ("command", "string"),  # Command
        ("service_name", "string?"),  # (optional) Service name. "info", "type", "call", "echo"
        ("service_type", "string?"),  # (optional) Service type. "find", "call"
        ("call", "bool?"),  # (optional) backwards-compatible call flag
        ("args", "string?"),  # (optional) YAML/JSON-like request data for `call`
        ("max_duration", "number?"),  # (optional) Maximum duration
        ("max_lines", "int?"),  # (optional) Maximum lines
    ]

    output_schema = {
        "output": "string",  # `ros2 service list`
    }

    def run(self, **kwargs):
        # Used in the suggestion string
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        service_name = kwargs.get("service_name", None)
        service_type = kwargs.get("service_type", None)
        call_args = kwargs.get("args", None)
        # Streaming commands variables
        max_duration = kwargs.get("max_duration", 2.0)  # default for echo
        max_lines = kwargs.get("max_lines", 50)

        result = {
            "output": "",
        }

        command = command.lower()

        service_name_list_str = run_oneshot_cmd(["ros2", "service", "list"])
        service_name_list = service_name_list_str.splitlines()

        # -- Service name suggestions --
        if command == "find":
            # TODO?
            """suggested_type = suggest_string(console, self.name, "Service_Type", service_type, service_name_list)
            if suggested_type is not None:
                service_type = suggested_type"""

        elif command != "list":
            # Check if the topic is not available ros2 topic list
            # if it is not create a window for the user to choose a correct topic name
            suggested_service_name = suggest_string(console, self.name, "Service", service_name, service_name_list)
            if suggested_service_name is not None:
                service_name = suggested_service_name

            # Check if the service_name is null (suggest_string() failed)
            if not service_name:
                raise ValueError("`command='{}'` requires `service_name`.".format(command))

        # -- ros2 service list ------------------------------------------------
        if command == "list":
            result["output"] = service_name_list_str

        # -- ros2 service info <service_name> ---------------------------------
        elif command == "info":
            info_output = run_oneshot_cmd(["ros2", "service", "info", service_name])
            result["output"] = info_output

        # -- ros2 service type <service_name> ---------------------------------
        elif command == "type":
            type_output = run_oneshot_cmd(["ros2", "service", "type", service_name])
            result["output"] = type_output.strip()

        # -- ros2 service find <type> -----------------------------------------
        elif command == "find":
            find_output = run_oneshot_cmd(["ros2", "service", "find", service_type])
            result["output"] = find_output

        # -- ros2 service call service_name service_type ----------------------
        elif command == "call":
            if call_args is None:
                raise ValueError("`command='call'` requires `args`.")

            # If service_type not given, detect it
            if not service_type:
                type_output = run_oneshot_cmd(["ros2", "service", "type", service_name])
                service_type = type_output.strip()

            call_output = run_oneshot_cmd(["ros2", "service", "call", service_name, service_type, call_args])
            result["output"] = call_output

        # -- ros2 service echo service_name -----------------------------------
        elif command == "echo":
            base_args = ["ros2", "service", "echo", service_name]
            ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines)
            result["output"] = last_output_lines(console, self.name, ret, max_lines=10)

        # -- unknown ------------------------------------------------------------
        else:
            raise ValueError(f"Unknown command '{command}'. Expected one of: list, info, type, call, echo, find.")

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
        ("command", "string"),  # Command
        ("action_name", "string?"),  # (optional) Action name
        ("action_type", "string?"),  # (optional) Action type. "find"
        ("send_goal", "bool?"),  # (optional) legacy flag (backwards compatible)
        ("goal_args", "string?"),  # (optional) goal YAML, e.g. '{order: 5}'
    ]

    output_schema = {
        "output": "string",  # `ros2 action list`
    }

    def run(self, **kwargs):
        # Used in the suggestion string
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        action_name = kwargs.get("action_name", None)
        action_type = kwargs.get("action_type", None)
        goal_args = kwargs.get("goal_args", None)

        result = {
            "output": "",
        }

        command = command.lower()

        action_name_list_str = run_oneshot_cmd(["ros2", "action", "list"])
        action_name_list = action_name_list_str.splitlines()

        # -- Action name suggestions --
        if command != "list":
            # Check if the topic is not available ros2 topic list
            suggested_action_name = suggest_string(console, self.name, "Action", action_name, action_name_list)
            if suggested_action_name is not None:
                action_name = suggested_action_name

            # Check if the action_name is null (suggest_string() failed)
            if not action_name:
                raise ValueError("`command='{}'` requires `action_name`.".format(command))

        # -- ros2 action list -------------------------------------------------
        if command == "list":
            result["output"] = action_name_list_str

        # -- ros2 action info <action_name> -----------------------------------
        elif command == "info":
            info_output = run_oneshot_cmd(["ros2", "action", "info", action_name])
            result["output"] = info_output

        # -- ros2 action type <type_name ---------------------------------------------------------------
        elif command == "type":
            type_output = run_oneshot_cmd(["ros2", "action", "type", action_name])
            result["output"] = type_output

        # send_goal -----------------------------------------------------------
        elif command == "send_goal":
            # Use explicit type if provided, otherwise detect it
            if not action_type:
                type_output = run_oneshot_cmd(["ros2", "action", "type", action_name])
                action_type = type_output.strip()

            args_list = ["ros2", "action", "send_goal", action_name, action_type]
            if goal_args is not None:
                args_list.extend(goal_args)

            goal_output = run_oneshot_cmd(args_list)
            result["output"] = goal_output

        # -- unknown ------------------------------------------------------------
        else:
            raise ValueError(f"Unknown command '{command}'. Expected one of: list, info, type, send_goal.")

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
        ("command", "string"),  # Command
        ("param_name", "string?"),  # (optional) Parameter name
        ("node_name", "string?"),  # (optional) Target node
        ("set_value", "string?"),  # (optional) value for set
        ("file_path", "string?"),  # (optional) for dump/load YAML file
    ]

    output_schema = {
        "output": "string",
    }

    def run(self, **kwargs):
        # Used in the suggestion string
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        node_name = kwargs.get("node_name", None)
        param_name = kwargs.get("param_name", None)
        set_value = kwargs.get("set_value", None)
        file_path = kwargs.get("file_path", None)

        result = {
            "output": "",
        }

        command = command.lower()

        param_name_list_str = run_oneshot_cmd(["ros2", "topic", "list"])
        node_name_list_str = run_oneshot_cmd(["ros2", "node", "list"])

        param_name_list = param_name_list_str.splitlines()
        node_name_list = node_name_list_str.splitlines()

        # -- Param/Node name suggestions --
        if command != "list":
            # Check if the param_name is not available 'ros2 param list'
            if command not in ["dump", "load"]:
                suggested_param_name = suggest_string(console, self.name, "Param", param_name, param_name_list)
                if suggested_param_name is not None:
                    param_name = suggested_param_name

                # Check if the param_name is null (suggest_string() failed)
                if not param_name:
                    raise ValueError("`command='{}'` requires `param_name`.".format(command))

            # Check if the node_name is not available 'ros2 node list'
            suggested_node_name = suggest_string(console, self.name, "Node", node_name, node_name_list)
            if suggested_node_name is not None:
                node_name = suggested_node_name

            # Check if the node_name is null (suggest_string() failed)
            if not node_name:
                raise ValueError("`command='{}'` requires `node_name`.".format(command))

        # -- ros2 param list` -------------------------------------------------
        if command == "list":
            if node_name:
                result["output"] = node_name_list_str
            else:
                result["output"] = param_name_list_str

        # -- ros2 param get <node> <param> ------------------------------------
        elif command == "get":
            get_output = run_oneshot_cmd(["ros2", "param", "get", node_name, param_name])
            result["output"] = get_output

        # -- ros2 param describe <node> <param> -------------------------------
        elif command == "describe":
            describe_output = run_oneshot_cmd(["ros2", "param", "describe", node_name, param_name])
            result["output"] = describe_output

        # -- ros2 param set <node> <param> <set_value> ------------------------
        elif command == "set":
            if set_value is None:
                raise ValueError("`command='set'` requires `set_value`.")

            set_output = run_oneshot_cmd(["ros2", "param", "set", node_name, param_name, set_value])
            result["output"] = set_output

        # -- ros2 param delete <node> <parm> ----------------------------------
        elif command == "delete":
            delete_output = run_oneshot_cmd(["ros2", "param", "delete", node_name, param_name])
            result["output"] = delete_output

        # -- ros2 param dump <node> [file_path] -------------------------------
        elif command == "dump":
            # Two modes:
            # - If file_path given, write to file with --output-file
            # - Otherwise, capture YAML from stdout
            if file_path:
                dump_output = run_oneshot_cmd(["ros2", "param", "dump", node_name, "--output-file", file_path])
                # CLI usually prints a line like "Saved parameters to file..."
                # so we just expose that.
                result["output"] = dump_output or f"Dumped parameters to {file_path}"
            else:
                dump_output = run_oneshot_cmd(["ros2", "param", "dump", node_name])
                result["output"] = dump_output

        # -- ros2 param load <node> <file_path> -------------------------------
        elif command == "load":
            if not file_path:
                raise ValueError("`command='load'` `file_path`.")

            load_output = run_oneshot_cmd(["ros2", "param", "load", node_name, file_path])
            result["output"] = load_output

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. Expected one of: list, get, describe, set, delete, dump, load."
            )

        return result


@vulcanai_tool
class Ros2PkgTool(AtomicTool):
    name = "ros2_pkg"
    description = "Wrapper for `ros2 pkg` CLI.Run any subcommand: 'list', 'executables'."
    tags = ["ros2", "pkg", "packages", "cli", "introspection"]

    # If package_name is not provided, runs: `ros2 pkg list`
    # If provided, runs: `ros2 pkg executables <package_name>`
    input_schema = [
        ("command", "string"),  # Command
    ]

    output_schema = {
        "output": "string",  # list of packages or list of executables for a package.
    }

    def run(self, **kwargs):
        # Get the package name if provided by the query
        command = kwargs.get("command", None)
        result = {
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
            raise ValueError(f"Unknown command '{command}'. Expected one of: list, executables, prefix, xml")

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
        ("command", "string"),  # Command
        ("interface_name", "string?"),  # (optional) Name of the interface, e.g. "std_msgs/msg/String".
        # If not provided, the command is `ros2 interface list`.
        # Otherwise `ros2 interface show <interface_name>`.
    ]

    output_schema = {
        "output": "string",  # list of interfaces (as list of strings) or full interface definition.
    }

    def run(self, **kwargs):
        # Used in the suggestion string
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        # Get the interface name if provided by the query
        command = kwargs.get("command", None)
        interface_name = kwargs.get("interface_name", None)

        result = {
            "output": "",
        }

        command = command.lower()

        interface_name_list_str = run_oneshot_cmd(["ros2", "interface", "list"])
        interface_name_list = interface_name_list_str.splitlines()

        package_name_list_str = run_oneshot_cmd(["ros2", "interface", "packages"])
        package_name_list = package_name_list_str.splitlines()

        # -- ros2 interface list ----------------------------------------------
        if interface_name is None:
            result["output"] = interface_name_list_str

        # -- ros2 interface packages ------------------------------------------
        elif command == "packages":
            result["output"] = package_name_list_str

        # -- ros2 interface package <interface_name> --------------------------------
        elif command == "package":
            package_name = interface_name
            # Check if the topic is not available ros2 topic list
            # if it is not create a window for the user to choose a correct topic name
            suggested_package_name = suggest_string(console, self.name, "Interface", package_name, package_name_list)
            if suggested_package_name is not None:
                package_name = suggested_package_name

            # Check if the interface_name is null (suggest_string() failed)
            if not interface_name:
                raise ValueError("`command='{}'` requires `interface_name`.".format(command))

            info_output = run_oneshot_cmd(["ros2", "topic", "package", package_name])
            result["output"] = info_output

        # -- ros2 interface show <interface_name> --------------------------------
        elif command == "show":
            # Check if the topic is not available ros2 topic list
            # if it is not create a window for the user to choose a correct topic name
            suggested_interface_name = suggest_string(
                console, self.name, "Interface", interface_name, interface_name_list
            )
            if suggested_interface_name is not None:
                interface_name = suggested_interface_name

            # Check if the interface_name is null (suggest_string() failed)
            if not interface_name:
                raise ValueError("`command='{}'` requires `interface_name`.".format(command))

            info_output = run_oneshot_cmd(["ros2", "topic", "show", interface_name])
            result["output"] = info_output

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
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
        node.get_logger().warn(
            f"Cannot import ROS message type '{type_str}'. " + "Adding default pkg 'std_msgs' instead."
        )
    else:
        pkg, _, msg_name = info_list

    module = importlib.import_module(f"{pkg}.msg")

    return getattr(module, msg_name)


@vulcanai_tool
class Ros2PublishTool(AtomicTool):
    name = "ros_publish"
    description = (
        "Publish one or more messages to a given ROS 2 topic <topic_name>. "
        "Or execute 'ros2 topic pub <topic_name>'. "
        "Supports both simple string messages (for std_msgs/msg/String) and custom message types. "
        "For custom types, pass message_data as a JSON object with field names and values. "
        "By default 10 messages 'Hello from VulcanAI PublishTool!' "
        "with type 'std_msgs/msg/String' in topic '/chatter' "
        "with 0.1 seconds of delay between messages to publish"
        'Example for custom type: msg_type=\'my_pkg/msg/MyMessage\', message_data=\'{"index": 1, "message": "Hello"}\''
    )
    tags = ["ros2", "publish", "message", "std_msgs"]

    input_schema = [
        ("topic", "string"),  # e.g. "/chatter"
        ("message_data", "string?"),  # (optional) payload - string for std_msgs/String or JSON for custom types
        ("msg_type", "string?"),  # (optional) e.g. "std_msgs/msg/String" or "my_pkg/msg/CustomMsg"
        ("max_lines", "int?"),  # (optional) number of messages to publish
        ("max_duration", "int?"),  # (optional) stop after this seconds
        ("period_sec", "float?"),  # (optional) delay between publishes (in seconds)
        ("message", "string?"),  # (deprecated) use message_data instead
    ]

    output_schema = {
        "published": "bool",
        "count": "int",
        "topic": "string",
        "output": "string",
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
        # Ros2 node to create the Publisher and print the log information
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        # Optional console handle to route logs to the subprocess panel.
        console = self.bb.get("console", None)

        result = {
            "published": "False",
            "count": "0",
            "topic": "",
            "output": "",
        }

        panel_enabled = console is not None and hasattr(console, "show_subprocess_panel")
        if panel_enabled:
            console.call_from_thread(console.show_subprocess_panel)
            if hasattr(console, "change_route_logs"):
                console.call_from_thread(console.change_route_logs, True)

        topic_name = kwargs.get("topic", "/chatter")
        # Support both 'message_data' (new) and 'message' (deprecated)
        message_data = kwargs.get("message_data", kwargs.get("message", "Hello from VulcanAI PublishTool!"))
        msg_type_str = kwargs.get("msg_type", "std_msgs/msg/String")

        max_duration = kwargs.get("max_duration", 60)
        if not isinstance(max_duration, int):
            max_duration = 60

        max_lines = kwargs.get("max_lines", 200)
        if not isinstance(max_lines, int):
            max_lines = 200

        period_sec = kwargs.get("period_sec", 0.1)

        qos_depth = 10

        if console is None:
            print("[ERROR] Console not is None")

            return result

        published_msgs = []
        publisher = None
        cancel_token = None

        try:
            if not topic_name:
                console.call_from_thread(console.logger.log_msg, "<gray>[ROS] [ERROR] No topic provided.</gray>")
                return result

            result["topic"] = topic_name

            if max_lines <= 0:
                # No messages to publish
                console.call_from_thread(
                    console.logger.log_msg, "<gray>[ROS] [WARN] max_lines <= 0, nothing to publish.</gray>"
                )
                return result

            MsgType = import_msg_type(msg_type_str, node)
            publisher = node.create_publisher(MsgType, topic_name, qos_depth)
            cancel_token = Future()
            console.set_stream_task(cancel_token)
            console.logger.log_tool("[tool]Publisher created![tool]", tool_name=self.name)

            for _ in range(max_lines):
                if cancel_token.cancelled():
                    console.logger.log_tool("[tool]Ctrl+C received:[/tool] stopping publish...", tool_name=self.name)
                    break

                msg = MsgType()

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
                        console.call_from_thread(
                            console.logger.log_msg,
                            "<gray>[ROS] [ERROR] Failed to parse message_data as JSON for custom type"
                            + f"'{msg_type_str}': {e}</gray>",
                        )
                        return result

                if hasattr(msg, "data"):
                    console.call_from_thread(
                        console.logger.log_msg, f"<gray>[ROS] [INFO] Publishing: '{msg.data}'</gray>"
                    )
                else:
                    console.call_from_thread(
                        console.logger.log_msg,
                        f"<gray>[ROS] [INFO] Publishing custom message to '{topic_name}'</gray>",
                    )
                publisher.publish(msg)
                published_msgs.append(msg.data if hasattr(msg, "data") else str(msg))

                rclpy.spin_once(node, timeout_sec=0.05)

                if period_sec and period_sec > 0.0:
                    time.sleep(period_sec)

        finally:
            console.set_stream_task(None)
            if panel_enabled:
                if hasattr(console, "change_route_logs"):
                    console.call_from_thread(console.change_route_logs, False)
                console.call_from_thread(console.hide_subprocess_panel)
            if publisher is not None:
                try:
                    node.destroy_publisher(publisher)
                except Exception:
                    pass

        result["subscribed"] = "True"
        result["published_msgs"] = published_msgs
        result["count"] = len(published_msgs)
        return result


@vulcanai_tool
class Ros2SubscribeTool(AtomicTool):
    name = "ros_subscribe"
    description = (
        "Subscribe to a topic <topic> or execute 'ros2 topic echo <topic>' "
        "and stop after receiving N messages or max duration."
    )
    tags = ["ros2", "subscribe", "topic", "std_msgs"]

    input_schema = [
        ("topic", "string"),  # topic name
        ("max_lines", "int?"),  # (optional) stop after this number of messages
        ("max_duration", "int?"),  # (optional) stop after this seconds
    ]

    output_schema = {
        "subscribed": "bool",
        "count": "int",
        "topic": "string",
        "output": "string",
    }

    def run(self, **kwargs):
        # Ros2 node to create the Publisher and print the log information
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        # Optional console handle to support Ctrl+C cancellation.
        console = self.bb.get("console", None)

        result = {
            "subscribed": "False",
            "subscribed_msgs": "",
            "count": "0",
            "topic": "",
        }

        topic_name = kwargs.get("topic", None)
        max_duration = kwargs.get("max_duration", 60)
        if not isinstance(max_duration, int):
            max_duration = 60

        max_lines = kwargs.get("max_lines", 200)
        if not isinstance(max_lines, int):
            max_lines = 200

        # "--field data" prints only the data field from each message
        # instead of the full YAML message
        # "--no-arr" do not print array fields of messages
        base_args = ["ros2", "topic", "echo", topic_name, "--field", "data", "--no-arr"]
        ret = execute_subprocess(console, self.name, base_args, max_duration, max_lines)
        result["output"] = last_output_lines(console, self.name, ret, n_lines=10)

        if ret is not None:
            result["subscribed"] = "True"
            result["count"] = len(ret)
            result["topic"] = topic_name

        return result
