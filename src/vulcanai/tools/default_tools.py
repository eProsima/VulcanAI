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


"""
This file contains the default tools given by VulcanAI.
It contains atomic tools used to call ROS2 CLI.
"""

import subprocess
from vulcanai import AtomicTool, CompositeTool, vulcanai_tool
from vulcanai.console.utils import execute_subprocess, run_oneshot_cmd, suggest_string

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


???
 |
 V

Commands:

  ros2 bag        Various rosbag related sub-commands
  ros2 component  Various component related sub-commands
  ros2 daemon     Various daemon related sub-commands
  ros2 doctor     Check ROS setup and other potential issues
  ros2 interface  Show information about ROS interfaces
  ros2 launch     Run a launch file
  ros2 lifecycle  Various lifecycle related sub-commands
  ros2 multicast  Various multicast related sub-commands
  ros2 pkg        Various package related sub-commands
  ros2 run        Run a package specific executable
  ros2 security   Various security related sub-commands
  ros2 wtf        Use `wtf` as alias to `doctor`
"""

color_tool = "#EB921E"


@vulcanai_tool
class Ros2NodeTool(AtomicTool):
    name = "ros2_node"
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
            "output": None
        }

        # -- Run `ros2 node list` ---------------------------------------------
        if node_name == None:
            node_name_list = run_oneshot_cmd(["ros2", "node", "list"])
            node_name_list = node_name_list.splitlines()
            result["output"] = node_name_list

        # -- Run `ros2 node info <node>` --------------------------------------
        else:
            if not node_name:
                raise ValueError("`command='info'` requires `node_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "info", node_name]
            )
            result["output"] = info_output

        return result


@vulcanai_tool
class Ros2TopicTool(AtomicTool):
    name = "ros2_topic"
    description = (
        "Wrapper for `ros2 topic` CLI. Always returns `ros2 topic list` "
        "and can optionally run any subcommand: bw, delay, echo, find, "
        "hz, info, list, pub, type"
    )
    tags = ["ros2", "topics", "cli", "info"]

    # - `command` lets you pick a single subcommand (echo/bw/hz/delay/find/pub/type).
    input_schema = [
        ("command", "string"),       # Command: "list", "info", "type", "find",
                                     #  "pub", "hz", "echo", "bw", "delay"
        ("topic_name", "string?"),   # (optional) Topic name. (info/echo/bw/delay/hz/type/pub)
        ("msg_type", "string?"),     # (optional) Message type (`find` <type>, `pub` <message> <type>)
        ("optional_msg", "string?"), # (optional) Message
                                     #
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
        optional_msg = kwargs.get("optional_msg", "")
        # streaming commands variables
        max_duration = kwargs.get("max_duration", 60.0)
        max_lines = kwargs.get("max_lines", 1000)

        result = {
            "ros2": True,
            "output": None,
        }

        command = command.lower()

        topic_name_list = run_oneshot_cmd(["ros2", "topic", "list"])
        topic_name_list = topic_name_list.splitlines()

        # -- ros2 topic list --------------------------------------------------
        if command == "list":
            result["output"] = topic_name_list

        # -- ros2 topic info <topic_name> -------------------------------------
        elif command == "info":
            if not topic_name:
                raise ValueError("`command='info'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "info", topic_name]
            )
            result["output"] = info_output

        # -- ros2 topic find <type> -------------------------------------------
        elif command == "find":
            if not msg_type:
                raise ValueError("`command='find'` requires `msg_type` (ROS type).")

            """# TODO. get types?
            $ ros2 topic list -t
            /parameter_events [rcl_interfaces/msg/ParameterEvent]
            /rosout [rcl_interfaces/msg/Log]
            /turtle1/cmd_vel [geometry_msgs/msg/Twist]
            /turtle1/color_sensor [turtlesim_msgs/msg/Color]
            /turtle1/pose [turtlesim_msgs/msg/Pose]"""

            find_output = run_oneshot_cmd(
                ["ros2", "topic", "find", msg_type]
            )
            find_topics = [
                line.strip() for line in find_output.splitlines() if line.strip()
            ]
            result["output"] = find_topics

        # -- ros2 topic type <topic_name> -------------------------------------
        elif command == "type":
            if not topic_name:
                raise ValueError("`command='type'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            type_output = run_oneshot_cmd(
                ["ros2", "topic", "type", topic_name]
            )
            result["output"] = type_output

        # -- ros2 topic echo <topic_name> -------------------------------------
        elif command == "echo":
            if not topic_name:
                raise ValueError("`command='echo'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            base_args = ["ros2", "topic", "echo", topic_name]

            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True

        # -- ros2 topic bw <topic_name> ---------------------------------------
        elif command == "bw":
            if not topic_name:
                raise ValueError("`command='echo'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            base_args = ["ros2", "topic", "bw", topic_name]
            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True

        # delay --------------------------------------------------------------
        elif command == "delay":
            if not topic_name:
                raise ValueError("`command='delay'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            base_args = ["ros2", "topic", "delay", topic_name]
            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True

        # -- ros2 topic hz <topic_name> ---------------------------------------
        elif command == "hz":
            if not topic_name:
                raise ValueError("`command='hz'` requires `topic_name`.")

            # Check if the topic is not available ros2 topic list
            suggested_topic = suggest_string(console, self.name, "Topic", topic_name, topic_name_list)
            if suggested_topic != None:
                topic_name = suggested_topic

            base_args = ["ros2", "topic", "hz", topic_name]
            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True

        # -- publisher --------------------------------------------------------
        elif command == "pub":
            # One-shot publish using `-1`
            # ros2 topic pub -1 <topic> <msg_type> "data: <optional_msg>"
            if not topic_name:
                raise ValueError("`command='pub'` requires `topic_name`.")
            if not msg_type:
                raise ValueError("`command='pub'` requires `msg_type`.")

            base_args = ["ros2", "topic", "pub", topic_name, msg_type, f"data: {optional_msg}"]
            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True
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
        "Wrapper for `ros2 service` CLI. Always returns `ros2 service list`, "
        "and can optionally run any subcommand: list, info, type, call, echo, find."
    )
    tags = ["ros2", "services", "cli", "info", "call"]

    # - `command` = "list", "info", "type", "call", "echo", "find"
    input_schema = [
        ("command", "string"),         # Command: "list", "info", "type", "find"
                                       # "call", "echo"
        ("service_name", "string?"),   # (optional) Service name. "info", "type", "call", "echo"
        ("service_type", "string?"),   # (optional) Service type. "find", "call"
        ("call", "bool?"),             # (optional) backwards-compatible call flag
        ("args", "string?"),           # (optional) YAML/JSON-like request data for `call`
        ("max_duration", "number?"),   # (optional) Maximum duration
        ("max_lines", "int?"),         # (optional) Maximum lines
    ]

    output_schema = {
        "ros2": "bool",                 # ros2 flag for pretty printing
        "output": "string",           # `ros2 service list`
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
        # streaming commands variables
        max_duration = kwargs.get("max_duration", 2.0) # default for echo
        max_lines = kwargs.get("max_lines", 50)

        result = {
            "ros2": True,
            "output": None,
        }

        command = command.lower()

        service_name_list = run_oneshot_cmd(["ros2", "service", "list"])
        service_name_list = service_name_list.splitlines()

        # -- ros2 service list ------------------------------------------------
        if command == "list":
            result["output"] = service_name_list + "prueba"

        # -- ros2 service info <service_name> ---------------------------------
        elif command == "info":
            if not service_name:
                raise ValueError("`command='info'` requires `service_name`.")

            # Check if the service is not available ros2 service list
            suggested_service = suggest_string(console, self.name, "Service", service_name, service_name_list)
            if suggested_service != None:
                service_name = suggested_service

            info_output = run_oneshot_cmd(
                ["ros2", "service", "info", service_name]
            )

            result["output"] = info_output

        # -- ros2 service type <service_name> ---------------------------------
        elif command == "type":
            if not service_name:
                raise ValueError("`command='type'` requires `service_name`.")

            # Check if the service is not available ros2 service list
            suggested_service = suggest_string(console, self.name, "Service", service_name, service_name_list)
            if suggested_service != None:
                service_name = suggested_service

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

            # Check if the service is not available ros2 service list
            suggested_service = suggest_string(console, self.name, "Service", service_name, service_name_list)
            if suggested_service != None:
                service_name = suggested_service

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

            # Check if the service is not available ros2 service list
            suggested_service = suggest_string(console, self.name, "Service", service_name, service_name_list)
            if suggested_service != None:
                service_name = suggested_service

            base_args = ["ros2", "service", "echo", service_name]
            execute_subprocess(console, self.name, base_args, max_duration, max_lines)

            result["output"] = True

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
        "Wrapper for `ros2 action` CLI. Always returns `ros2 action list`, "
        "and can optionally run: list, info, type, send_goal."
    )
    tags = ["ros2", "actions", "cli", "info", "goal"]

    # - `command`: "list", "info", "type", "send_goal"
    input_schema = [
        ("command", "string"),        # Command: "list" "info" "type" "send_goal"
        ("action_name", "string?"),   # (optional) Action name
        ("action_type", "string?"),   # (optional) Action type. "find"
        ("send_goal", "bool?"),       # (optional) legacy flag (backwards compatible)
        ("args", "string?"),          # (optional) goal YAML, e.g. '{order: 5}'
        ("wait_for_result", "bool?"), # (optional) if true => add `--result`
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

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        action_name = kwargs.get("action_name", None)
        goal_args = kwargs.get("args", None)
        wait_for_result = kwargs.get("wait_for_result", True)
        action_type = kwargs.get("action_type", None)

        result = {
            "ros2": True,
            "output": None,
        }

        command = command.lower()

        action_name_list = run_oneshot_cmd(["ros2", "action", "list"])
        action_name_list = action_name_list.splitlines()

        # -- ros2 action list -------------------------------------------------
        if command == "list":
            result["output"] = action_name_list

        # -- ros2 action info <action_name> -----------------------------------
        elif command == "info":
            if not action_name:
                raise ValueError("`command='info'` requires `action_name`.")

            # Check if the service is not available ros2 service list
            suggested_action = suggest_string(console, self.name, "Action", action_name, action_name_list)
            if suggested_action != None:
                action_name = suggested_action

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
            if goal_args is None:
                raise ValueError("`command='send_goal'` requires `args`.")

            # Check if the service is not available ros2 service list
            suggested_action = suggest_string(console, self.name, "Action", action_name, action_name_list)
            if suggested_action != None:
                action_name = suggested_action

            # Use explicit type if provided, otherwise detect it
            if not action_type:
                type_output = run_oneshot_cmd(
                    ["ros2", "action", "type", action_name]
                )
                action_type = type_output.strip()

            args_list = ["ros2", "action", "send_goal"]
            if wait_for_result:
                args_list.append("--result")
            args_list.extend([action_name, action_type, goal_args])

            goal_output = run_oneshot_cmd(args_list)
            """result["goal_output"] = goal_output
            result["type"] = action_type"""
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
        "Wrapper for `ros2 param` CLI. Always returns `ros2 param list` "
        "(optionally filtered by node), and can run: list, get, describe, "
        "set, delete, dump, load."
    )
    tags = ["ros2", "param", "parameters", "cli"]

    # - `command`: "list", "get", "describe", "set", "delete", "dump", "load"
    input_schema = [
        ("command", "string"),     # Command: "list" "get" "describe"
                                   # "set" "delete" "dump" "load"
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

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        command = kwargs.get("command", None)
        node = kwargs.get("node_name", None)
        param = kwargs.get("param_name", None)
        set_value = kwargs.get("set_value", None)
        file_path = kwargs.get("file_path", None)

        result = {
            "ros2": True,
            "output": None,
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
    description = "List ROS2 packages and optionally get executables for a specific package."
    tags = ["ros2", "pkg", "packages", "cli", "introspection"]

    # If package_name is not provided, runs: `ros2 pkg list`
    # If provided, runs: `ros2 pkg executables <package_name>`
    input_schema = [
        ("package_name", "string?")  # (optional) Name of the package.
                                     # If not provided, the command is `ros2 pkg list`.
                                     # Otherwise `ros2 pkg executables <package_name>`.
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

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        # Get the package name if provided by the query
        package_name = kwargs.get("package_name", None)

        result = {
            "ros2": True,
            "output": None
        }

        command = command.lower()

        # -- Run `ros2 pkg list` --------------------------------------------
        if command == "list":
            pkg_name_list = run_oneshot_cmd(["ros2", "pkg", "list"])
            pkg_name_list = pkg_name_list.splitlines()
            result["output"] = pkg_name_list

        # -- Run `ros2 pkg executables` --------------------------------------------
        elif command == "executables":
            pkg_name_list = run_oneshot_cmd(["ros2", "pkg", "executables"])
            pkg_name_list = pkg_name_list.splitlines()
            result["output"] = pkg_name_list

        # -- Run `ros2 pkg executables <package>` ---------------------------
        elif command == "prefix":
            if not package_name:
                raise ValueError("`command='prefix'` requires `package_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "prefix", package_name]
            )
            result["output"] = info_output

        # -- Run `ros2 pkg executables <package>` ---------------------------
        elif command == "xml":
            if not package_name:
                raise ValueError("`command='xml'` requires `package_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "xml", package_name]
            )
            result["output"] = info_output

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
            )

        return result


@vulcanai_tool
class Ros2InterfaceTool(AtomicTool):
    name = "ros2_interface"
    description = "List ROS2 interfaces and optionally show the definition of a specific interface."
    tags = ["ros2", "interface", "msg", "srv", "action", "cli", "introspection"]

    # - `command` lets you pick a single subcommand (list/packages/package).
    input_schema = [
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

        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")

        # Get the interface name if provided by the query
        interface_name = kwargs.get("interface_name", None)

        result = {
            "ros2": True,
            "output": None
        }

        command = command.lower()

        pkg_name_list = run_oneshot_cmd(["ros2", "pkg", "list"])
        pkg_name_list = pkg_name_list.splitlines()

        # -- ros2 interface list ----------------------------------------------
        if interface_name is None:
            interface_name_list = run_oneshot_cmd(["ros2", "interface", "list"])
            interface_name_list = interface_name_list.splitlines()
            result["output"] = interface_name_list

        # -- ros2 interface packages ------------------------------------------
        elif command == "inpackagesfo":
            interface_pkg_name_list = run_oneshot_cmd(["ros2", "interface", "packages"])
            interface_pkg_name_list = interface_pkg_name_list.splitlines()
            result["output"] = interface_pkg_name_list

        # -- ros2 interface package <pkg_name> --------------------------------
        elif command == "package":
            if not interface_name:
                raise ValueError("`command='package'` requires `interface_name`.")

            info_output = run_oneshot_cmd(
                ["ros2", "topic", "package", interface_name]
            )
            result["output"] = info_output

        # TODO. proto, show?

        # -- unknown ----------------------------------------------------------
        else:
            raise ValueError(
                f"Unknown command '{command}'. "
                "Expected one of: list, info, echo, bw, delay, hz, find, pub, type."
            )

        return result