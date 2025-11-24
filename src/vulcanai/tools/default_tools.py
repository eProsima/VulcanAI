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

import time

import rclpy

from vulcanai import AtomicTool, CompositeTool, vulcanai_tool

import subprocess

"""
TODO

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



@vulcanai_tool
class Ros2NodeTool(AtomicTool):
    name = "ros2_node"
    description = "List ROS2 nodes and optionally get detailed info for a specific node."
    tags = ["ros2", "nodes", "cli", "info", "diagnostics"]
    input_schema = [
        ("node_name", "string?") # optional
    ]
    output_schema = {
        "nodes": "array",
        "info": "string?"
    }

    def run(self, **kwargs):
        node_name = kwargs.get("node_name", None)

        # -- Run `ros2 node list` ---------------------------------------------
        try:
            list_output = subprocess.check_output(
                ["ros2", "node", "list"],
                stderr=subprocess.STDOUT,
                text=True
            )
            node_list = [line.strip() for line in list_output.splitlines()]
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run 'ros2 node list': {e.output}")

        result = {
            "nodes": node_list,
            "info": None
        }

        # -- Run `ros2 node info <node>` --------------------------------------
        if node_name:
            try:
                info_output = subprocess.check_output(
                    ["ros2", "node", "info", node_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                result["info"] = info_output
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to get info for node '{node_name}': {e.output}")

        return result


@vulcanai_tool
class Ros2TopicTool(AtomicTool):
    name = "ros2_topic"
    description = "List ROS2 topics or get info for a specific topic."
    tags = ["ros2", "topics", "cli", "info"]
    input_schema = [
        ("topic_name", "string?"),     # optional input
        ("echo", "bool?")              # optional: run 'ros2 topic echo'
    ]
    output_schema = {
        "topics": "array",             # topic names
        "info": "string?",             # output of `ros2 topic info`
        "echo_output": "string?"       # output of `ros2 topic echo` (one-shot)
    }

    def run(self, **kwargs):
        topic_name = kwargs.get("topic_name", None)
        do_echo = kwargs.get("echo", False)

        # -- Run `ros2 topic list` --------------------------------------------
        try:
            list_output = subprocess.check_output(
                ["ros2", "topic", "list"],
                stderr=subprocess.STDOUT,
                text=True
            )
            topic_list = [line.strip() for line in list_output.splitlines()]
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run 'ros2 topic list': {e.output}")

        result = {
            "topics": topic_list,
            "info": None,
            "echo_output": None
        }

        # -- Run `ros2 topic info <topic>` ------------------------------------
        if topic_name:
            try:
                info_output = subprocess.check_output(
                    ["ros2", "topic", "info", topic_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                result["info"] = info_output
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 topic info {topic_name}': {e.output}")

        return result


@vulcanai_tool
class Ros2ServiceTool(AtomicTool):
    name = "ros2_service"
    description = "List ROS2 services, get service info, type, and optionally call a service."
    tags = ["ros2", "services", "cli", "info", "call"]
    input_schema = [
        ("service_name", "string?"),     # optional service name
        ("call", "bool?"),               # optional: perform service call
        ("args", "string?")              # JSON-like request data for service
    ]
    output_schema = {
        "services": "array",             # output of `ros2 service list`
        "info": "string?",               # output of `ros2 service info`
        "type": "string?",               # output of `ros2 service type`
        "call_output": "string?"         # output of `ros2 service call` (optional)
    }

    def run(self, **kwargs):
        service_name = kwargs.get("service_name", None)
        do_call = kwargs.get("call", False)
        call_args = kwargs.get("args", None)

        # -- Run `ros2 service list` ------------------------------------------
        try:
            list_output = subprocess.check_output(
                ["ros2", "service", "list"],
                stderr=subprocess.STDOUT,
                text=True
            )
            service_list = [line.strip() for line in list_output.splitlines()]
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run 'ros2 service list': {e.output}")

        result = {
            "services": service_list,
            "info": None,
            "type": None,
            "call_output": None
        }

        # -- Run `ros2 service info <service>` --------------------------------
        if service_name:
            try:
                info_output = subprocess.check_output(
                    ["ros2", "service", "info", service_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                result["info"] = info_output
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 service info {service_name}': {e.output}")

        # -- Run `ros2 service type <service>` --------------------------------
        if service_name:
            try:
                type_output = subprocess.check_output(
                    ["ros2", "service", "type", service_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                service_type = type_output.strip()
                result["type"] = service_type
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 service type {service_name}': {e.output}")

        return result


@vulcanai_tool
class Ros2ActionTool(AtomicTool):
    name = "ros2_action"
    description = "List ROS2 actions, get action info/type, and optionally send a goal."
    tags = ["ros2", "actions", "cli", "info", "goal"]
    input_schema = [
        ("action_name", "string?"),  # optional: target action
        ("send_goal", "bool?"),      # optional: whether to send a goal
        ("args", "string?"),         # goal data, e.g. '{order: 5}'
        ("wait_for_result", "bool?") # if true, use --result
    ]
    output_schema = {
        "actions": "array",          # output of `ros2 action list`
        "info": "string?",           # output of `ros2 action info`
        "type": "string?",           # output of `ros2 action type`
        "goal_output": "string?"     # output of `ros2 action send_goal`
    }

    def run(self, **kwargs):
        action_name = kwargs.get("action_name", None)
        do_send_goal = kwargs.get("send_goal", False)
        goal_args = kwargs.get("args", None)
        wait_for_result = kwargs.get("wait_for_result", True)

        # -- Run `ros2 action list` -------------------------------------------
        try:
            list_output = subprocess.check_output(
                ["ros2", "action", "list"],
                stderr=subprocess.STDOUT,
                text=True
            )
            action_list = [line.strip() for line in list_output.splitlines()]
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run 'ros2 action list': {e.output}")

        result = {
            "actions": action_list,
            "info": None,
            "type": None,
            "goal_output": None
        }

        # -- Run `ros2 action info <action>` ----------------------------------
        if action_name:
            try:
                info_output = subprocess.check_output(
                    ["ros2", "action", "info", action_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                result["info"] = info_output
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 action info {action_name}': {e.output}")

        # -- Run `ros2 action type <action>` ----------------------------------
        if action_name:
            try:
                type_output = subprocess.check_output(
                    ["ros2", "action", "type", action_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )
                action_type = type_output.strip()
                result["type"] = action_type
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 action type {action_name}': {e.output}")

        return result


@vulcanai_tool
class Ros2ParamTool(AtomicTool):
    name = "ros2_param"
    description = "List ROS2 parameters, get/describe a parameter, or set a parameter."
    tags = ["ros2", "param", "parameters", "cli"]
    input_schema = [
        ("node_name", "string?"),        # optional: target node
        ("param_name", "string?"),       # optional: specific parameter
        ("set_value", "string?")         # optional: set a parameter to a value
    ]
    output_schema = {
        "param_list": "string?",         # `ros2 param list` output
        "get_output": "string?",          # `ros2 param get`
        "describe_output": "string?",     # `ros2 param describe`
        "set_output": "string?"           # `ros2 param set`
    }

    def run(self, **kwargs):
        node = kwargs.get("node_name", None)
        param = kwargs.get("param_name", None)
        set_value = kwargs.get("set_value", None)

        result = {
            "param_list": None,
            "get_output": None,
            "describe_output": None,
            "set_output": None,
        }

        # -- Run `ros2 param list (GLOBAL or per-node)` -----------------------
        try:
            if node:
                list_cmd = ["ros2", "param", "list", node]
            else:
                list_cmd = ["ros2", "param", "list"]

            list_output = subprocess.check_output(
                list_cmd, stderr=subprocess.STDOUT, text=True
            )
            result["param_list"] = list_output
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run 'ros2 param list': {e.output}")

        return result

