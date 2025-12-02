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

import asyncio
import os
from typing import List, Optional

import threading

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

import os
import time
import subprocess
from typing import List, Optional


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
            try:

                # is one-shot, the of the subprocess is automatic
                # (when it prints the nodes).
                list_output = subprocess.check_output(
                    ["ros2", "node", "list"],
                    stderr=subprocess.STDOUT,
                    text=True
                )

                # add the output of the command to the dictionary
                result["output"] = [line.strip() for line in list_output.splitlines()]
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to run 'ros2 node list': {e.output}")

        # -- Run `ros2 node info <node>` --------------------------------------
        else:

            try:
                # COMMAND: ros2 node info <node>
                # is one-shot, the of the subprocess is automatic
                # (when it prints the infomation of the node).
                info_output = subprocess.check_output(
                    ["ros2", "node", "info", node_name],
                    stderr=subprocess.STDOUT,
                    text=True
                )

                # add the ouptut of the comand to the dictionary
                result["output"] = info_output
            except subprocess.CalledProcessError as e:
                raise Exception(f"Failed to get info for node '{node_name}': {e.output}")

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
                                     #
        ("max_duration", "number?"), # (optional) Seconds for streaming commands (echo/bw/hz/delay)
        ("max_lines", "int?"),       # (optional) Cap number of lines for streaming commands
    ]

    output_schema = {
        "ros2": "bool",            # ros2 flag for pretty printing
        "output": "string",        # output
    }

    # region utils

    def run_oneshot_cmd(self, args: List[str]) -> str:
        try:
            return subprocess.check_output(
                args,
                stderr=subprocess.STDOUT,
                text=True
            )

        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")
    #1
    def run_streaming_cmd(self, node, base_args: List[str],
            max_duration: float,
            max_lines: Optional[int] = None,
            echo: bool = True) -> str:
        """
        Run `ros2 topic ...` for at most `max_duration` seconds.

        - Streams output line by line (optionally echoing to this process' stdout).
        - Optionally stops after `max_lines` lines.
        - Returns all collected output as a single string.
        TODO. if maxduration=-1 or max_lines=-1 STREAMING COMMAND until signal received
        """

        # Help ROS2 (Python) flush output promptly even when not attached to a TTY
        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")

        proc = subprocess.Popen(
            base_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        lines = []
        first = True
        start = time.monotonic()

        try:
            assert proc.stdout is not None

            for raw_line in proc.stdout:
                line = raw_line.rstrip("\n")
                lines.append(line)

                if echo:
                    # TODO. danip
                    if first:
                        first = False
                        print("\n")
                    print(line)

                # Check max_lines
                if len(lines) >= max_lines:
                    break
                # Check duration
                if (time.monotonic() - start) >= max_duration:
                    break
        finally:

            # Ensure the subprocess is terminated
            proc.kill()
            # Wait for killed subprocess
            try:
                proc.wait(timeout=1)
            except Exception:
                pass

        # Spin
        rclpy.spin_once(node, timeout_sec=0.1)

        # Return as a string.
        return "\n".join(lines)

    #2-3
    async def run_streaming_cmd_async(
        self, node, console,
        base_args: List[str],
        max_duration: float,
        max_lines: Optional[int] = None,
        echo: bool = True
    ) -> str:
        

        console._log("\n turtlesim_tool. run() -> launcher -> run_streaming_cmd_async")
        console._log(threading.current_thread())
        console._log(threading.current_thread().name)

        """from vulcanai.console.utils import StreamToTextual

        # Disable terminal input
        sys.stdout = StreamToTextual(console, "stdout")
        sys.stderr = StreamToTextual(console, "stderr")"""

        process = await asyncio.create_subprocess_exec(
            "ros2", "topic", "echo", "/turtle1/pose",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        assert process.stdout is not None

        with open("/home/danny/eProsima/ARISE/workspace_console_2/output.txt", "w", encoding="utf-8") as f:

            try:
                console._log("\n turtlesim_tool. run() -> launcher -> run_streaming_cmd_async -> process")
                console._log(threading.current_thread())
                console._log(threading.current_thread().name)
                # Read line by line
                async for raw_line in process.stdout:
                    line = raw_line.decode(errors="ignore").rstrip("\n")
                    #node.get_logger().info(line)
                    #print(line)
                    #console._log(line)
                    f.write(line + "\n")

            except asyncio.CancelledError:
                # Task was cancelled → stop the subprocess
                #node.get_logger().info("Cancellation received: terminating subprocess...")
                #print("Cancellation received: terminating subprocess...")
                console._log("Cancellation received: terminating subprocess...")
                process.terminate()
                raise

            except KeyboardInterrupt:
                # Ctrl+C pressed → stop subprocess
                #node.get_logger().info("Ctrl+C received: terminating subprocess...")
                #print("Ctrl+C received: terminating subprocess...")
                console._log("Ctrl+C received: terminating subprocess...")
                process.terminate()

            finally:
                try:
                    await asyncio.wait_for(process.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    #node.get_logger().warn("Subprocess didn't exit in time → killing it.")
                    #print("Subprocess didn't exit in time → killing it.")
                    console._log("Subprocess didn't exit in time → killing it.")
                    process.kill()
                    await process.wait()

        return "Process stopped due to Ctrl+C"




        """# Help ROS2 (Python) flush output promptly even when not attached to a TTY
        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")

        proc = subprocess.Popen(
            base_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        lines = []
        first = True
        start = time.monotonic()

        try:
            assert proc.stdout is not None

            for raw_line in proc.stdout:
                line = raw_line.rstrip("\n")
                lines.append(line)

                if echo:
                    # TODO. danip
                    if first:
                        first = False
                        print("\n")
                    print(line)

                # Check max_lines
                if len(lines) >= max_lines:
                    break
                # Check duration
                if (time.monotonic() - start) >= max_duration:
                    break
        finally:

            # Ensure the subprocess is terminated
            proc.kill()
            # Wait for killed subprocess
            try:
                proc.wait(timeout=1)
            except Exception:
                pass

        # Spin
        rclpy.spin_once(node, timeout_sec=0.1)

        # Return as a string.
        return "\n".join(lines)"""

    # endregion

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        
        console = self.bb.get("console", None)
        if console is None:
            raise Exception("Could not find console, aborting...")
        

        console._log("\n turtlesim_tool. run()")
        console._log(threading.current_thread())
        console._log(threading.current_thread().name)

        command = kwargs.get("command", None)  # optional explicit subcommand
        topic_name = kwargs.get("topic_name", None)
        msg_type = kwargs.get("msg_type", None)
        # streaming commands variables
        max_duration = kwargs.get("max_duration", 2.0)
        max_lines = kwargs.get("max_lines", 50)

        result = {
            "ros2": True,
            "output": None,
        }

        command = command.lower()

        # -- ros2 topic list --------------------------------------------------
        if command == "list":
            list_output = self.run_oneshot_cmd(["ros2", "topic", "list"])
            result["output"] = [line.strip() for line in list_output.splitlines() \
                            if line.strip()]

        # -- ros2 topic info <topic_name> -------------------------------------
        elif command == "info":
            if not topic_name:
                raise ValueError("`command='info'` requires `topic_name`.")

            info_output = self.run_oneshot_cmd(
                ["ros2", "topic", "info", topic_name]
            )
            result["output"] = info_output

        # -- ros2 topic find <type> -------------------------------------------
        elif command == "find":
            # ``
            if not msg_type:
                raise ValueError("`command='find'` requires `msg_type` (ROS type).")
            find_output = self.run_oneshot_cmd(
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

            type_output = self.run_oneshot_cmd(
                ["ros2", "topic", "type", topic_name]
            )
            result["output"] = type_output

        # streaming commands TODO. danip
        # -- ros2 topic echo <topic_name> -------------------------------------
        elif command == "echo":
            if not topic_name:
                raise ValueError("`command='echo'` requires `topic_name`.")

            # streaming commands TODO. danip

            """if console.loop == None:
                # No loop -> top-level / CLI case
                node.get_logger().info("EMPTY\n\n\n\n\n\n\n\n")
                stream_task = asyncio.run(self.run_streaming_cmd_async(node, console, ["ros2", "topic", "echo", topic_name],
                                                                max_duration=max_duration,
                                                                max_lines=max_lines))
            else:
                echo_output = console.loop.create_task(self.run_streaming_cmd_async(node, console, ["ros2", "topic", "echo", topic_name],
                                                                max_duration=max_duration,
                                                                max_lines=max_lines))

                def _on_done(task: asyncio.Task):
                    try:
                        task.result()
                    except Exception as e:
                        # Use console so you see the error in the Textual terminal
                        #self._log(f"[red]Echo task error:[/red] {e!r}")
                        node.get_logger().info("ERROR")

                echo_output.add_done_callback(_on_done)"""
            
            """
            try:
                # Are we already inside an event loop? (e.g. Textual)
                loop = asyncio.get_running_loop()
            except RuntimeError:
                # No loop -> top-level / CLI case
                stream_task = asyncio.run(self.run_streaming_cmd_async(node, console, ["ros2", "topic", "echo", topic_name],
                                                                max_duration=max_duration,
                                                                max_lines=max_lines))
            else:
                # Inside an event loop -> schedule as a task
                # You probably don't care about the return value here,
                # you just want streaming side effects.
                echo_output = loop.create_task(self.run_streaming_cmd_async([node, console, "ros2", "topic", "echo", topic_name],
                                                                max_duration=max_duration,
                                                                max_lines=max_lines))
            """

            base_args = ["ros2", "topic", "echo", topic_name]
            def launcher() -> None:

                console._log("\n turtlesim_tool. run() -> launcher")
                console._log(threading.current_thread())
                console._log(threading.current_thread().name)

                # This always runs in the Textual event-loop thread
                loop = asyncio.get_running_loop()
                stream_task = loop.create_task(
                    self.run_streaming_cmd_async(
                        node,
                        console,
                        base_args,
                        max_duration=0.0,
                        max_lines=None,
                    )
                )

                # function used to track this launcher function
                def _on_done(task: asyncio.Task) -> None:
                    try:
                        task.result()
                    except Exception as e:
                        # IMPORTANT: don't call node.get_logger().info here,
                        # because your hook uses call_from_thread (see below)
                        #console.write(f"Echo task error: {e!r}\n")
                        console._log(f"Echo task error: {e!r}\n")

                stream_task.add_done_callback(_on_done)

                #return stream_task

            try:
                # Are we already in the Textual event loop thread?
                asyncio.get_running_loop()
            except RuntimeError:
                # No loop here → probably ROS thread. Bounce into Textual thread.
                # `console.app` is your Textual App instance.
                console.set_stream_task("a")
                console.app.call_from_thread(launcher)
                console.set_stream_task("b")
            else:
                # We *are* in the loop → just launch directly.
                console.set_stream_task("c")
                launcher()
                console.set_stream_task("d")

            
            console.set_stream_task("e")

            

            #result["output"] = echo_output
            #result["output"] = echo_output != None

        # streaming commands TODO. danip
        # -- ros2 topic bw <topic_name> ---------------------------------------
        elif command == "bw":

            subprocess.run(
                ["ros2", "topic", "bw", "/turtle1/pose"],
                check=True,
            )
            """if not topic_name:
                raise ValueError("`command='bw'` requires `topic_name`.")
            bw_output = self.run_streaming_cmd(node,
                ["ros2", "topic", "bw", topic_name],
                max_duration=10,
                max_lines=10,
            )"""
            #result["bw_output"] = bw_output
            result["output"] = True

        # streaming commands TODO. danip
        # delay --------------------------------------------------------------
        elif command == "delay":
            if not topic_name:
                raise ValueError("`command='delay'` requires `topic_name`.")

            delay_output = self.run_streaming_cmd(node,
                ["ros2", "topic", "delay", topic_name],
                max_duration=max_duration,
                max_lines=max_lines,
            )
            result["output"] = delay_output

        # streaming commands TODO. danip
        # -- ros2 topic hz <topic_name> ---------------------------------------
        elif command == "hz":
            if not topic_name:
                raise ValueError("`command='hz'` requires `topic_name`.")

            hz_output = self.run_streaming_cmd(node,
                ["ros2", "topic", "hz", topic_name],
                max_duration=max_duration,
                max_lines=max_lines,
            )
            result["output"] = hz_output

        # streaming commands TODO. danip
        # -- publisher --------------------------------------------------------
        elif command == "pub":
            # One-shot publish using `-1`
            # ros2 topic pub -1 <topic> <msg_type> "<data>"
            # ros2 topic pub -1 /rosout2 std_msgs/msg/String "{data: 'Hello'}"
            if not topic_name:
                raise ValueError("`command='pub'` requires `topic_name`.")
            if not msg_type:
                raise ValueError("`command='pub'` requires `msg_type`.")

            # only send 1
            pub_output = self.run_oneshot_cmd(
                ["ros2", "topic", "pub", "-1", topic_name, msg_type]
            )
            result["output"] = pub_output
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

    # region utils

    def run_oneshot_cmd(self, args: List[str]) -> str:
        try:
            return subprocess.check_output(
                args,
                stderr=subprocess.STDOUT,
                text=True
            )
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")

    def run_streaming_cmd(
        self,
        base_args: List[str],
        max_duration: float,
        max_lines: Optional[int] = None,
    ) -> str:
        """
        Run a streaming `ros2 service` command (currently used for `echo`) for at most
        `max_duration` seconds and optionally stop after `max_lines` lines.
        """
        proc = subprocess.Popen(
            base_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,  # line-buffered
        )

        lines: List[str] = []
        try:
            import time
            start = time.time()

            while True:
                if max_duration is not None and (time.time() - start) > max_duration:
                    break

                line = proc.stdout.readline()
                if not line:
                    break  # process ended

                lines.append(line.rstrip("\n"))
                if max_lines is not None and len(lines) >= max_lines:
                    break

            if proc.poll() is None:
                proc.kill()
                proc.wait()
        finally:
            if proc.stdout and not proc.stdout.closed:
                proc.stdout.close()

        return "\n".join(lines)

    # endregion

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        command = kwargs.get("command", None)
        service_name = kwargs.get("service_name", None)
        service_type = kwargs.get("service_type", None)
        do_call = kwargs.get("call", False)            # legacy flag
        call_args = kwargs.get("args", None)
        # streaming commands variables
        max_duration = kwargs.get("max_duration", 2.0) # default for echo
        max_lines = kwargs.get("max_lines", 50)

        result = {
            "ros2": True,
            "output": None,
        }

        # Backwards-compatible mode: no `command` specified
        # TODO. danip
        """if command is None:
            # Original behavior:
            # - if service_name: info + type
            # - if call=True: call service using inferred type
            if service_name:
                # info
                info_output = self.run_oneshot_cmd(
                    ["ros2", "service", "info", service_name]
                )
                result["info"] = info_output

                # type
                type_output = self.run_oneshot_cmd(
                    ["ros2", "service", "type", service_name]
                )
                detected_type = type_output.strip()
                result["type"] = detected_type

                # optional call
                if do_call:
                    if call_args is None:
                        raise ValueError(
                            "Backward-compatible `call=True` requires `args`."
                        )
                    call_output = self.run_oneshot_cmd(
                        ["ros2", "service", "call", service_name, detected_type, call_args]
                    )
                    result["call_output"] = call_output

            return result"""

        command = command.lower()

        # -- ros2 service list ------------------------------------------------
        if command == "list":
            list_output = self.run_oneshot_cmd(["ros2", "service", "list"])
            result["output"] = list_output

        # -- ros2 service info <service_name> ---------------------------------
        elif command == "info":
            if not service_name:
                raise ValueError("`command='info'` requires `service_name`.")

            info_output = self.run_oneshot_cmd(
                ["ros2", "service", "info", service_name]
            )

            result["output"] = info_output

        # -- ros2 service type <service_name> ---------------------------------
        elif command == "type":
            if not service_name:
                raise ValueError("`command='type'` requires `service_name`.")

            type_output = self.run_oneshot_cmd(
                ["ros2", "service", "type", service_name]
            )

            result["output"] = type_output.strip()

        # -- ros2 service find <type> -----------------------------------------
        elif command == "find":
            if not service_type:
                raise ValueError("`command='find'` requires `service_type`.")

            find_output = self.run_oneshot_cmd(
                ["ros2", "service", "find", service_type]
            )

            result["output"] = find_output

        # call ---------------------------------------------------------------
        elif command == "call":
            if not service_name:
                raise ValueError("`command='call'` requires `service_name`.")
            if call_args is None:
                raise ValueError("`command='call'` requires `args`.")

            # If service_type not given, detect it
            if not service_type:
                type_output = self.run_oneshot_cmd(
                    ["ros2", "service", "type", service_name]
                )
                service_type = type_output.strip()

            call_output = self.run_oneshot_cmd(
                ["ros2", "service", "call", service_name, service_type, call_args]
            )

            result["output"] = call_output

        # streaming commands TODO. danip
        # echo ---------------------------------------------------------------
        elif command == "echo":
            if not service_name:
                raise ValueError("`command='echo'` requires `service_name`.")

            echo_output = self.run_streaming_cmd(
                ["ros2", "service", "echo", service_name],
                max_duration=max_duration,
                max_lines=max_lines,
            )
            result["echo_output"] = echo_output

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

    # regin utils

    def run_oneshot_cmd(self, args: List[str]) -> str:
        try:
            return subprocess.check_output(
                args,
                stderr=subprocess.STDOUT,
                text=True
            )
        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")

    # endregion

    def run(self, **kwargs):
        # Get the shared ROS2 node from the blackboard
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        
        command = kwargs.get("command", None)
        action_name = kwargs.get("action_name", None)
        do_send_goal = kwargs.get("send_goal", False)          # legacy flag
        goal_args = kwargs.get("args", None)
        wait_for_result = kwargs.get("wait_for_result", True)
        action_type = kwargs.get("action_type", None)

        result = {
            "ros2": True,
            "output": None,
        }

        # Backwards-compatible mode: no `command` specified
        # TODO. danip
        """if command is None:
            # Original behavior:
            # - if action_name: info + type
            # - if send_goal=True: send goal (using detected type) and optionally wait for result
            if action_name:
                # info
                info_output = self.run_oneshot_cmd(
                    ["ros2", "action", "info", action_name]
                )
                result["info"] = info_output

                # type
                type_output = self.run_oneshot_cmd(
                    ["ros2", "action", "type", action_name]
                )
                detected_type = type_output.strip()
                result["type"] = detected_type
                action_type = detected_type  # reuse below if send_goal=True

                # optional goal
                if do_send_goal:
                    if goal_args is None:
                        raise ValueError(
                            "Backward-compatible `send_goal=True` requires `args`."
                        )

                    args_list = ["ros2", "action", "send_goal"]
                    if wait_for_result:
                        args_list.append("--result")
                    args_list.extend([action_name, action_type, goal_args])

                    goal_output = self.run_oneshot_cmd(args_list)
                    result["goal_output"] = goal_output

            return result"""

        command = command.lower()

        # -- ros2 action list -------------------------------------------------
        if command == "list":
            list_output = self.run_oneshot_cmd(["ros2", "action", "list"])
            result["output"] = list_output

        # -- ros2 action info <action_name> -----------------------------------
        elif command == "info":
            if not action_name:
                raise ValueError("`command='info'` requires `action_name`.")

            info_output = self.run_oneshot_cmd(
                ["ros2", "action", "info", action_name]
            )
            result["output"] = info_output

        # -- ros2 action type <type_name ---------------------------------------------------------------
        elif command == "type":
            if not action_name:
                raise ValueError("`command='type'` requires `action_name`.")
            type_output = self.run_oneshot_cmd(
                ["ros2", "action", "type", action_name]
            )

            result["output"] = type_output

        # send_goal -----------------------------------------------------------
        elif command == "send_goal":
            if not action_name:
                raise ValueError("`command='send_goal'` requires `action_name`.")
            if goal_args is None:
                raise ValueError("`command='send_goal'` requires `args`.")

            # Use explicit type if provided, otherwise detect it
            if not action_type:
                type_output = self.run_oneshot_cmd(
                    ["ros2", "action", "type", action_name]
                )
                action_type = type_output.strip()

            args_list = ["ros2", "action", "send_goal"]
            if wait_for_result:
                args_list.append("--result")
            args_list.extend([action_name, action_type, goal_args])

            goal_output = self.run_oneshot_cmd(args_list)
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

    # region utils
    def run_oneshot_cmd(self, args: List[str]) -> str:
        try:
            return subprocess.check_output(
                args,
                stderr=subprocess.STDOUT,
                text=True
            )

        except subprocess.CalledProcessError as e:
            raise Exception(f"Failed to run '{' '.join(args)}': {e.output}")

    # endregion

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
            "output": None,
        }

        # Backwards-compatible mode: no `command` specified
        # TODO. danip
        """if command is None:
            # Old behavior was just param list; we extend it slightly:
            # - If node + param => get + describe
            # - If set_value provided => set
            if node and param:
                # get
                get_output = self.run_oneshot_cmd(
                    ["ros2", "param", "get", node, param]
                )
                result["get_output"] = get_output

                # describe
                describe_output = self.run_oneshot_cmd(
                    ["ros2", "param", "describe", node, param]
                )
                result["describe_output"] = describe_output

                # optional set
                if set_value is not None:
                    set_output = self.run_oneshot_cmd(
                        ["ros2", "param", "set", node, param, set_value]
                    )
                    result["set_output"] = set_output

            return result"""

        command = command.lower()

        # -- ros2 param list` -------------------------------------------------
        if command == "list":
            try:
                if node:
                    list_cmd = ["ros2", "param", "list", node]
                else:
                    list_cmd = ["ros2", "param", "list"]

                list_output = self.run_oneshot_cmd(list_cmd)
                result["output"] = list_output

            except Exception as e:
                raise Exception(str(e))

        # -- ros2 param get <node> <param> ------------------------------------
        elif command == "get":
            if not node or not param:
                raise ValueError("`command='get'` requires `node_name` and `param_name`.")

            get_output = self.run_oneshot_cmd(
                ["ros2", "param", "get", node, param]
            )

            result["output"] = get_output

        # -- ros2 param describe <node> <param> -------------------------------
        elif command == "describe":
            if not node or not param:
                raise ValueError("`command='describe'` requires `node_name` and `param_name`.")

            describe_output = self.run_oneshot_cmd(
                ["ros2", "param", "describe", node, param]
            )

            result["output"] = describe_output

        # -- ros2 param set <node> <param> <set_value> ------------------------
        elif command == "set":
            if not node or not param:
                raise ValueError("`command='set'` requires `node_name` and `param_name`.")
            if set_value is None:
                raise ValueError("`command='set'` requires `set_value`.")

            set_output = self.run_oneshot_cmd(
                ["ros2", "param", "set", node, param, set_value]
            )

            result["output"] = set_output

        # -- ros2 param delete <node> <parm> ----------------------------------
        elif command == "delete":
            if not node or not param:
                raise ValueError("`command='delete'` requires `node_name` and `param_name`.")

            delete_output = self.run_oneshot_cmd(
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
                dump_output = self.run_oneshot_cmd(
                    ["ros2", "param", "dump", node, "--output-file", file_path]
                )

                # CLI usually prints a line like "Saved parameters to file..."
                # so we just expose that.
                result["dump_output"] = dump_output or f"Dumped parameters to {file_path}"
            else:
                dump_output = self.run_oneshot_cmd(
                    ["ros2", "param", "dump", node]
                )

                result["dump_output"] = dump_output

        # -- ros2 param load <node> <file_path> -------------------------------
        elif command == "load":
            if not node or not file_path:
                raise ValueError("`command='load'` requires `node_name` and `file_path`.")

            load_output = self.run_oneshot_cmd(
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



