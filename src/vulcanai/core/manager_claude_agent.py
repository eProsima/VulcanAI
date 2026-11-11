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

import asyncio
import json
import mimetypes
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import warnings
from pathlib import Path
from typing import Any, Dict, Optional

from vulcanai.console.logger import VulcanAILogger
from vulcanai.core.manager import ToolManager
from vulcanai.core.plan_types import ArgValue, GlobalPlan, PlanNode, Step
from vulcanai.core.validator import PlanValidator
from vulcanai.tools.tool_registry import ToolRegistry

MCP_SERVER_NAME = "vulcanai"
# Built-in agent tool to open image files, as Claude Code does with its Read tool
VIEW_IMAGE_TOOL = "view_image"
# Built-in agent tool to run many tool calls in one turn (a whole GlobalPlan)
EXECUTE_PLAN_TOOL = "execute_plan"
IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg", ".gif", ".webp")
# Claude rejects larger images
MAX_IMAGE_BYTES = 5 * 1024 * 1024
# Claude Code tools to write and run code (e.g. analyze an image with numpy/PIL, compute coordinates)
CODE_READ_TOOLS = ("Read", "Glob", "Grep")
CODE_WRITE_TOOLS = ("Write", "Edit")

# VulcanAI schema types -> JSON schema types
JSON_TYPES = {
    "int": "integer",
    "integer": "integer",
    "float": "number",
    "number": "number",
    "bool": "boolean",
    "boolean": "boolean",
    "str": "string",
    "string": "string",
}


class ClaudeAgentManager(ToolManager):
    """
    Agent mode: Claude runs the loop itself, as in Claude Code.

    Instead of generating a whole plan in one inference, Claude gets every VulcanAI tool as a
    native tool. It thinks, calls a tool, sees the result, and decides the next step until the
    request is done. The session is kept between requests, so Claude remembers the conversation.

    Model names: "claude-agent" (account default model), "claude-agent-opus",
    "claude-agent-sonnet", "claude-agent-haiku" or "claude-agent-<full model id>".
    """

    def __init__(
        self,
        model: str = "claude-agent",
        registry: Optional[ToolRegistry] = None,
        validator: Optional[PlanValidator] = None,
        k: int = 10,
        hist_depth: int = 3,
        logger: Optional[VulcanAILogger] = None,
        default_tools: bool = True,
        max_turns: int = 200,
        code_tools: bool = True,
    ):
        self.claude_model = self._parse_model_name(model)
        # The ClaudeModel of the base Agent is only used for the console hooks (spinner)
        # and to build image content blocks
        if not self.claude_model:
            base_model = "claude"
        elif self.claude_model.startswith("claude"):
            base_model = self.claude_model
        else:
            base_model = "claude-" + self.claude_model
        super().__init__(
            base_model,
            registry,
            validator,
            k,
            hist_depth,
            logger,
            default_tools,
        )
        self.agent_validator = PlanValidator(self.registry)
        self.max_turns = max_turns
        # Claude Code tools (Bash, Read, Write...) to write and run code, as in Claude Code.
        # Bash runs sandboxed and files can only be written in this scratch folder
        self.code_tools = code_tools and self._sandbox_available()
        self.scratch_dir = Path(tempfile.mkdtemp(prefix="vulcanai_claude_agent_")) if self.code_tools else None

        # The SDK client is async: it lives in its own event loop thread, and the sync
        # VulcanAI API (handle_user_request) submits coroutines to it
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, name="claude_agent_loop", daemon=True).start()
        self._client = None
        self._client_tools: tuple = ()
        self._tool_lock = threading.Lock()

        self.logger.log_manager(
            "Claude agent mode: Claude runs the loop and calls the tools itself, with model "
            + f"[manager]{self.claude_model or 'account default'}[/manager]"
        )

    def _sandbox_available(self) -> bool:
        """
        Check that the Bash sandbox (bubblewrap) can run here. On Ubuntu 23.10+ AppArmor blocks the
        user namespaces bubblewrap needs for programs without a profile that allows them.
        """
        reason = ""
        # Fail closed: code tools are only offered where Claude Code can sandbox Bash and it was verified
        if sys.platform == "darwin":
            if shutil.which("sandbox-exec") is None:
                reason = "sandbox-exec is not available"
        elif not sys.platform.startswith("linux"):
            reason = f"Bash sandboxing is not supported on {sys.platform}"
        elif shutil.which("bwrap") is None:
            reason = "bubblewrap is not installed (sudo apt install bubblewrap socat)"
        else:
            try:
                command = ["bwrap", "--unshare-net", "--ro-bind", "/", "/", "true"]
                check = subprocess.run(command, capture_output=True, text=True, timeout=10)
                if check.returncode != 0:
                    reason = f"bubblewrap cannot create its sandbox: {check.stderr.strip()}"
                    if os.path.exists("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"):
                        # AppArmor profile shipped in the VulcanAI repository (resources/apparmor/bwrap)
                        profile = Path(__file__).resolve().parents[3] / "resources" / "apparmor" / "bwrap"
                        source = str(profile) if profile.exists() else "<VulcanAI repo>/resources/apparmor/bwrap"
                        reason += (
                            ". AppArmor restricts unprivileged user namespaces. Allow them for bubblewrap with:"
                            f" sudo cp {source} /etc/apparmor.d/bwrap && sudo apparmor_parser -r /etc/apparmor.d/bwrap"
                        )
            except Exception as e:
                reason = f"bubblewrap check failed: {e}"
        if reason:
            self.logger.log_manager(
                f"Claude code tools disabled (Bash, files): the sandbox is not available, {reason}.", error=True
            )
        return not reason

    @staticmethod
    def _parse_model_name(model: str) -> Optional[str]:
        name = model[len("claude-agent") :].lstrip("-")
        if not name:
            return None
        if name.lower() in ("opus", "sonnet", "haiku"):
            return name.lower()
        return name

    # region Requests

    def handle_user_request(self, user_text: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send the request to Claude and let it work until it is done.

        :param user_text: The user request in natural language.
        :param context: Additional context. 'images' is a list of image paths to include.
        :return: A dictionary with the success status, Claude's final answer and the blackboard.
        """
        images = (context or {}).get("images", [])
        try:
            result = asyncio.run_coroutine_threadsafe(self._run_request(user_text, images), self._loop).result()
        except Exception as e:
            self.logger.log_manager(f"Claude agent: {e}", error=True)
            if "not logged in" in str(e).lower():
                self.logger.log_manager(
                    "Log in to your Claude account by running [manager]vulcanai-claude-login[/manager] in a terminal,"
                    " or set the ANTHROPIC_API_KEY environment variable.",
                    error=True,
                )
            return {"error": str(e)}

        success = result is not None and not result.is_error
        answer = getattr(result, "result", None) if result is not None else None
        if answer:
            self._add_to_history(user_text, answer)
        return {"plan": None, "success": success, "answer": answer, "blackboard": self.bb}

    def reset_session(self):
        """Forget the conversation: the next request starts a new Claude session."""
        asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop).result()
        self.history = []
        self.logger.log_manager("Claude agent session reset.")

    async def _run_request(self, user_text: str, images: list):
        sdk = self._sdk()
        client = await self._ensure_client()

        user_content = self.llm.model._build_user_content(user_text, images, None)

        async def message():
            yield {
                "type": "user",
                "message": {"role": "user", "content": user_content},
                "parent_tool_use_id": None,
                "session_id": "default",
            }

        self._hook("on_request_start", "Claude is thinking...")
        result = None
        try:
            await client.query(message())
            async for msg in client.receive_response():
                if isinstance(msg, sdk.AssistantMessage):
                    for block in msg.content:
                        self._log_block(sdk, block)
                elif isinstance(msg, sdk.ResultMessage):
                    result = msg
                    self._log_result(msg)
        finally:
            self._hook("on_request_end")
        return result

    # endregion

    # region Claude session

    @staticmethod
    def _sdk():
        import claude_agent_sdk

        return claude_agent_sdk

    async def _ensure_client(self):
        """Connect the Claude session, or reconnect it if the set of active tools changed."""
        tools = tuple(sorted(name for name in self.registry.tools if name != "help"))
        if self._client is not None and tools == self._client_tools:
            return self._client
        if self._client is not None:
            self.logger.log_manager("Active tools changed: starting a new Claude session.")
            await self._disconnect()

        sdk = self._sdk()
        sdk_tools = [self._make_sdk_tool(sdk, self.registry.tools[name]) for name in tools]
        sdk_tools.append(self._make_view_image_tool(sdk))
        sdk_tools.append(self._make_execute_plan_tool(sdk))
        server = sdk.create_sdk_mcp_server(name=MCP_SERVER_NAME, tools=sdk_tools)
        allowed = [f"mcp__{MCP_SERVER_NAME}__{name}" for name in (*tools, VIEW_IMAGE_TOOL, EXECUTE_PLAN_TOOL)]
        code_options = {}
        if self.code_tools:
            code_options = {
                "cwd": str(self.scratch_dir),
                # Bash only runs inside the sandbox (bubblewrap): it can read files, but only write
                # in the scratch folder, and has no network
                "sandbox": {"enabled": True, "autoAllowBashIfSandboxed": True, "allowUnsandboxedCommands": False},
                # Write/Edit are not auto-approved: _can_use_tool only allows them inside the scratch folder
                "can_use_tool": self._can_use_tool,
            }
            allowed += list(CODE_READ_TOOLS)
        # The VulcanAI and read-only tools are auto-approved on purpose: can_use_tool only gates Write/Edit
        warnings.filterwarnings("ignore", category=sdk.CanUseToolShadowedWarning)
        options = sdk.ClaudeAgentOptions(
            system_prompt=self._system_prompt(),
            model=self.claude_model,
            mcp_servers={MCP_SERVER_NAME: server},
            # The VulcanAI tools (auto-approved) and, if enabled, the Claude Code tools to work with code.
            # Ignore the user's Claude Code settings, CLAUDE.md files and MCP servers
            tools=[*CODE_READ_TOOLS, *CODE_WRITE_TOOLS, "Bash"] if self.code_tools else [],
            allowed_tools=allowed,
            setting_sources=[],
            strict_mcp_config=True,
            **code_options,
            # Reason between steps, as in Claude Code, and show the reasoning summaries
            thinking={"type": "adaptive", "display": "summarized"},
            max_turns=self.max_turns,
        )
        self._client = sdk.ClaudeSDKClient(options=options)
        await self._client.connect()
        self._client_tools = tools
        self.logger.log_manager(f"Claude session started with [manager]{len(tools)}[/manager] tools.")
        return self._client

    async def _disconnect(self):
        if self._client is not None:
            try:
                await self._client.disconnect()
            except Exception:
                pass
            self._client = None
            self._client_tools = ()

    def _system_prompt(self) -> str:
        return (
            "You are an agent controlling a robotic system through VulcanAI tools (ROS 2).\n"
            "Work step by step: think, call the tools you need, check their results (for example read "
            "poses or sensors) before continuing, and correct course if something did not work as "
            "expected. If a tool fails, read the error and try to fix the problem.\n"
            "Inputs not marked as required may be omitted: do not invent values for them.\n"
            "Do not assume default values of the environment (colors, positions, parameters...): "
            "check them with the tools when they matter for the result.\n"
            f"For long or repetitive sequences of tool calls (drawing shapes or text, following paths...), "
            f"batch many steps in a single '{EXECUTE_PLAN_TOOL}' call instead of calling the tools one by one. "
            "You can split a long job in several plans and check the result between them.\n"
            f"If the user mentions an image file, open it with the '{VIEW_IMAGE_TOOL}' tool.\n"
            "When the request is done, reply with a short summary of what you did."
            f"{self._code_tools_prompt()}"
            f"{self._parse_user_context()}"
        )

    def _code_tools_prompt(self) -> str:
        if not self.code_tools:
            return ""
        return (
            "\n\n## Working with code\n"
            "You can also write and run code with the Bash, Read, Write and Edit tools, as in Claude Code. "
            "Use it whenever accuracy matters instead of estimating by eye: for example to analyze an image "
            "pixel by pixel (shapes, coordinates, colors), to compute geometry or long lists of coordinates, "
            "to generate large plans as JSON, or to render a preview of a drawing and compare it with the target "
            "before running it on the robot.\n"
            f"- Your working (scratch) folder is {self.scratch_dir}. Files can only be written there.\n"
            f"- Run Python with {sys.executable} (numpy and PIL are available).\n"
            "- Bash runs in a sandbox without network. Control the robot only with the VulcanAI tools "
            f"(load generated plans and send them with '{EXECUTE_PLAN_TOOL}'), not with ros2 commands."
        )

    async def _can_use_tool(self, tool_name: str, tool_input: Dict[str, Any], context):
        """Permission callback for the tools that are not auto-approved (Write, Edit, some Bash commands)."""
        sdk = self._sdk()
        if tool_name == "Bash":
            # Sandboxed commands are auto-approved, but Claude Code asks for the ones it cannot analyze
            # (e.g. a variable as the command name). They still run inside the sandbox, as unsandboxed
            # commands are not allowed: only refuse explicit requests to leave it
            if tool_input.get("dangerouslyDisableSandbox"):
                return sdk.PermissionResultDeny(message="Bash commands must run inside the sandbox")
            return sdk.PermissionResultAllow()
        if tool_name in CODE_WRITE_TOOLS:
            path = Path(str(tool_input.get("file_path", ""))).expanduser().resolve()
            if path.is_relative_to(self.scratch_dir.resolve()):
                return sdk.PermissionResultAllow()
            self.logger.log_manager(f"Denied {tool_name} outside the scratch folder: {path}", error=True)
            return sdk.PermissionResultDeny(message=f"Files can only be written inside {self.scratch_dir}")
        return sdk.PermissionResultDeny(message=f"Tool '{tool_name}' is not allowed")

    # endregion

    # region Tools

    def _make_sdk_tool(self, sdk, tool):
        """Wrap a VulcanAI tool as an in-process MCP tool."""
        properties, required = {}, []
        defaults = getattr(tool, "input_defaults", {}) or {}
        for key, type_name in tool.input_schema:
            base, optional = self._schema_type_parts(type_name)
            prop: Dict[str, Any] = {}
            if base in JSON_TYPES:
                prop["type"] = JSON_TYPES[base]
            else:
                prop["description"] = f"Type: {base}"
            if key in defaults:
                prop["default"] = defaults[key]
            properties[key] = prop
            if not optional:
                required.append(key)
        schema = {"type": "object", "properties": properties, "required": required}

        description = tool.description
        if tool.output_schema:
            description += f"\nOutputs: {self._format_tool_outputs(tool)}"

        async def handler(args: Dict[str, Any]) -> Dict[str, Any]:
            # Tools block (ROS calls, sleeps): run them outside the event loop
            return await asyncio.to_thread(self._run_tool, tool.name, args)

        return sdk.tool(tool.name, description, schema)(handler)

    def _make_view_image_tool(self, sdk):
        """Tool to let Claude open image files mentioned by the user."""
        schema = {
            "type": "object",
            "properties": {"path": {"type": "string", "description": "Absolute path of the image file"}},
            "required": ["path"],
        }
        description = (
            "Open an image file (png, jpg, jpeg, gif, webp) from disk to look at it. "
            "Use it when the user gives the path of an image."
        )

        async def handler(args: Dict[str, Any]) -> Dict[str, Any]:
            return await asyncio.to_thread(self._view_image, str(args.get("path", "")))

        return sdk.tool(VIEW_IMAGE_TOOL, description, schema)(handler)

    def _make_execute_plan_tool(self, sdk):
        """Tool to let Claude run a whole GlobalPlan (many tool calls) in one turn."""
        plan_schema = GlobalPlan.model_json_schema()
        defs = plan_schema.pop("$defs", {})
        schema = {
            "type": "object",
            "properties": {"plan": {"$ref": "#/$defs/GlobalPlan"}},
            "required": ["plan"],
            "$defs": {**defs, "GlobalPlan": plan_schema},
        }
        description = (
            "Run a VulcanAI plan: several tool calls in one turn. 'plan' is a list of PlanNodes run in order; "
            "each PlanNode has kind 'SEQUENCE' or 'PARALLEL' and a list of steps, and each step names a tool "
            "(plain name, e.g. 'move_turtle') and its args as [{'key': ..., 'val': ...}] "
            "(numbers and booleans as JSON values, not strings). "
            "The output of a previous step can be used as '{{bb.<tool_name>.<output_key>}}'. "
            "The plan is validated before running, and execution stops at the first failed step. "
            "Returns whether it succeeded and the last output of each tool."
        )

        async def handler(args: Dict[str, Any]) -> Dict[str, Any]:
            return await asyncio.to_thread(self._execute_plan, args.get("plan"))

        return sdk.tool(EXECUTE_PLAN_TOOL, description, schema)(handler)

    def _execute_plan(self, raw_plan) -> Dict[str, Any]:
        self._hook("on_request_end")
        try:
            with self._tool_lock:
                try:
                    if isinstance(raw_plan, str):
                        raw_plan = json.loads(raw_plan)
                    plan = GlobalPlan.model_validate(raw_plan)
                    # Claude sees the tools as 'mcp__vulcanai__<name>': accept that form too
                    prefix = f"mcp__{MCP_SERVER_NAME}__"
                    for node in plan.plan:
                        for step in [*node.steps, *(node.on_fail.steps if node.on_fail else [])]:
                            step.tool = step.tool.removeprefix(prefix)
                    self.agent_validator.validate(plan)
                except Exception as e:
                    self.logger.log_validator(f"Plan validation: {e}", error=True)
                    return self._tool_result(f"Invalid plan: {e}", is_error=True)

                self.logger.log_manager(f"Plan received from Claude:\n{plan}")
                ok = self.executor.run(plan, self.bb).get("success", False)
                error = self.executor.last_error
                results = {k: v for k, v in self.bb.items() if k not in ("main_node", "console")}
        finally:
            self._flush_console()
            self._hook("on_request_start", "Claude is thinking...")

        text = json.dumps({"success": ok, "results": results}, default=str)
        if not ok:
            return self._tool_result(f"Plan failed: {error or 'a step did not succeed'}. {text}", is_error=True)
        return self._tool_result(text)

    def _view_image(self, path: str) -> Dict[str, Any]:
        path = os.path.abspath(os.path.expanduser(path.strip().strip("\"'")))
        self.logger.log_msg(f"<bold>[manager][CLAUDE][/manager]</bold> Opening image {path}")
        if not os.path.isfile(path):
            folder = os.path.dirname(path)
            hint = ""
            if os.path.isdir(folder):
                images = sorted(f for f in os.listdir(folder) if f.lower().endswith(IMAGE_EXTENSIONS))
                hint = f" Images in {folder}: {', '.join(images) if images else 'none'}."
            else:
                hint = f" The folder {folder} does not exist either."
            return self._tool_result(f"File not found: {path}.{hint}", is_error=True)
        if not path.lower().endswith(IMAGE_EXTENSIONS):
            return self._tool_result(f"Not a supported image file: {path}", is_error=True)
        if os.path.getsize(path) > MAX_IMAGE_BYTES:
            return self._tool_result(f"Image too large (max {MAX_IMAGE_BYTES // (1024 * 1024)} MB): {path}", True)
        return self._tool_result(f"Image {path}", images=[path])

    def _run_tool(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Validate and execute a tool call with the VulcanAI executor, and build the MCP result."""
        self._hook("on_request_end")
        try:
            with self._tool_lock:
                step = Step(tool=name, args=[ArgValue(key=k, val=v) for k, v in args.items() if v is not None])
                plan = GlobalPlan(plan=[PlanNode(kind="SEQUENCE", steps=[step])])
                try:
                    self.agent_validator.validate(plan)
                except Exception as e:
                    self.logger.log_validator(f"Tool call validation: {e}", error=True)
                    return self._tool_result(f"Invalid call: {e}", is_error=True)

                ok = self.executor.run(plan, self.bb).get("success", False)
                output = self.bb.get(name)
                error = self.executor.last_error
        finally:
            self._flush_console()
            self._hook("on_request_start", "Claude is thinking...")

        if not ok:
            return self._tool_result(f"Tool '{name}' failed: {error or 'unknown error'}", is_error=True)
        return self._tool_result(json.dumps(output, default=str), images=self._output_images(output))

    @staticmethod
    def _output_images(output) -> list:
        """Image paths returned by a tool under the 'images' key (validation tools)."""
        if isinstance(output, dict) and isinstance(output.get("images"), list):
            return [p for p in output["images"] if isinstance(p, str)]
        return []

    def _tool_result(self, text: str, is_error: bool = False, images: Optional[list] = None) -> Dict[str, Any]:
        content = [{"type": "text", "text": text}]
        for path in images or []:
            try:
                data = self.llm.model._encode_image(path)
                content.append(
                    {"type": "image", "data": data, "mimeType": mimetypes.guess_type(path)[0] or "image/png"}
                )
            except Exception as e:
                self.logger.log_manager(f"Fail soft. Image '{path}' could not be encoded: {e}", error=True)
        result: Dict[str, Any] = {"content": content}
        if is_error:
            result["is_error"] = True
        return result

    # endregion

    # region Console output

    def _log_block(self, sdk, block):
        prefix = "<bold>[manager][CLAUDE][/manager]</bold> "
        if isinstance(block, sdk.ThinkingBlock):
            if block.thinking.strip():
                self.logger.log_msg(f"{prefix}<italic>Thinking:</italic>\n<dim>{block.thinking.strip()}</dim>")
        elif isinstance(block, sdk.TextBlock):
            if block.text.strip():
                self.logger.log_msg(f"{prefix}{block.text.strip()}")
        elif isinstance(block, sdk.ToolUseBlock):
            name = block.name.split("__")[-1]
            if name == EXECUTE_PLAN_TOOL:
                plan = block.input.get("plan") or {}
                nodes = plan.get("plan", []) if isinstance(plan, dict) else []
                steps = sum(len(node.get("steps", [])) for node in nodes if isinstance(node, dict))
                self.logger.log_msg(f"{prefix}Calling [executor]{name}[/executor] with {steps} steps")
                return
            if name == "Bash":
                lines = str(block.input.get("description") or block.input.get("command", "")).splitlines()
                summary = lines[0][:150] if lines else ""
                self.logger.log_msg(f"{prefix}Running code: [executor]{summary}[/executor]")
                return
            if name in (*CODE_READ_TOOLS, *CODE_WRITE_TOOLS):
                target = block.input.get("file_path") or block.input.get("pattern") or ""
                self.logger.log_msg(f"{prefix}{name} [executor]{target}[/executor]")
                return
            args = ", ".join(f"{k}={v}" for k, v in block.input.items())
            self.logger.log_msg(f"{prefix}Calling [executor]{name}[/executor]({args})")

    def _log_result(self, result):
        usage = result.usage or {}
        input_tokens = (
            usage.get("input_tokens", 0)
            + usage.get("cache_creation_input_tokens", 0)
            + usage.get("cache_read_input_tokens", 0)
        )
        status = "[error]failed[/error]" if result.is_error else "[executor]done[/executor]"
        self.logger.log_manager(
            f"Claude agent {status} ({result.subtype}) in [manager]{result.duration_ms / 1000:.1f} s[/manager], "
            f"{result.num_turns} turns. Prompt tokens: [manager]{input_tokens}[/manager], "
            f"Completion tokens: [manager]{usage.get('output_tokens', 0)}[/manager]"
        )

    def _hook(self, name: str, *args):
        """Notify the console spinner, if any."""
        try:
            getattr(self.llm.model.hooks, name)(*args)
        except Exception:
            pass

    def _flush_console(self):
        """Write the executor logs the console held back while a tool stream panel is open."""
        console = self.bb.get("console", None)
        if console is not None and hasattr(console, "write_deferred_main_output"):
            try:
                console.call_from_thread(console.write_deferred_main_output)
            except Exception:
                pass

    # endregion
