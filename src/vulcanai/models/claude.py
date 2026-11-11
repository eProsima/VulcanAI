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
import concurrent.futures
import mimetypes
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, AsyncIterator, Dict, Iterable, Optional, Type, TypeVar

import claude_agent_sdk

from vulcanai.core.plan_types import AIValidation, GlobalPlan, GoalSpec
from vulcanai.models.model import IModel, IModelHooks

# Generic type variable for response classes
T = TypeVar("T", GlobalPlan, GoalSpec, AIValidation)


class ClaudeModel(IModel):
    """
    Wrapper for Claude models through the Claude Agent SDK.

    The SDK runs the Claude Code CLI bundled with the package, which authenticates with the
    Claude account the user is logged into (shared with Claude Code if it is installed).
    Log in once with `vulcanai-claude-login`. If the ANTHROPIC_API_KEY environment variable
    is set, the API key is used instead.
    """

    def __init__(self, model_name: Optional[str], logger=None, hooks: Optional[IModelHooks] = None):
        super().__init__()
        self.logger = logger
        # None uses the default model of the logged-in account
        self.model_name = model_name
        self.hooks = hooks
        self._load_model()

    def _load_model(self):
        """
        Initialize the SDK and check at startup that the CLI and authentication are available.
        Model access and credential validity are checked on the first query. Only logs, never raises.
        """
        self.model = claude_agent_sdk
        if self.logger is None:
            return
        try:
            cli = _claude_cli()
            if cli is None:
                self.logger.log_manager(
                    "Claude Code CLI not found. Install VulcanAI: python3 -m pip install .", error=True
                )
                self.model = None
                return
            status = subprocess.run([cli, "auth", "status"], capture_output=True, text=True, timeout=5)
            if status.returncode == 1:
                self.logger.log_manager(
                    "Claude is not logged in. Run [manager]vulcanai-claude-login[/manager] in a terminal,"
                    " or set the ANTHROPIC_API_KEY environment variable.",
                    error=True,
                )
            elif status.returncode != 0:
                self.logger.log_manager("Could not check Claude authentication status.", error=True)
        except Exception as e:
            self.logger.log_manager(f"Could not check Claude model '{self.model_name}': {e}", error=True)

    def _inference(
        self,
        *,
        system_prompt: str,
        user_prompt: str,
        response_cls: Type[T],
        images: Optional[Iterable[str]] = None,
        history: Optional[list[tuple[str, str]]] = None,
    ) -> Optional[T]:
        """
        Generic inference template function that parameterizes the response class.

        :param system_prompt: System message.
        :param user_prompt: User message.
        :param response_cls: Target structured output type (e.g., GlobalPlan, GoalSpec, AIValidation).
        :param images: Optional image paths; included as base64 image content blocks.
        :param history: Optional (user_text, plan_summary) tuples to reconstruct conversational context.
        :return: Parsed response object of type T, or None on error.
        """
        if self.model is None:
            self.logger.log_manager("Claude Agent SDK is not available.", error=True)
            return None

        start = time.time()

        # Build user content (text + optional images)
        user_content = self._build_user_content(user_prompt, images)

        # Build messages (optional history + current user)
        messages = self._build_messages(user_content, history)

        # Notify hooks of request start
        try:
            self.hooks.on_request_start()
        except Exception:
            pass

        # Call Claude with the output format bound to the desired schema/class
        try:
            result = self._run_sync(self._query(system_prompt, messages, response_cls))
        except Exception as e:
            self.logger.log_manager(f"Claude Agent SDK: {e}", error=True)
            if isinstance(e, claude_agent_sdk.ResultError):
                self.logger.log_manager(
                    f"Claude result error ({e.subtype}): result={e.result}, errors={e.errors}", error=True
                )
            if "not logged in" in str(e).lower():
                self.logger.log_manager(
                    "Log in to your Claude account by running [manager]vulcanai-claude-login[/manager] in a terminal,"
                    " or set the ANTHROPIC_API_KEY environment variable.",
                    error=True,
                )
            return None
        finally:
            # Notify hooks of request end
            try:
                self.hooks.on_request_end()
            except Exception:
                pass

        if result is None or result.is_error or result.structured_output is None:
            reason = getattr(result, "subtype", "no result received")
            self.logger.log_manager(f"Claude did not return a {response_cls.__name__}: {reason}", error=True)
            if result is not None:
                self.logger.log_manager(f"Claude result: {result.result}, errors={result.errors}", error=True)
            return None

        # Extract parsed object safely
        parsed: Optional[T] = None
        try:
            parsed = response_cls.model_validate(result.structured_output)
        except Exception as e:
            self.logger.log_manager(f"Failed to parse response into {response_cls.__name__}: {e}", error=True)

        end = time.time()
        self.logger.log_manager(f"Claude response time: [manager]{end - start:.3f} seconds[/manager]")
        try:
            input_tokens = (
                result.usage.get("input_tokens", 0)
                + result.usage.get("cache_creation_input_tokens", 0)
                + result.usage.get("cache_read_input_tokens", 0)
            )
            output_tokens = result.usage.get("output_tokens", 0)
            self.logger.log_manager(
                f"Prompt tokens: [manager]{input_tokens}[/manager], "
                + f"Completion tokens: [manager]{output_tokens}[/manager]"
            )
        except Exception:
            pass

        return parsed

    async def _query(self, system_prompt: str, messages: list[Dict[str, Any]], response_cls: Type[T]):
        """Run a single stateless query and return its ResultMessage."""
        sdk = self.model
        options = sdk.ClaudeAgentOptions(
            system_prompt=system_prompt,
            model=self.model_name,
            output_format={"type": "json_schema", "schema": response_cls.model_json_schema()},
            # Plain planner: no built-in tools (Bash, Read, ...), and ignore the user's
            # Claude Code settings, CLAUDE.md files and MCP servers
            tools=[],
            setting_sources=[],
            strict_mcp_config=True,
            # Capture stderr so CLI cleanup cannot re-enable the terminal's native cursor.
            stderr=lambda line: None,
            # Structured output needs one extra turn to return the result
            max_turns=3,
            # Do not save each query as a session transcript in ~/.claude/projects
            extra_args={"no-session-persistence": None},
        )

        async def prompt() -> AsyncIterator[Dict[str, Any]]:
            for message in messages:
                yield {
                    "type": "user",
                    "message": message,
                    "parent_tool_use_id": None,
                    "session_id": "",
                }

        result = None
        async for message in sdk.query(prompt=prompt(), options=options):
            if isinstance(message, sdk.ResultMessage):
                result = message
        return result

    @staticmethod
    def _run_sync(coro):
        """Run a coroutine from sync code, also when the calling thread already runs an event loop."""
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.run(coro)
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            return pool.submit(asyncio.run, coro).result()

    def _build_user_content(self, user_text: str, images: Optional[Iterable[str]]) -> list[Dict[str, Any]]:
        """Compose user content blocks with text first and optional images."""
        content: list[Dict[str, Any]] = [{"type": "text", "text": user_text}]
        if images:
            for image_path in images:
                if isinstance(image_path, str) and image_path.startswith("http"):
                    content.append({"type": "image", "source": {"type": "url", "url": image_path}})
                else:
                    try:
                        base64_image = self._encode_image(image_path)
                        mime = mimetypes.guess_type(image_path)[0] or "image/png"
                        content.append(
                            {"type": "image", "source": {"type": "base64", "media_type": mime, "data": base64_image}}
                        )
                    except Exception as e:
                        # Fail soft on a single bad image but continue with others
                        self.logger.log_manager(
                            f"Fail soft. Image '{image_path}' could not be encoded: {e}", error=True
                        )
        return content

    def _build_messages(
        self,
        user_content: list[Dict[str, Any]],
        history: Optional[list[tuple[str, str]]],
    ) -> list[Dict[str, Any]]:
        """Construct a single user message containing optional history and current user content."""
        # Each query is stateless, so replay the short history as context in the prompt
        if history:
            previous = "\n".join(f"- {request}\n  Action plan: {summary}" for request, summary in history)
            text = f"Previous interactions:\n{previous}\n\n{user_content[0]['text']}"
            user_content = [{"type": "text", "text": text}, *user_content[1:]]
        return [{"role": "user", "content": user_content}]


def _claude_cli() -> Optional[str]:
    """Path to the Claude Code CLI: the one bundled with the Agent SDK, or an installed `claude`."""
    bundled = Path(claude_agent_sdk.__file__).parent / "_bundled" / "claude"
    if bundled.exists():
        return str(bundled)
    return shutil.which("claude")


def login() -> None:
    """
    Entry point of `vulcanai-claude-login`: sign in to the Claude account used by the Claude models.

    `vulcanai-claude-login` logs in, `vulcanai-claude-login status` shows the current login and
    `vulcanai-claude-login logout` logs out.
    """
    cli = _claude_cli()
    if cli is None:
        sys.exit("Claude Code CLI not found. Install VulcanAI: python3 -m pip install .")
    action = sys.argv[1] if len(sys.argv) > 1 else "login"
    if action not in ("login", "status", "logout"):
        sys.exit("Usage: vulcanai-claude-login [login|status|logout]")
    os.execv(cli, [cli, "auth", action, *sys.argv[2:]])
