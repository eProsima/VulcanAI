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

from typing import Any, Dict, Optional, Tuple

from vulcanai.executor import PlanExecutor
from vulcanai.tool_registry import ToolRegistry
from vulcanai.llm_agent import LLMAgent
from vulcanai.logger import VulcanAILogger


class ToolManager:
    """Manages the LLM Agent and calls the executor with the LLM output."""
    def __init__(self, model: str, registry: Optional[ToolRegistry]=None, k: int=10, logger=None):
        self.logger = logger or VulcanAILogger().log_manager
        self.llm = LLMAgent(model, self.logger)
        self.k = k
        self.registry = registry or ToolRegistry(logger=(logger or VulcanAILogger().log_registry))
        # self.validator = PlanValidator(registry)
        self.executor = PlanExecutor(self.registry, logger=(logger or VulcanAILogger().log_executor))

    def register_tool(self, tool):
        """Wrapper for registering a single tool."""
        self.registry.register(tool)

    def register_tools_from_file(self, path: str):
        """Wrapper for discovering tools from a file."""
        self.registry.discover_tools_from_file(path)

    def register_tools_from_entry_points(self, group: str = "custom_tools"):
        """Wrapper for discovering tools from entry points."""
        self.registry.discover_tools_from_entry_points(group)

    def handle_user_request(self, user_text: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Given a natural language request, ask LLM to pick ONE tool and args.
        Then execute it via PlanExecutor.
        """
        # Build prompt with available tools
        system_prompt, user_prompt = self._build_prompt(user_text, context)

        if not system_prompt or not user_prompt:
            return {}

        # Query LLM
        plan = self.llm.inference(system_prompt, user_prompt)
        self.logger(f"Plan received:\n{plan}")

        # Execute
        result = self.executor.run(plan)
        return {"plan": plan, **result}

    def _build_prompt(self, user_text: str, ctx: Dict[str, Any]) -> Tuple[str, str]:
        """
        Create a simple prompt listing available tools and asking for one.
        The prompt is divided into 'system' and 'user' parts, which will be handled by the LLM agent
        to create the most appropriate prompt for the specific LLM.
        """
        tools = self.registry.top_k(user_text, self.k)
        if not tools:
            self.logger("No tools available in the registry.", error=True)
            return "", ""
        tool_descriptions = []
        for tool in tools:
            tool_descriptions.append(
                f"- **{tool.name}**: {tool.description}\n"
                f"  Inputs: {tool.input_schema}\n"
                f"  Outputs: {tool.output_schema}\n"
            )
        tools_text = "\n".join(tool_descriptions)

        user_prompt = "User request:\n" + user_text

        return self._get_prompt_template().format(tools_text=tools_text), user_prompt

    def _get_prompt_template(self) -> str:
        template = """
You are a planner assistant controlling a robot.
Your job is to take a user request and generate a valid execution plan, containing only ONE step.
Be sure to understand the text received and select the best action command from the available options.

## Available tools:
{tools_text}

## Plan format:

plan = GlobalPlan(
            plan=[
                PlanNode(
                    kind="SEQUENCE | PARALLEL",
                    steps=[
                        Step(tool="tool_name", args=[ArgValue(key="arg_name", val="value or {{{{bb.tool.key}}}}"), ...]),
                    ],
                    ## Optional execution control parameters:
                    condition="Python expression to evaluate before executing this PlanNode",
                    success_criteria="Python expression to determine if this PlanNode succeeded",
                    timeout_ms=0,
                    retry=0,
                    on_fail=PlanNode(
                        kind="SEQUENCE | PARALLEL",
                        steps=[
                            Step(tool="tool_name", args=[ArgValue(key="arg_name", val="value or {{{{bb.tool.key}}}}"), ...]),
                        ],
                    ),
                )
            ],
        )

Use "{{{{bb.tool.key}}}}" to reference the output of a previous step.
For example, if tool 'detect_object' outputs {{"pose": [1.0, 2.0]}}, you can pass it to navigate as:
"args": {{"target": "{{{{bb.detect_object.pose}}}}"}}

Choose the most appropriate tool and arguments to satisfy the request.
Add only optional execution control parameters if strictly necessary or requested by the user.
"""
        return template
