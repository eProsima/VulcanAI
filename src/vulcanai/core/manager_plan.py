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

from typing import Optional

from vulcanai.core.manager import ToolManager
from vulcanai.core.validator import PlanValidator
from vulcanai.tools.tool_registry import ToolRegistry


class PlanManager(ToolManager):
    """Manager to target complex plan generation, involving multiples plans and steps."""

    def __init__(
        self,
        model: str,
        registry: Optional[ToolRegistry] = None,
        validator: Optional[PlanValidator] = None,
        k: int = 5,
        hist_depth: int = 3,
        logger=None,
        default_tools=True
    ):
        super().__init__(model, registry=registry, validator=validator, k=k, hist_depth=hist_depth, logger=logger, default_tools=default_tools)

    def _get_prompt_template(self) -> str:
        """
        Override the template generation method to allow multiple PlanNodes and Steps.
        """
        template = """
You are a planner assistant controlling a robotic system.
Your job is to take a user request and generate a valid execution plan,
containing one or more steps grouped into one or more PlanNodes.
Use PlanNodes to group steps that need to be executed together, either in sequence or in parallel, to
achieve sub-goals.
{user_context}
## Available tools:
{tools_text}

## Plan format:

plan = GlobalPlan(
    summary="Brief description of the plan",
    plan=[
        PlanNode(
            kind="SEQUENCE | PARALLEL",
            steps=[
                Step(tool="tool_name", args=[ArgValue(key="arg_name", val="value or {{{{bb.tool.key}}}}"), ...],
                ## Optional execution control parameters. Do not add if not strictly necessary:
                condition="Python expression to evaluate before executing this Step",
                success_criteria="Python expression to determine if this Step succeeded",
                timeout_ms=0,
                retry=0),
                ...
            ],
            ## Optional execution control parameters. Do not add if not strictly necessary:
            condition="Python expression to evaluate before executing this PlanNode",
            success_criteria="Python expression to determine if this PlanNode succeeded",
            timeout_ms=0,
            retry=0,
            on_fail=PlanNode(
                kind="SEQUENCE | PARALLEL",
                steps=[
                    Step(tool="tool_name", args=[
                        ArgValue(key="arg_name", val="value or {{{{bb.tool.key}}}}"),
                        ...
                    ]),
                ],
            ),
        ),
        ...
    ],
)

Use "{{{{bb.tool.key}}}}" to reference the output of a previous step.
For example, if tool 'detect_object' outputs {{"pose": [1.0, 2.0]}},
you can pass it to navigate as:
args=[ArgValue(key="target", val="{{{{bb.detect_object.pose}}}}")]

Choose the best combination of tools to satisfy the request.
Add only optional execution control parameters if strictly necessary or requested by the user.
They are not needed to create a Step or PlanNode.
"""
        return template
