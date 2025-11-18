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

from pydantic import BaseModel, Field
from typing import List, Literal, Optional, Union


Kind = Literal["SEQUENCE","PARALLEL"]


class ArgValue(BaseModel):
    """Key-value pair representing a tool argument."""
    key: str
    val: Union[str, int, float, bool]


class StepBase(BaseModel):
    """Atomic execution unit bound to a ITool."""
    # Associated tool
    tool: str = None
    # Tool arguments
    args: List[ArgValue] = Field(
        default_factory=list,
        description="List of key-value pairs representing tool arguments",
    )


class Step(StepBase):
    """Final atomic execution unit bound to a ITool with execution control."""
    # Execution control
    condition: Optional[str] = None
    success_criteria: Optional[str] = None
    timeout_ms: Optional[int] = None
    retry: int = 0


class PlanBase(BaseModel):
    """
    A base class that defines a plan which is composed of one or more steps and
    has execution control parameters.
    """
    # SEQUENCE or PARALLEL execution of children
    kind: Kind
    # Child nodes
    steps: List[Step] = Field(default_factory=list)
    # Execution control
    condition: Optional[str] = None
    success_criteria: Optional[str] = None
    timeout_ms: Optional[int] = None
    retry: int = 0


class PlanNode(PlanBase):
    """
    Final plan node with optional failure handling.
    """
    on_fail: Optional["PlanBase"] = None


class GlobalPlan(BaseModel):
    """GlobalPlan returned by the LLM with each step to be executed."""
    # Top-level plan structure. Always executed sequentially.
    plan: List[PlanNode] = Field(default_factory=list)
    # Brief summary of the plan
    summary: Optional[str] = None

    def __str__(self) -> str:
        lines = []
        if self.summary:
            lines.append(f"- [bold]Plan Summary[/bold]: {self.summary}\n")

        color_tool = "#15B606"
        color_variable = "#C49C00"
        color_value = "#069899"

        for i, node in enumerate(self.plan, 1):
            lines.append(f"- PlanNode {i}: [{color_variable}]kind[/{color_variable}]=[{color_value}]{node.kind}[/{color_value}]")
            if node.condition:
                lines.append(f"    Condition: {node.condition}")
            if node.retry:
                lines.append(f"    Retry: {node.retry}")
            if node.timeout_ms:
                lines.append(f"    Timeout: {node.timeout_ms} ms")
            if node.success_criteria:
                lines.append(f"    Success Criteria: {node.success_criteria}")
            if node.on_fail:
                lines.append(f"    On Fail: {node.on_fail.kind} with {len(node.on_fail.steps)} steps")
            for j, step in enumerate(node.steps, 1):
                #arg_str: <key_0>=<val_0>, ..., <key_n-1>=<val_n-1>
                arg_str = ", ".join([f"[{color_variable}]{a.key}[/{color_variable}]=[{color_value}]{a.val}[/{color_value}]" for a in step.args]) if step.args else f"[{color_value}]no args[/{color_value}]"
                # Step <num_step>: <tool>(<arg_str>)
                lines.append(f"    Step {j}: [{color_tool}]{step.tool}[/{color_tool}]({arg_str})")
                if step.condition:
                    lines.append(f"      Condition: {step.condition}")
                if step.retry:
                    lines.append(f"      Retry: {step.retry}")
                if step.timeout_ms:
                    lines.append(f"      Timeout: {step.timeout_ms} ms")
                if step.success_criteria:
                    lines.append(f"      Success Criteria: {step.success_criteria}")
        return "\n".join(lines)


class GoalSpec(BaseModel):
    """Specification defining the user goal to be achieved."""
    summary: str
    # Mode used for goal verification:
    # - [Perceptual] mode uses AI to verify if the goal has been achieved based on evidence (e.g., images) or blackboard data.
    # - [Objective] mode uses deterministic predicates based on validation tools results to verify if the goal has been achieved.
    mode: Literal["perceptual", "objective"] = "objective"
    # List of simple boolean predicates over the blackboard (e.g., "{{bb.navigation.at_target}} == true")
    success_predicates: List[str] = Field(default_factory=list)
    # Tools called every iteration to verify if the goal has been achieved, described as (tool_name, [args])
    verify_tools: List[StepBase] = Field(default_factory=list)
    # List of bb keys that the LLM should verify or give importance to verify if the goal was achieved
    evidence_bb_keys: List[str] = Field(default_factory=list)

    def __str__(self) -> str:
        lines = []
        if self.summary:
            lines.append(f"- [bold]Goal Summary[/bold]: {self.summary}")
        if self.mode:
            lines.append(f"- Verification Mode: {self.mode}")

        for i, pred in enumerate(self.success_predicates, 1):
            lines.append(f"- Success Predicate {i}: {pred}")
        for i, tool in enumerate(self.verify_tools, 1):
            lines.append(f"- Verify Tool {i}: {tool.tool}({tool.args})")
        if self.evidence_bb_keys:
            lines.append(f"- Evidence BB Keys: {self.evidence_bb_keys}")
        return "\n".join(lines)


class AIValidation(BaseModel):
    """AI-based validation result."""
    success: bool
    confidence: float
    explanation: Optional[str] = None
