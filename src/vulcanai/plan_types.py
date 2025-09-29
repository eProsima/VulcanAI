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


class Step(BaseModel):
    """Atomic execution unit bound to a ITool."""
    # Associated tool
    tool: str = None
    # Tool arguments
    args: List[ArgValue] = Field(
        default_factory=list,
        description="List of key-value pairs representing tool arguments",
    )
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

        for i, node in enumerate(self.plan, 1):
            lines.append(f"- PlanNode {i}: kind={node.kind}")
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
                arg_str = ", ".join([f"{a.key}={a.val}" for a in step.args]) if step.args else "no args"
                lines.append(f"    Step {j}: {step.tool}({arg_str})")
                if step.condition:
                    lines.append(f"      Condition: {step.condition}")
                if step.retry:
                    lines.append(f"      Retry: {step.retry}")
                if step.timeout_ms:
                    lines.append(f"      Timeout: {step.timeout_ms} ms")
                if step.success_criteria:
                    lines.append(f"      Success Criteria: {step.success_criteria}")
        return "\n".join(lines)
