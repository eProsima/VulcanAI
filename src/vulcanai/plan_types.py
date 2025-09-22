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


class PlanNode(BaseModel):
    """
    A node that defines a plan which is composed of one or more steps and
    has execution control parameters.
    """
    # Sequence or parallel execution of children
    kind: Kind
    # Child nodes
    steps: List[Step] = Field(default_factory=list)
    # Execution control
    condition: Optional[str] = None
    success_criteria: Optional[str] = None
    timeout_ms: Optional[int] = None
    retry: int = 0
    on_fail: Optional["PlanNode"] = None


class GlobalPlan(BaseModel):
    """GlobalPlan returned by the LLM with each step to be executed."""
    # Top-level plan structure. Always executed sequentially.
    plan: List[PlanNode] = Field(default_factory=list)
    # Brief summary of the plan
    summary: Optional[str] = None
