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

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Literal


Kind = Literal["SEQUENCE","PARALLEL"]


@dataclass
class Step:
    """Atomic execution unit bound to a ToolInterface."""
    # Associated tool
    tool: str = None
    # Tool arguments
    args: Dict[str, Any] = field(default_factory=dict)
    # Execution control
    condition: Optional[str] = None
    success_criteria: Optional[str] = None
    timeout_ms: Optional[int] = None
    retry: int = 0

@dataclass
class PlanNode:
    """
    A node that defines a plan which is composed of one or more steps and
    has execution control parameters.
    """
    # Sequence or parallel execution of children
    kind: Kind
    # Child nodes
    steps: List[Step] = field(default_factory=list)
    # Execution control
    condition: Optional[str] = None
    success_criteria: Optional[str] = None
    timeout_ms: Optional[int] = None
    retry: int = 0
    on_fail: Optional["PlanNode"] = None

@dataclass
class GlobalPlan:
    """GlobalPlan returned by the LLM with each step to be executed."""
    # Top-level plan structure
    plan: List[PlanNode] = field(default_factory=list)
    # Brief summary of the plan
    summary: Optional[str] = None
