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

from importlib import import_module

_EXPORTS = {
    # Tools
    "AtomicTool": "vulcanai.tools.tools:AtomicTool",
    "CompositeTool": "vulcanai.tools.tools:CompositeTool",
    "vulcanai_tool": "vulcanai.tools.tool_registry:vulcanai_tool",
    "ToolRegistry": "vulcanai.tools.tool_registry:ToolRegistry",
    # Core
    "ToolManager": "vulcanai.core.manager:ToolManager",
    "PlanManager": "vulcanai.core.manager_plan:PlanManager",
    "LLMAgent": "vulcanai.core.llm_agent:LLMAgent",
    "PlanExecutor": "vulcanai.core.executor:PlanExecutor",
    "Blackboard": "vulcanai.core.executor:Blackboard",
    "ArgValue": "vulcanai.core.plan_types:ArgValue",
    "Step": "vulcanai.core.plan_types:Step",
    "PlanNode": "vulcanai.core.plan_types:PlanNode",
    "GlobalPlan": "vulcanai.core.plan_types:GlobalPlan",
    "PlanValidator": "vulcanai.core.validator:PlanValidator",
    # Console
    "VulcanConsole": "vulcanai.console.console:VulcanConsole",
    "VulcanAILogger": "vulcanai.console.logger:VulcanAILogger",
}

__all__ = list(_EXPORTS.keys())

def __getattr__(name: str):
    target = _EXPORTS.get(name)
    if not target:
        raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
    module_name, attr_name = target.split(':')
    module = import_module(module_name)
    return getattr(module, attr_name)

def __dir__() -> list[str]:
    """Make dir() show the public API."""
    return sorted(list(globals().keys()) + __all__)
