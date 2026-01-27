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
    "Agent": ".agent:Agent",
    "ArgValue": ".plan_types:ArgValue",
    "Blackboard": ".executor:Blackboard",
    "GlobalPlan": ".plan_types:GlobalPlan",
    "IterativeManager": ".manager_iterator:IterativeManager",
    "PlanExecutor": ".executor:PlanExecutor",
    "PlanManager": ".manager_plan:PlanManager",
    "PlanNode": ".plan_types:PlanNode",
    "PlanValidator": ".validator:PlanValidator",
    "Step": ".plan_types:Step",
    "TimelineEvent": ".manager_iterator:TimelineEvent",
    "ToolManager": ".manager:ToolManager",
}

__all__ = list(_EXPORTS.keys())


def __getattr__(name: str):
    target = _EXPORTS.get(name)
    if not target:
        raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
    module_name, attr_name = target.split(":")
    if module_name.startswith("."):
        module = import_module(module_name, package=__name__)
    else:
        module = import_module(module_name)
    return getattr(module, attr_name)


def __dir__() -> list[str]:
    """Make dir() show the public API."""
    return sorted(list(globals().keys()) + __all__)
