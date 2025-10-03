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

import re
from vulcanai.core.plan_types import GlobalPlan, PlanNode, Step

class PlanValidator:
    """Validates and optionally augments a plan before execution."""
    def __init__(self, registry):
        self.registry = registry

    def validate(self, plan: GlobalPlan):
        """
        Validate the provided plan structure and contents.
          1) Check tools exist and are available
          2) Validate args against each tool.input_schema
          3) Check tool output references are correctly formatted

        :param plan: GlobalPlan to validate
        :raises ValueError: if validation fails
        """
        if not isinstance(plan, GlobalPlan):
            raise ValueError("Provided plan is not a GlobalPlan instance.")
        for node in plan.plan:
            if isinstance(node, PlanNode):
                if not node.steps:
                    raise ValueError(f"PlanNode '{node.node_id}' has no steps defined.")
                for step in node.steps:
                    self._validate_step(step)

    def _validate_step(self, step: Step):
        """Validate a single step in the plan."""
        if not isinstance(step, Step):
            raise ValueError("Provided step is not a Step instance.")
        # Check tool exists and is registered
        if step.tool not in self.registry.tools:
            raise ValueError(f"Tool '{step.tool}' not found in registry.")
        # Validate args against tool input schema
        tool = self.registry.tools.get(step.tool)
        if tool.input_schema:
            if len(step.args) != len(tool.input_schema):
                raise ValueError(f"Tool '{tool.name}' expects {len(tool.input_schema)} arguments, but {len(step.args)} were provided.")
            for arg in step.args:
                if arg.key not in {k for d in tool.input_schema for k in d}:
                    raise ValueError(f"Argument '{arg.key}' not defined in tool '{tool.name}' input schema.")
                # Check that if there are blackboard references, they are correctly encapsulated
                if arg.val and isinstance(arg.val, str):
                    bb_items = re.findall(r"(bb\..*?)", arg.val)
                    if bb_items:
                        bb_correct_format = re.findall(r"\{\{(bb\..*?)\}\}", arg.val)
                        if len(bb_items) != len(bb_correct_format):
                            raise ValueError(f"Blackboard reference in argument '{arg.key}' of tool '{tool.name}' is incorrectly formatted: '{arg.val}'")
        else:
            if step.args:
                raise ValueError(f"Tool '{tool.name}' does not accept arguments, but arguments were provided.")
