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

TYPE_ALIAS = {"int": int, "integer": int, "float": float, "bool": bool, "boolean": bool, "str": str, "string": str}


def _is_optional_schema_type(type_name: str) -> bool:
    return isinstance(type_name, str) and type_name.endswith("?")


def _base_schema_type(type_name: str) -> str:
    return type_name[:-1] if _is_optional_schema_type(type_name) else type_name


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
                    raise ValueError(f"PlanNode '{node.kind}' has no steps defined.")
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
            required_keys = {key for key, type_name in tool.input_schema if not _is_optional_schema_type(type_name)}
            provided_keys = {arg.key for arg in step.args}
            schema_len = len(tool.input_schema)
            required_len = len(required_keys)

            if len(step.args) < required_len or len(step.args) > schema_len:
                if required_len == schema_len:
                    raise ValueError(
                        f"Tool '{tool.name}' expects {schema_len} arguments,"
                        f" but {len(step.args)} were provided."
                    )
                raise ValueError(
                    f"Tool '{tool.name}' expects between {required_len} and {schema_len} arguments,"
                    f" but {len(step.args)} were provided."
                )

            missing_required = sorted(required_keys - provided_keys)
            if missing_required:
                raise ValueError(
                    f"Tool '{tool.name}' is missing required arguments: {', '.join(missing_required)}."
                )

            for arg in step.args:
                if arg.key not in {k for d in tool.input_schema for k in d}:
                    raise ValueError(f"Argument '{arg.key}' not defined in tool '{tool.name}' input schema.")
                # If the argument is of string type, it can contain blackboard references
                if arg.val and isinstance(arg.val, str):
                    # Check that if there are blackboard references, they are correctly encapsulated
                    bb_items = re.findall(r"(bb\..*?)", arg.val)
                    if bb_items:
                        bb_correct_format = re.findall(r"\{\{(bb\..*?)\}\}", arg.val)
                        if len(bb_items) != len(bb_correct_format):
                            raise ValueError(
                                f"Blackboard reference in argument '{arg.key}' of tool '{tool.name}'"
                                f" is incorrectly formatted: '{arg.val}'"
                            )
                    else:
                        # If there are no blackboard references, ensure the argument is of string type, as it must
                        # adhere to the schema
                        for schema in tool.input_schema:
                            if arg.key in schema:
                                is_string_type = _base_schema_type(schema[1]) in ["str", "string"]
                                if not is_string_type:
                                    raise ValueError(
                                        f"Argument '{arg.key}' of tool '{tool.name}' expects type"
                                        f" '{TYPE_ALIAS.get(_base_schema_type(schema[1]))}',"
                                        f" but got '{type(arg.val).__name__}'."
                                    )
                # Check type if static value for non-string types
                else:
                    expected_type = None
                    for schema in tool.input_schema:
                        if arg.key in schema:
                            print(f"Schema for arg '{arg.key}' in tool '{tool.name}': {schema}")  # Debug print
                            # Use TYPE_ALIAS to map string type names to actual types
                            expected_type = TYPE_ALIAS.get(_base_schema_type(schema[1]))
                            print(
                                f"Expected type for arg '{arg.key}' in tool '{tool.name}': {expected_type}"
                            )  # Debug print
                            break
                    if expected_type and not isinstance(arg.val, expected_type):
                        if expected_type is float and isinstance(arg.val, int):
                            # Allow int to float conversion
                            continue
                        raise ValueError(
                            f"Argument '{arg.key}' of tool '{tool.name}' expects type '{expected_type.__name__}',"
                            f" but got '{type(arg.val).__name__}'."
                        )
        else:
            if step.args:
                raise ValueError(f"Tool '{tool.name}' does not accept arguments, but arguments were provided.")
