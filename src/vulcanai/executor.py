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

import concurrent.futures
import re
import time
from typing import Dict, Any, Tuple, List
from vulcanai.plan_types import GlobalPlan, PlanNode, Step, ArgValue


class Blackboard(dict):
    """Shared memory for passing outputs between steps."""
    pass


class PlanExecutor:
    """Executes a validated GlobalPlan with blackboard and execution control parameters."""

    def __init__(self, registry, logger=None):
        self.registry = registry
        self.logger = logger or print

    def run(self, plan: GlobalPlan) -> Dict[str, Any]:
        """
        Execute the entire plan.

        :param plan: The @GlobalPlan to execute.
        :return: A dictionary with execution success status and the final blackboard state.
        """
        bb = Blackboard()
        ok = True
        for node in plan.plan:
            ok = self._run_plan_node(node, bb)
            if not ok:
                break
        return {"success": ok, "blackboard": bb}

    def _run_plan_node(self, node: PlanNode, bb: Blackboard) -> bool:
        """Run a PlanNode with execution control parameters."""
        # Evaluate PlanNode-level condition
        if node.condition and not self._safe_eval(node.condition, bb):
            self.logger(f"Skipping PlanNode {node.kind} due to not fulfilled condition={node.condition}")
            return True

        attempts = node.retry + 1 if node.retry else 1
        for i in range(attempts):
            self.logger(f"Executing PlanNode {node.kind} attempt {i+1}/{attempts}")
            ok = self._execute_plan_node_with_timeout(node, bb)
            if ok and self._check_success(node, bb):
                self.logger(f"PlanNode {node.kind} succeeded on attempt {i+1}/{attempts}")
                return True
            self.logger(f"PlanNode {node.kind} failed on attempt {i+1}/{attempts}")

        if node.on_fail:
            self.logger(f"Executing on_fail branch for PlanNode {node.kind}")
            # Prevent nested on_fail to avoid infinite loops
            node.on_fail.on_fail = None
            # Execute the on_fail branch but ignore its result and return False
            self._run_plan_node(node.on_fail, bb)

        return False

    def _execute_plan_node_with_timeout(self, node: PlanNode, bb: Blackboard) -> bool:
        """Check if timeout needs to be considered for executing the PlanNode logic."""
        if node.timeout_ms:
            try:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    future = executor.submit(self._execute_plan_node, node, bb)
                    return future.result(timeout=node.timeout_ms / 1000.0)
            except concurrent.futures.TimeoutError:
                self.logger(f"PlanNode {node.kind} timed out after {node.timeout_ms} ms")
                return False
        else:
            return self._execute_plan_node(node, bb)

    def _execute_plan_node(self, node: PlanNode, bb: Blackboard) -> bool:
        """Execute a PlanNode, which can be SEQUENCE or PARALLEL."""
        if node.kind == "SEQUENCE":
            for step in node.steps:
                if not self._run_step(step, bb):
                    return False
            return True

        if node.kind == "PARALLEL":
            with concurrent.futures.ThreadPoolExecutor() as executor:
                futures = [executor.submit(self._run_step, step, bb) for step in node.steps]
                results = [f.result() for f in futures]
            return all(results)

        # Pydantic should have validated this already
        self.logger(f"Unknown PlanNode kind {node.kind}, skipping")
        return True

    def _run_step(self, step: Step, bb: Blackboard) -> bool:
        # Evaluate Step-level condition
        if step.condition and not self._safe_eval(step.condition, bb):
            self.logger(f"Skipping step {step.tool} due to condition={step.condition}")
            return True

        # Bind args with blackboard placeholders
        args = self._bind_args(step.args, bb)

        attempts = step.retry + 1 if step.retry else 1
        for i in range(attempts):
            # Call the tool
            ok, out = self._call_tool(step.tool, args, timeout_ms=step.timeout_ms, bb=bb)
            # Save output to blackboard
            bb[step.tool] = out
            # Check if the step succeeded
            if ok and self._check_success(step, bb, is_step=True):
                return True
            else:
                self.logger(f"Step {step.tool} attempt {i+1}/{attempts} failed")

        return False

    def _check_success(self, entity: Step | PlanNode, bb: Blackboard, is_step: bool = False) -> bool:
        """Evaluate success criteria expression."""
        if not entity.success_criteria:
            # No criteria means any finishing is success
            return True
        log_value = entity.tool if is_step else entity.kind
        if self._safe_eval(entity.success_criteria, bb):
            self.logger(f"Entity '{log_value}' succeeded with criteria={entity.success_criteria}")
            return True
        else:
            self.logger(f"Entity '{log_value}' failed with criteria={entity.success_criteria}")
        return False

    def _safe_eval(self, expr: str, bb: Blackboard) -> bool:
        """Evaluate a simple expression against bb."""
        try:
            if expr and isinstance(expr, str):
                expr = self._make_bb_subs(expr, bb)
                # Eval does not correctly evaluate dot notation with nested dicts
                return bool(eval(expr))
        except Exception as e:
            self.logger(f"Condition evaluation failed: {expr} ({e})")
        return False

    def _make_bb_subs(self, expr: str, bb: Blackboard) -> str:
        """Substitute all {{bb.key}} in expr with their values from bb."""
        try:
            if expr and isinstance(expr, str):
                matches = re.findall(r"\{\{(bb\..*?)\}\}", expr)
                for match in matches:
                    val = self._get_from_bb(f"{match}", bb)
                    expr = expr.replace(f"{{{{{match}}}}}", str(val))
            return expr
        except Exception as e:
            self.logger(f"Blackboard substitution failed: {expr} ({e})")
            return expr

    def _bind_args(self, args: List[ArgValue], bb: Blackboard) -> List[ArgValue]:
        """Replace {{bb.key}} placeholders with actual values."""
        bound = []
        for arg in args:
            arg.val = self._make_bb_subs(arg.val, bb)
            bound.append(arg)
        return bound

    def _get_from_bb(self, path: str, bb: Blackboard) -> Any:
        """Retrieve a value from the blackboard given a dot-separated path."""
        keys = path.replace("{{", "").replace("}}", "").replace("bb.", "").split(".")
        val = bb
        for key in keys:
            if "[" in key and "]" in key:
                base, idx = key[:-1].split("[")
                val = val.get(base, None) if isinstance(val, dict) else None
                if isinstance(val, list):
                    try:
                        val = val[int(idx)]
                    except (IndexError, ValueError):
                        val = None
                        break
                else:
                    val = None
                    break
            else:
                val = val.get(key, None) if isinstance(val, dict) else None
        return val

    def _call_tool(self, tool_name: str, args: List[ArgValue], timeout_ms: int = None, bb: Blackboard = None) -> Tuple[bool, Any]:
        """Invoke a registered tool."""
        tool = self.registry.tools.get(tool_name)
        if not tool:
            self.logger(f"Tool {tool_name} not found")
            return False, None

        # Convert args list to dict
        arg_dict = {a.key: a.val for a in args}
        tool.bb = bb

        start = time.time()
        try:
            if timeout_ms:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    future = executor.submit(tool.run, **arg_dict)
                    result = future.result(timeout=timeout_ms / 1000)
            else:
                result = tool.run(**arg_dict)
            elapsed = (time.time() - start) * 1000
            self.logger(f"Executed {tool_name} in {elapsed:.1f} ms with result: {result}")
            return True, result
        except concurrent.futures.TimeoutError:
            self.logger(f"Execution of {tool_name} timed out after {timeout_ms} ms")
            return False, None
        except Exception as e:
            self.logger(f"Execution failed for {tool_name}: {e}")
            return False, None
