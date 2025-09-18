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
from typing import Dict, Any, Tuple
from vulcanai.plan_types import GlobalPlan, PlanNode, Step


class Blackboard(dict):
    """Shared memory for passing outputs between steps."""
    pass


class PlanExecutor:
    """Executes a validated GlobalPlan with blackboard and conditions."""

    def __init__(self, registry, logger=None):
        self.registry = registry
        self.logger = logger or print

    def run(self, plan: GlobalPlan) -> Dict[str, Any]:
        bb = Blackboard()
        ok = True
        for node in plan.plan:
            ok = self._run_node(node, bb)
            if not ok:
                break
        return {"success": ok, "blackboard": bb}

    def _run_node(self, node: PlanNode, bb: Blackboard) -> bool:
        # Evaluate PlanNode-level condition
        if node.condition and not self._safe_eval(node.condition, bb):
            self.logger(f"Skipping node {node.kind} due to not fulfilled condition={node.condition}")
            return True

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

        self.logger(f"Unknown node kind {node.kind}, skipping")
        return True

    def _run_step(self, step: Step, bb: Blackboard) -> bool:
        # Evaluate Step-level condition
        if step.condition and not self._safe_eval(step.condition, bb):
            self.logger(f"Skipping step {step.tool} due to condition={step.condition}")
            return True

        # Bind args with blackboard placeholders
        args = self._bind_args(step.args, bb)

        attempts = step.retry + 1
        for i in range(attempts):
            # Call the tool
            ok, out = self._call_tool(step.tool, args, timeout_ms=step.timeout_ms)
            # Save output to blackboard
            bb[step.tool] = out
            # Check if the step succeeded
            if ok and self._check_success(step, bb):
                return True
            else:
                self.logger(f"Step {step.tool} attempt {i+1}/{attempts} failed")

        return False

    def _check_success(self, step: Step, bb: Blackboard) -> bool:
        """Evaluate success criteria expression."""
        if not step.success_criteria:
            # No criteria means any finishing is success
            return True
        if self._safe_eval(step.success_criteria, bb):
            self.logger(f"Step {step.tool} succeeded with criteria={step.success_criteria}")
            return True
        else:
            self.logger(f"Step {step.tool} failed with criteria={step.success_criteria}")
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

    def _bind_args(self, args: Dict[str, Any], bb: Blackboard) -> Dict[str, Any]:
        """Replace {{bb.key}} placeholders with actual values."""
        bound = {}
        for k, v in args.items():
            if isinstance(v, str) and v.startswith("{{bb.") and v.endswith("}}"):
                val = self._get_from_bb(v, bb)
                bound[k] = val
            else:
                bound[k] = v
        return bound

    def _get_from_bb(self, path: str, bb: Blackboard) -> Any:
        """Retrieve a value from the blackboard given a dot-separated path."""
        keys = path.replace("{{", "").replace("}}", "").replace("bb.", "").split(".")
        val = bb
        for key in keys:
            val = val.get(key, None) if isinstance(val, dict) else None
        return val

    def _call_tool(self, tool_name: str, args: Dict[str, Any], timeout_ms: int = None) -> Tuple[bool, Any]:
        """Invoke a registered tool."""
        tool = self.registry.tools.get(tool_name)
        if not tool:
            self.logger(f"Tool {tool_name} not found")
            return False, None

        start = time.time()
        try:
            if timeout_ms:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    future = executor.submit(tool.run, **args)
                    result = future.result(timeout=timeout_ms / 1000)
            else:
                result = tool.run(**args)
            elapsed = (time.time() - start) * 1000
            self.logger(f"Executed {tool_name} in {elapsed:.1f} ms with result: {result}")
            return True, result
        except concurrent.futures.TimeoutError:
            self.logger(f"Execution of {tool_name} timed out after {timeout_ms} ms")
            return False, None
        except Exception as e:
            self.logger(f"Execution failed for {tool_name}: {e}")
            return False, None
