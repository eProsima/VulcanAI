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
import contextlib
import io
import re
import time
from typing import Dict, Any, Optional, Set, Tuple, List

from vulcanai.console.logger import VulcanAILogger
from vulcanai.core.plan_types import GlobalPlan, PlanBase, Step, ArgValue


TYPE_CAST = {
    "float": float,
    "int": int,
    "bool": lambda v: v if isinstance(v, bool)
                      else str(v).strip().lower() in ("1","true","yes","on"),
    "str": str,
}


class Blackboard(dict):
    """Shared memory for passing outputs between steps."""

    def text_snapshot(self, keys: Optional[List[str]] = None) -> str:
        """
        Return a string representing the blackboard.

        :param keys: Optional list of keys to include. If None, include all.
        :return: A string representation of the blackboard entries.
        """
        snapshot = {}
        keyset: Set[str] = set(keys) if keys is not None else set(self.keys())
        filtered = {k: self.get(k) for k in keyset if k in self}

        for k, v in filtered.items():
            if isinstance(v, (str, int, float, bool, list, dict, type(None))):
                snapshot[k] = v
            else:
                snapshot[k] = f"<{type(v).__name__}>"

        return str(snapshot)

class PlanExecutor:
    """Executes a validated GlobalPlan with blackboard and execution control parameters."""

    class_color = "#15B606"
    color_variable = "#C49C00"
    color_value = "#069899"
    color_error = "#CC0C0C"

    def __init__(self, registry, logger=None):
        self.registry = registry
        self.logger = logger

    def run(self, plan: GlobalPlan, bb: Blackboard) -> Dict[str, Any]:
        """
        Execute the entire plan.

        :param plan: The @GlobalPlan to execute.
        :param bb: The blackboard to use for sharing data between steps.
        :return: A dictionary with execution success status and the final blackboard state.
        """
        ok = True
        for node in plan.plan:
            ok = self._run_plan_node(node, bb)
            if not ok:
                break
        return {"success": ok, "blackboard": bb}

    def _run_plan_node(self, node: PlanBase, bb: Blackboard) -> bool:
        """Run a PlanNode with execution control parameters."""
        # Evaluate PlanNode-level condition
        if node.condition and not self.safe_eval(node.condition, bb):
            # Print in textual terminal:
            # [EXECUTOR] Skipping PlanNode <node.kind> due to not fulfilled condition=<node.condition>
            self.logger(f"Skipping PlanNode {node.kind} due to not fulfilled " + \
                        f"condition={node.condition}", log_type="executor")
            return True

        attempts = node.retry + 1 if node.retry else 1
        for i in range(attempts):
            ok = self._execute_plan_node_with_timeout(node, bb)
            if ok and self._check_success(node, bb):
                # Print in textual terminal:
                # [EXECUTOR] PlanNode <node.kind> succeeded on attempt <i+1>/<attempts>
                self.logger(f"PlanNode [{self.color_value}]{node.kind}[/{self.color_value}] " + \
                            f"[{self.class_color}]succeeded[/{self.class_color}] " + \
                            f"on attempt {i+1}/{attempts}", log_type="executor")
                return True
        # Print in textual terminal:
        # [EXECUTOR] PlanNode <node.kind> failed on attempt <i+1>/<attempts>"
        self.logger(f"PlanNode {node.kind} failed on attempt [bold]{i+1}/{attempts}[/bold]",
                    log_type="executor", log_color=0)

        if node.on_fail:
            # Print in textual terminal:
            # [EXECUTOR] Executing on_fail branch for PlanNode <node.kind>
            self.logger(f"Executing on_fail branch for PlanNode " + \
                        f"[{self.color_value}]{node.kind}[/{self.color_value}]", log_type="executor")
            # Execute the on_fail branch but ignore its result and return False
            self._run_plan_node(node.on_fail, bb)

        return False

    def _execute_plan_node_with_timeout(self, node: PlanBase, bb: Blackboard) -> bool:
        """Check if timeout needs to be considered for executing the PlanNode logic."""
        if node.timeout_ms:
            try:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    future = executor.submit(self._execute_plan_node, node, bb)
                    return future.result(timeout=node.timeout_ms / 1000.0)
            except concurrent.futures.TimeoutError:
                # Print in textual terminal:
                # [EXECUTOR] PlanNode <node.kind> timed out after <node.timeout_ms> ms
                self.logger(f"PlanNode {node.kind} timed out after [bold]{node.timeout_ms} ms[/bold]",
                            log_type="executor", log_color=0)
                return False
        else:
            return self._execute_plan_node(node, bb)

    def _execute_plan_node(self, node: PlanBase, bb: Blackboard) -> bool:
        """Execute a PlanNode, which can be SEQUENCE or PARALLEL."""
        if node.kind == "SEQUENCE":
            for step in node.steps:
                if not self._run_step(step, bb):
                    return False
            return True

        if node.kind == "PARALLEL":
            with concurrent.futures.ThreadPoolExecutor() as executor:
                futures = [executor.submit(self._run_step, step, bb, parallel=True) for step in node.steps]
                results = [f.result() for f in futures]
            return all(results)

        # Pydantic should have validated this already

        # Print in textual terminal:
        # [EXECUTOR] Unknown PlanNode kind <node.kind>, skipping
        self.logger(f"Unknown PlanNode kind {node.kind}, skipping", log_type="executor", log_color=0)
        return True

    def _run_step(self, step: Step, bb: Blackboard, parallel: bool = False) -> bool:
        # Evaluate Step-level condition
        if step.condition and not self.safe_eval(step.condition, bb):
            # Print in textual terminal:
            # [EXECUTOR] Skipping step '<step.tool>' due to condition=<step.condition>
            self.logger(f"Skipping step [italic]'{step.tool}'[/italic] " + \
                        f"due to condition=[{self.class_color}]{step.condition}[/{self.class_color}]",
                        log_type="executor")
            return True

        # Bind args with blackboard placeholders
        tool = self.registry.tools.get(step.tool)
        input_schema = tool.input_schema if tool else []
        args = self._bind_args(step.args, input_schema, bb)

        attempts = step.retry + 1 if step.retry else 1
        for i in range(attempts):
            # Call the tool
            ok, out = self._call_tool(step.tool, args, timeout_ms=step.timeout_ms, bb=bb, parallel=parallel)
            # Save output to blackboard
            bb[step.tool] = out
            # Check if the step succeeded
            if ok and self._check_success(step, bb, is_step=True):
                return True
            else:
                # Print in textual terminal:
                # [EXECUTOR] Step '<step.tool>' attempt <i+1>/<attempts> failed
                self.logger(f"Step [{self.class_color}][italic]'{step.tool}'[/italic][/{self.class_color}] " + \
                            f"attempt {i+1}/{attempts} failed", log_type="executor")

        return False

    def _check_success(self, entity: Step | PlanBase, bb: Blackboard, is_step: bool = False) -> bool:
        """Evaluate success criteria expression."""
        if not entity.success_criteria:
            # No criteria means any finishing is success
            return True
        log_value = entity.tool if is_step else entity.kind
        if self.safe_eval(entity.success_criteria, bb):
            # Print in textual terminal:
            # [EXECUTOR] Entity '<log_value>' succeeded with criteria=<entity.success_criteria>
            self.logger(f"Entity '{log_value}' [{self.class_color}]succeeded[/{self.class_color}] " + \
                        f"with criteria={entity.success_criteria}", log_type="executor")
            return True
        else:
            # Print in textual terminal:
            # [EXECUTOR] Entity '<log_value>' failed with criteria=<entity.success_criteria>
            self.logger(f"Entity '{log_value}' [{self.color_error}]failed[/{self.color_error}] " + \
                        f"with criteria={entity.success_criteria}", log_type="executor")
        return False

    def safe_eval(self, expr: str, bb: Blackboard) -> bool:
        """Evaluate a simple expression against bb."""
        try:
            if expr and isinstance(expr, str):
                # Some models might output 'None' as a string for no condition or success_criteria
                # In both cases, we treat it as always true, as there is no condition to evaluate
                if expr.strip().lower() == "none":
                    return True
                sub_expr = self._make_bb_subs(expr, bb)
                # Eval does not correctly evaluate dot notation with nested dicts
                return bool(eval(sub_expr))
        except Exception as e:
            # Print in textual terminal:
            # [EXECUTOR] Condition evaluation failed: <expr> (<exception>)
            self.logger(f"Condition evaluation failed: {expr} ({e})",
                        log_type="executor", log_color=0)
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
            # Print in textual terminal:
            # [EXECUTOR] Blackboard substitution failed: <expr> (<exception>)
            self.logger(f"Blackboard substitution failed: {expr} ({e})",
                        log_type="executor", log_color=0)
            return expr

    def _bind_args(self, args: List[ArgValue], schema: List[Tuple[str, str]], bb: Blackboard) -> List[ArgValue]:
        """Replace {{bb.key}} placeholders with actual values."""
        bound = []
        for arg in args:
            bound_arg = self._make_bb_subs(arg.val, bb)
            arg.val = self._coerce_to_schema(schema, arg.key, bound_arg)
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

    def _coerce_to_schema(self, schema: List[Tuple[str, str]], key: str, arg: str) -> Any:
        spec = dict(schema)
        t = spec.get(key, "str")
        try:
            out = TYPE_CAST[t](arg)
        except Exception:
            out = arg
        return out

    def _call_tool(self,
                   tool_name: str,
                   args: List[ArgValue],
                   timeout_ms: int = None,
                   bb: Blackboard = None,
                   parallel: bool = False) -> Tuple[bool, Any]:
        """Invoke a registered tool."""

        tool = self.registry.tools.get(tool_name)
        if not tool:
            # Print in textual terminal:
            # [EXECUTOR] Tool '<tool_name>' not found
            self.logger(f"Tool [italic]'{tool_name}'[/italic] not found",
                        log_type="executor", log_color=0)
            return False, None

        # Convert args list to dict
        arg_dict = {a.key: a.val for a in args}
        tool.bb = bb

        first = True

        msg = f"Invoking [italic][{self.class_color}]'{tool_name}'[/{self.class_color}][/italic] with args:"
        msg += "'{"
        for key, value in arg_dict.items():
            if first:
                msg += f"[{self.color_variable}]'{key}'[/{self.color_variable}]: " + \
                    f"[{self.color_value}]'{value}'[/{self.color_value}]"
            else:
                msg += f", [{self.color_variable}]'{key}'[/{self.color_variable}]: " + \
                    f"[{self.color_value}]'{value}'[/{self.color_value}]"
            first = False
        msg+="}''"
        self.logger(msg, log_type="executor")

        start = time.time()
        tool_log = ""
        try:
            if timeout_ms:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    if parallel:
                        future = executor.submit(tool.run, **arg_dict)
                        result = future.result(timeout=timeout_ms / 1000)
                    else:
                        buff = io.StringIO()
                        with contextlib.redirect_stdout(buff):
                            future = executor.submit(tool.run, **arg_dict)
                            result = future.result(timeout=timeout_ms / 1000)
                        tool_log = buff.getvalue().strip()
            else:
                if parallel:
                    result = tool.run(**arg_dict)
                else:
                    buff = io.StringIO()
                    with contextlib.redirect_stdout(buff):
                        result = tool.run(**arg_dict)
                    tool_log = buff.getvalue().strip()
            if tool_log:
                # Print in textual terminal:
                # [EXECUTOR] <tool_log>: <tool_name>
                self.logger(f"{tool_log}: {tool_name}", log_type="executor")
            elapsed = (time.time() - start) * 1000
            # Print in textual terminal:
            # [EXECUTOR] Executed '<tool_name>' in <elapsed> ms with result: <result>
            self.logger(f"Executed [italic][{self.class_color}]'{tool_name}'[/{self.class_color}][/italic] " + \
                        f"in [{self.color_value}]{elapsed:.1f} ms[/{self.color_value}] " + \
                        f"with result:", log_type="executor")

            if isinstance(result, dict):
                for key, value in result.items():
                    if key == "ros2":
                        continue
                    self.logger(f"[bold]{key}[/bold]")
                    self.logger(value)
            else:
                self.logger(result)

            return True, result
        except concurrent.futures.TimeoutError:
            # Print in textual terminal:
            # [EXECUTOR] Execution of '<tool_name>' timed out after <timeout_ms> ms
            self.logger(f"Execution of [italic][{self.class_color}]'{tool_name}'[/{self.class_color}][/italic] " + \
                        f"[{self.color_error}]timed out[/{self.color_error}] " + \
                        f"after [{self.color_value}]{timeout_ms}[/{self.color_value}] ms",
                        log_type="executor")
            return False, None
        except Exception as e:
            # Print in textual terminal:
            # [EXECUTOR] Execution failed for '<tool_name>': <exception>
            self.logger(f"Execution [bold][{self.color_error}]failed[/{self.color_error}][/bold] for " + \
                        f"[italic][{self.class_color}]'{tool_name}'[/{self.class_color}][/italic]: {e}", log_type="executor")
            return False, None
