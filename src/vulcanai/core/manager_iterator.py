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

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from vulcanai.core.manager import ToolManager
from vulcanai.core.plan_types import ArgValue, GlobalPlan, PlanNode, Step
from vulcanai.core.plan_types import GoalSpec
from vulcanai.core.validator import PlanValidator
from vulcanai.tools.tool_registry import ToolRegistry


class TimelineEvent(Enum):
    GOAL_SET = "goal_set"
    GOAL_ACHIEVED = "goal_achieved"
    PLAN_GENERATION_FAILED = "plan_generation_failed"
    PLAN_REPEATED = "plan_repeated"
    PLAN_NOT_VALID = "plan_not_valid"
    PLAN_EXECUTED = "plan_executed"


class IterativeManager(ToolManager):
    """Manager that implements iterative planning to re-adapt plans."""
    def __init__(
            self,
            model: str,
            registry: Optional[ToolRegistry]=None,
            validator: Optional[PlanValidator]=None,
            k: int=5,
            hist_depth: int = 3,
            logger=None,
            max_iters: int = 5,
            step_timeout_ms: Optional[int] = None
        ):
        super().__init__(model, registry, validator, k, max(3, hist_depth), logger)

        self.iter: int = 0
        self.max_iters: int = int(max_iters)
        self.step_timeout_ms: int = int(step_timeout_ms) if step_timeout_ms else 10000

        # Goal of the current user request
        self.goal = None
        # To check that no plans are repeated
        self._used_plans: set[str] = set()
        # Timeline for tracking iterations and class variables
        self._timeline: List[Dict[str, Any]] = []
        # Amount of timeline events to include in the prompts
        self._timeline_events_printed: int = 3

        self._single_tool_plan = self._init_single_tool_plan()

    def _restart_iterative_plan(self):
        """Reset the iterative planning state."""
        self.iter = 0
        self._used_plans = set()
        self._timeline = []
        if self.bb and self.bb.get("ai_validation", None):
            self.bb.pop("ai_validation")

    def handle_user_request(self, user_text: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Override the user request handler to implement iterative planning."""
        # Initialize class variables and timeline for tracking iterations
        self._restart_iterative_plan()

        # Get GoalSpec from user request
        try:
            self.goal = self._get_goal_from_user_request(user_text, context)
            self._timeline.append({"iteration": self.iter, "event": TimelineEvent.GOAL_SET.value})
        except Exception as e:
            self.logger.log_manager(f"Error getting goal from user request: {e}", error=True)
            return {"error": "Error getting goal from user request."}

        skip_verification_step = False
        while self.iter < self.max_iters:
            self.iter += 1
            self.logger.log_manager(f"--- Iteration {self.iter} ---")
            try:
                # Check progress toward the goal and stop if achieved
                if not skip_verification_step and self._verify_progress():
                    self._timeline.append({"iteration": self.iter, "event": TimelineEvent.GOAL_ACHIEVED.value})
                    break
                skip_verification_step = False

                # Get plan from LLM
                plan = self.get_plan_from_user_request(user_text, context)
                if not plan:
                    self.logger.log_manager(f"Error getting plan from model", error=True)
                    self._timeline.append({"iteration": self.iter, "event": TimelineEvent.PLAN_GENERATION_FAILED.value})
                    skip_verification_step = True
                    continue
                plan_str = str(plan.plan)
                if plan_str in self._used_plans:
                    self.logger.log_manager("LLM produced a repeated plan. Stopping iterations.", error=True)
                    self._timeline.append({"iteration": self.iter, "event": TimelineEvent.PLAN_REPEATED.value})
                    skip_verification_step = True
                    continue
                self._used_plans.add(plan_str)

                # Validate plan if validator is available
                if self.validator:
                    try:
                        self.validator.validate(plan)
                    except Exception as e:
                        self.logger.log_manager(f"Plan validation error. Asking for new plan: {e}")
                        self._timeline.append({"iteration": self.iter, "event": TimelineEvent.PLAN_NOT_VALID.value, "detail": str(e)})
                        continue

                # Set step timeouts if not set or too high
                for node in plan.plan:
                    for step in node.steps:
                        if step.timeout_ms is None or step.timeout_ms >= self.step_timeout_ms:
                            step.timeout_ms = self.step_timeout_ms

                # Execute plan
                ret = self.execute_plan(plan)
                self._timeline.append({
                    "iteration": self.iter,
                    "event": TimelineEvent.PLAN_EXECUTED.value,
                    "plan": plan.summary,
                    "result": ret.get("success", False),
                })

                # If execution was successful, return the result
                if not ret.get("success", False):
                    self.logger.log_manager(f"Iteration {self.iter} failed.", error=True)

            except Exception as e:
                self.logger.log_manager(f"Error handling user request: {e}", error=True)
                return {"error": str(e), "timeline": self._timeline}

        return {"timeline": self._timeline, "success": self.bb.get("goal_achieved", False), "blackboard": self.bb.copy(), "plan": plan}

    def _is_goal_achieved(self):
        """
        Check if the goal has been achieved based on the blackboard and update the blackboard.
        This method is intented to be called after running the verification tools and getting a valid goal.
        """
        preds = getattr(self.goal, "success_predicates", []) or []
        # All predicates must be true for the goal to be achieved
        success = all(bool(self.executor.safe_eval(p, self.bb)) for p in preds)
        self.bb["goal_achieved"] = success
        return success

    def _get_iter_context(self) -> str:
        """
        Get context about the current iteration to include in the prompt.
        Blackboard state is not included here, as it must be added separately when needed.
        """
        context_template="""\
- Iteration: {iteration}
- Goal used: {goal}
- Last timeline events:
{timeline_events}
- Current blackboard state:
{bb_snapshot}
"""

        timeline_events = ""
        if self._timeline:
            for e in self._timeline[-self._timeline_events_printed:]:
                if e.get('event', "") == TimelineEvent.PLAN_EXECUTED.value:
                    timeline_events += f"  - Iteration {e['iteration']}: {e['event']} - Plan: {e.get('plan', 'N/A')} - Result: {'Success' if e.get('result', False) else 'Failure'}\n"
                elif e.get('event', "") == TimelineEvent.PLAN_NOT_VALID.value:
                    timeline_events += f"  - Iteration {e['iteration']}: {e['event']} - Detail: {e.get('detail', 'N/A')}\n"
                else:
                    timeline_events += f"  - Iteration {e['iteration']}: {e['event']}\n"
        else:
            timeline_events = "  - None\n"

        # Blackboard state is not filled in this method
        context = context_template.format(
            iteration=self.iter,
            goal=self.goal.summary if self.goal else "N/A",
            timeline_events=timeline_events,
            bb_snapshot="{bb_snapshot}"  # Placeholder for blackboard snapshot
        )
        return context

    def _build_prompt(self, user_text: str, ctx: Dict[str, Any]) -> Tuple[str, str]:
        """Override the prompt building method to include iteration information."""
        tools = self.registry.top_k(user_text, self.k)
        if not tools:
            self.logger.log_manager("No tools available in the registry.", error=True)
            return "", ""
        tool_descriptions = []
        for tool in tools:
            tool_descriptions.append(
                f"- *{tool.name}*: {tool.description}\n"
                f"  Inputs: {tool.input_schema}\n"
                f"  Outputs: {tool.output_schema}\n"
            )
        tools_text = "\n".join(tool_descriptions)

        bb_snapshot = self.bb.text_snapshot()

        user_context = self._parse_user_context()

        system_prompt = self._get_prompt_template()
        system_prompt = system_prompt.format(
            tools_text=tools_text,
            user_context=user_context,
        )

        user_prompt = "## User Request: " + user_text + "\nContext:\n" + self._get_iter_context().format(bb_snapshot=bb_snapshot)

        return system_prompt, user_prompt

    def _build_goal_prompt(self, user_text: str, ctx: Dict[str, Any]) -> Tuple[str, str]:
        """
        Create a simple prompt listing available validation tools and asking for one.
        The prompt is divided into 'system' and 'user' parts, which will be handled by the LLM agent
        to create the most appropriate prompt for the specific LLM.
        """
        tools = self.registry.top_k(user_text, self.k, validation=True)
        if tools:
            tool_descriptions = []
            for tool in tools:
                tool_descriptions.append(
                    f"- *{tool.name}*: {tool.description}\n"
                    f"  Inputs: {tool.input_schema}\n"
                    f"  Outputs: {tool.output_schema}\n"
                )
            tools_text = "\n".join(tool_descriptions)
        else:
            tools_text = "No available tools. Use blackboard"

        user_context = self._parse_user_context()

        system_prompt = self._get_goal_prompt_template()
        system_prompt = system_prompt.format(
            user_context=user_context,
            tools_text=tools_text
        )
        user_prompt = "User request:\n" + user_text

        return system_prompt, user_prompt

    def _build_validation_prompt(self, instruction: str, evidence: Dict[str, Any]) -> Tuple[str, str]:
        """
        Create a simple prompt asking to verify if the user request was satisfied.
        The prompt is divided into 'system' and 'user' parts, which will be handled by the LLM agent
        to create the most appropriate prompt for the specific LLM.
        """

        user_context = self._parse_user_context()

        system_prompt = self._get_validation_prompt_template()
        system_prompt = system_prompt.format(
            user_context=user_context,
        )
        context_info = self._get_iter_context().format(bb_snapshot=evidence)
        user_prompt = context_info + "\nTask:\n" + instruction

        return system_prompt, user_prompt

    def _get_goal_prompt_template(self) -> str:
        """Goal prompt template used for generating the goal verification prompt."""
        template = """
You are a goal generator of a robotic/agent system.
Your objective is to produce the final goal to be verified during iterative execution from the user request.
You can access the blackboard (bb) to check the current state of the system, which is updated by the execution of verification tools.
Rules:
- You have two modes to choose from for goal verification: 'perceptual' and 'objective'.
  1) Objective mode uses deterministic predicates based on validation tools results to verify if the goal has been achieved.
  2) Perceptual mode uses AI to verify if the goal has been achieved based on evidence (e.g., images) or blackboard data.
- Whenever possible, prefer 'objective' mode as it is more reliable and faster to evaluate. Rely on 'perceptual' mode when the task requires a higher level of abstraction and cannot be easily converted to objective predicates.
- Add to 'verify_tools' any tool needed to verify the goal. It will be executed to update the blackboard before checking the success predicates. This applies to both modes.
- When using 'objective' mode, the 'success_predicates' must be simple boolean expressions over the blackboard.
- If no tool is useful to verify the goal rely on 'perceptual' mode.
- When using 'perceptual' mode, provide a 'evidence_bb_keys' list containing blackboard keys with relevant evidence to consider.
- Use "{{{{bb.tool_name.key}}}}" to reference tools results. This MUST be used in 'success_predicates'. It can also be used in 'verify_tools' arguments.
For example, if tool 'get_pose' outputs {{"pose": [1.0, 2.0]}}, you can pass reference it as "{{{{bb.get_pose.pose}}}}".

{user_context}
## Available tools:
{tools_text}
## Goal format:

goal = GoalSpec(
  summary="Brief description of the goal in human-readable form",
  mode="perceptual | objective", # Verification mode
  # Python expressions over the blackboard used ONLY in objective mode to evaluate success
  success_predicates: Optional[List[str]]=[
    "{{{{bb.tool_name.key}}}} == True",
    "{{{{bb.tool_name.key}}}} >= value"
  ],
  # Tools that need to be executed to verify the goal
  verify_tools=[
    ("tool_name", [ArgValue(key="arg_name", val="value or {{{{bb.tool_name.key}}}}"), ...]),
    ...
  ],
  # List of blackboard keys that should be used ONLY in perceptual mode to decide if the goal has been achieved
  evidence_bb_keys=[{{{{bb.tool_name.key}}}}...] # List of blackboard keys with relevant evidence to consider
)
"""
        return template

    def _get_prompt_template(self) -> str:
        """
        Override the template generation method to allow multiple PlanNodes and Steps.
        Note that the success_criteria is not included in the template, as success is determined at the goal level.
        """
        template = """
You are a iterative planner for a robotic/agent system.
Your objective is to produce a plan that will make progress toward the user's goal in the current iteration.
Rules:
- You will produce ONE actionable plan at a time. Prefer the simplest plan that makes tangible progress toward the goal.
- You must follow the logic execute, observe, and then re-plan if needed.
- System context will be provided as a blackboard that contains the current state of the system as outputs of tools.
- Keep the plan minimal and focused on the next immediate progress.
- CONTEXT will be provided about the current state of the system and previous iterations.
- If the last step failed, propose a different approach or tool/arguments and include a brief rationale of what will be done differently (as a comment-like line in the Summary).
- Add only optional execution control parameters if strictly necessary or requested by the user.
- Use "{{{{bb.tool_name.key}}}}" to pass outputs from previous steps if relevant.
For example, if tool 'detect_object' outputs {{"pose": [1.0, 2.0]}},
you can pass it to navigate as:
args=[ArgValue(key="target", val="{{{{bb.detect_object.pose}}}}")]

{user_context}
## Available tools:
{tools_text}

## Plan format (Simplest possible plan that makes progress):

plan = GlobalPlan(
  summary="Brief description of the plan",
  plan=[
    PlanNode(
      kind="SEQUENCE | PARALLEL", # Sequential or simultaneous execution
      steps=[
        Step(tool="tool_name", args=[ArgValue(key="arg_name", val="value or {{{{bb.tool_name.key}}}}"), ...],
      ],
      ## Optional execution control parameters. Do not add if not strictly necessary:
      condition="Python expression to evaluate before executing this PlanNode",
      timeout_ms=0,
      retry=0,
    ),
    ...
  ],
)
"""
        return template

    def _get_validation_prompt_template(self) -> str:
        """Validation prompt template used for during AI validation."""
        template = """
You are an evaluator of a robotic/agent system.
Your objective is to determine if the user request has been achieved, based on the evidence and images provided.
CONTEXT will be provided about the current state of the system and previous iterations.

{user_context}

## Validation format:
validation = AIValidation(
  success=bool, # True if the goal has been achieved, False otherwise
  confidence=float, # Confidence level (0.0 to 1.0)
  explanation=Optional[str] # Brief explanation of the decision
)
"""
        return template

    def _get_goal_from_user_request(self, user_text, context) -> GoalSpec:
        """
        Given a natural language request, ask LLM to generate a Goal Specification that will be used to determine
        whether the user's goal has been achieved.

        :param user_text: The user request in natural language.
        :param context: Additional context that may help the LLM to choose the best tool.
        :return: A GoalSpec object with the goal specification.
        :raises Exception: If there is an error during LLM inference or prompt generation.
        """
        # Build prompt with available tools
        system_prompt, user_prompt = self._build_goal_prompt(user_text, context)

        if not system_prompt or not user_prompt:
            raise Exception("Error building prompt for goal inference.")

        # Query LLM
        goal = self.llm.inference_goal(system_prompt, user_prompt, self.history)
        self.logger.log_manager(f"Goal received:\n{goal}")

        if not goal:
            raise Exception("Error: Goal inference failed to produce a valid goal.")

        return goal

    def _init_single_tool_plan(self):
        """Initialize a plan that uses a single tool to achieve the goal."""
        plan = GlobalPlan(
            summary="Single tool plan",
            plan=[
                PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        Step(
                            tool="name_of_tool",  # Placeholder tool name
                            args=[],              # Placeholder args
                            timeout_ms=self.step_timeout_ms
                        ),
                    ],
                ),
            ],
        )
        return plan

    def _set_single_tool_params(self, tool_name: str, args: List[ArgValue]):
        """Set the parameters of the single tool plan."""
        if not self._single_tool_plan or not self._single_tool_plan.plan:
            self.logger.log_manager("Single tool plan is not initialized properly.", error=True)
            return
        self._single_tool_plan.plan[0].steps[0].tool = tool_name
        self._single_tool_plan.plan[0].steps[0].args = args

    def _verify_progress(self) -> bool:
        """
        Run the verification strategy indicated by self.goal.mode to check if the goal has been achieved.
        This method is responsible for determining if the user request has been satisfied.
        """
        mode = getattr(self.goal, "mode", "objective") or "objective"

        # Run verification tools to update blackboard
        self._run_verification_tools()

        if mode == "objective":
            # Check if goal is already achieved
            if self._is_goal_achieved():
                self.logger.log_manager("Goal achieved in objective mode. Stopping iterations.")
                return True
            else:
                return False

        elif mode == "perceptual":
            # Check if goal is already achieved using AI inference
            instruction = f"Check if the user goal has been achieved based on the evidence provided. User goal: '{self.goal.summary}'"
            evidence_keys = self.goal.evidence_bb_keys
            # Gather evidence from blackboard
            evidence = {}
            images_paths = []
            for verify_tool in self.goal.verify_tools:
                # Get real tool from registry
                tool = self.registry.tools.get(verify_tool.tool)
                if tool:
                    evidence[tool.name] = self.bb.get(tool.name, {})
                # Note that provide_images is only available for ValidationTool, needs to be checked after verifying its a validation tool
                if tool and tool.is_validation_tool and tool.provide_images:
                    images_paths.extend(self.bb.get(tool.name, {}).get("images", []))
            # Build prompt to check if goal is achieved
            system_prompt, user_prompt = self._build_validation_prompt(instruction, evidence)
            self.logger.log_manager(f"Running perceptual verification with instruction: {instruction} - Evidence: {evidence_keys} - Images: {images_paths}")

            try:
                validation = self.llm.inference_validation(system_prompt, user_prompt, images_paths, self.history)
                success = bool(validation.success)
                self.bb["ai_validation"] = validation
                self.bb["goal_achieved"] = success
                if success:
                    self.logger.log_manager(f"Goal achieved in perceptual mode. Stopping iterations. - {validation}")
                    return True
                else:
                    self.logger.log_manager(f"Goal not achieved in perceptual mode. - {validation}")
                    return False
            except Exception as e:
                self.logger.log_manager(f"Error during perceptual verification: {e}", error=True)
                return False

    def _run_verification_tools(self):
        """
        Run verification tools to check if the goal has been achieved and update blackboard.
        Verification tools should only be called if a plan has been executed in this iteration or if
        it is the first iteration (to set initial state).
        """
        if self._timeline and len(self._timeline) > 0:
            last_event = self._timeline[-1].get('event', "N/A")
            if last_event not in (TimelineEvent.PLAN_EXECUTED.value, TimelineEvent.GOAL_SET.value):
                self.logger.log_manager("Skipping verification tools as no plan has been executed in this iteration.")
                return
        for verify_tool in self.goal.verify_tools:
            tool_name = verify_tool.tool
            tool_args = verify_tool.args
            tool = self.registry.tools.get(tool_name)
            if not tool:
                self.logger.log_manager(f"Verification tool '{tool_name}' not found in registry.", error=True)
                continue
            try:
                self.bb
                self._set_single_tool_params(tool_name, tool_args)
                self.logger.log_manager(f"Running verification tool: {tool_name}")
                result = self.execute_plan(self._single_tool_plan)
                if not result.get("success"):
                    self.bb[tool_name] = {"error": "Validation tool execution failed"}
                    self.logger.log_manager(f"Error running verification tool '{tool_name}'. BB entry of this tool will be empty.", error=True)
            except Exception as e:
                self.logger.log_manager(f"Error running verification tool '{tool_name}': {e}", error=True)
                continue

    def _add_to_history(self, user_text: str, plan_summary: str):
        """Override method to add interactions with timeline information."""
        last_detail = ""
        last_event = "N/A"
        if self._timeline and len(self._timeline) > 0:
            last_event = self._timeline[-1].get('event', "N/A")
            if self._timeline[-1].get('detail', None):
                last_detail = f" - Details: {self._timeline[-1].get('detail', '')}"

        history_plan = f"Plan summary: {plan_summary} - Iteration result: {last_event}{last_detail}"
        self.history.append((user_text, history_plan))
        # Keep only the last `history_depth` interactions
        if len(self.history) > self.history_depth:
            if self.history_depth <= 0:
                self.history = []
            else:
                self.history = self.history[-self.history_depth:]