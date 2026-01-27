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

import pytest

from vulcanai import (
    ArgValue,
    AtomicTool,
    GlobalPlan,
    IterativeManager,
    PlanNode,
    Step,
    TimelineEvent,
    ValidationTool,
    VulcanAILogger,
)


class DummyTool(AtomicTool):
    def __init__(self, name, description="desc", input_schema=None, output_schema=None, output="result"):
        self.name = name
        self.description = description
        self.input_schema = input_schema or []
        self.output_schema = output_schema or []
        self.output = output

    def run(self, **kwargs):
        return {"result": self.output}

    def change_output(self, new_output):
        self.output = new_output


class DummyValidationTool(ValidationTool):
    def __init__(self, name, description="desc", input_schema=None, output_schema=None, output="result"):
        self.name = name
        self.description = description
        self.input_schema = input_schema or []
        self.output_schema = output_schema or []
        self.output = output

    def run(self, **kwargs):
        return {"validation_result": self.output}

    def change_output(self, new_output):
        self.output = new_output


class FakeStepBase:
    """Replica of StepBase class with tool name and args."""

    def __init__(self, tool: str, args: list[ArgValue]):
        self.tool = tool
        self.args = args


class DummyImageValidationTool(ValidationTool):
    def __init__(self, name, description="desc", input_schema=None, output_schema=None, output="result"):
        self.name = name
        self.description = description
        self.input_schema = input_schema or []
        self.output_schema = output_schema or []
        self.output = output
        self.provide_images = True

    def run(self, **kwargs):
        return {"validation_result": True, "images": self.output}

    def change_output(self, new_output):
        self.output = new_output


class MockGoalSpec:
    def __init__(
        self, summary="goal", mode="objective", success_predicates=None, verify_tools=None, evidence_bb_keys=None
    ):
        self.summary = summary
        self.mode = mode
        self.success_predicates = success_predicates or []
        self.verify_tools: list[FakeStepBase] = verify_tools or []
        self.evidence_bb_keys = evidence_bb_keys or []

    def __str__(self) -> str:
        lines = []
        if self.summary:
            lines.append(f"- Mock Goal Summary: {self.summary}\n")
        if self.mode:
            lines.append(f"- Mock Verification Mode: {self.mode}\n")

        for i, pred in enumerate(self.success_predicates, 1):
            lines.append(f"- Success Predicate {i}: {pred}")
        for i, tool in enumerate(self.verify_tools, 1):
            lines.append(f"- Verify Tool {i}: {tool.tool}({tool.args})")
        if self.evidence_bb_keys:
            lines.append(f"- Mock Evidence BB Keys: {self.evidence_bb_keys}")
        return "\n".join(lines)


class MockAIValidation:
    def __init__(self, success=True, confidence=1.0, explanation=""):
        self.success: bool = success
        self.confidence: float = confidence
        self.explanation: str = explanation


class FakeRegistry:
    """Mimics ToolRegistry enough for top_k() and tools{} lookups."""

    def __init__(self, tools=None):
        self.tools = {t.name: t for t in (tools or [])}

    def top_k(self, user_text, k, validation=False):
        # For simplicity, return all tools; real logic not needed for manager tests
        return list(self.tools.values())


class MockAgent:
    """Mock agent that records prompts passed to inference_plan() and inference_goal()."""

    def __init__(self, plans=[], goal=None, validation=None, success_validation=0, logger=None):
        """
        :param plans: list[GlobalPlan] to return on successive inference_plan() calls
        :param goal:  GoalSpec to return on inference_goal() calls
        """
        self.plans = list(plans)
        self.goal = goal
        self.validation = validation
        # List of dicts: {"system": str, "user": str, "images": list, "history": list}
        self.inference_calls = []
        # Same for inference_goal
        self.goal_calls = []
        # Same for inference_validation. Inference validation is only called if goal is 'perceptual'
        self.validation_calls = []
        # If >0, number of successive validation calls needed to return success=True.
        # Any call before that returns success = False. First call is 0.
        self.success_validation = success_validation

    def inference_plan(self, system_prompt, user_prompt, images, history):
        self.inference_calls.append(
            {"system": system_prompt, "user": user_prompt, "images": list(images), "history": list(history)}
        )
        # Pop next prepared plan
        if not self.plans:
            return None
        return self.plans.pop(0)

    def inference_goal(self, system_prompt, user_prompt, history):
        self.goal_calls.append({"system": system_prompt, "user": user_prompt, "history": list(history)})
        return self.goal

    def inference_validation(self, system_prompt, user_prompt, images, history):
        self.validation_calls.append(
            {"system": system_prompt, "user": user_prompt, "images": list(images), "history": list(history)}
        )
        if self.success_validation > 0:
            self.success_validation -= 1
            self.validation.success = False
        else:
            self.validation.success = True
        return self.validation


def make_single_step_plan(summary="plan", tool="dummy_tool", key="arg", val="x", timeout_ms=None):
    return GlobalPlan(
        summary=summary,
        plan=[
            PlanNode(
                kind="SEQUENCE",
                steps=[
                    Step(tool=tool, args=[ArgValue(key=key, val=val)], timeout_ms=timeout_ms),
                ],
            )
        ],
    )


#################
### Fixtures
#################


class ListSink:
    def __init__(self):
        self.lines = []  # list[str]

    def write(self, msg: str, color: str = "") -> None:
        self.lines.append(msg)


@pytest.fixture
def logger():
    sink = ListSink()
    log = VulcanAILogger(sink=sink)
    log.set_sink(sink)
    return log


@pytest.fixture(autouse=True)
def patch_core_symbols(monkeypatch):
    """Patch the exact Agent to avoid doing real inferences."""
    target_mod = "vulcanai.core.manager"

    # Patch Agent so ToolManager(self.llm = Agent(...)) doesn't spin a real model
    monkeypatch.setattr(f"{target_mod}.Agent", MockAgent, raising=True)


@pytest.fixture
def base_manager(logger, monkeypatch):
    # Build a minimal IterativeManager with mocked dependencies. The agent will be pathed
    dummy_tool1 = DummyTool("dummy_tool_1", input_schema=[("arg", "str")], output="out1")
    dummy_tool2 = DummyTool("dummy_tool_2", input_schema=[("id", "str")], output="out2")
    dummy_validation = DummyValidationTool("dummy_validation", input_schema=[("id", "str")], output="not_valid")
    dummy_image_validation = DummyImageValidationTool("dummy_image_validation", output=["img1.png", "img2.png"])
    mgr = IterativeManager(
        model="fake-model",
        registry=FakeRegistry([dummy_tool1, dummy_tool2, dummy_validation, dummy_image_validation]),
        validator=None,
        k=5,
        hist_depth=5,
        logger=logger,
        max_iters=3,
        step_timeout_ms=1200,
    )
    # Allow extra hooks to be set for testing
    mgr._after_execute_hooks = []
    # Initialize blackboard iteration counter
    mgr.bb["iteration"] = 0
    original_execute_plan = mgr.execute_plan

    # Each plan execution advances iteration by 1 and "fails" to force another iteration
    def exec_wrapper(_plan):
        ret = original_execute_plan(_plan)
        # Advance iteration count on BB after executing the plan
        mgr.bb["iteration"] += 1
        for h in mgr._after_execute_hooks:
            h(_plan, ret)
        return ret

    monkeypatch.setattr(mgr, "execute_plan", exec_wrapper, raising=True)
    return mgr


#################
### Tests
#################


def test_prompts_reflect_bb_updates_across_iterations(base_manager):
    """
    On iteration N+1, the user prompt must include the updated blackboard snapshot
    produced by executing the plan in iteration N.
    This means that the inference call of iter N+1 must contain the outputs of tools run in iter N because the
    blackboard state is carried over between iterations to help the LLM understand the context.
    """
    mgr = base_manager

    # 1) LLM goal: use a predicate based on the iteration counter
    goal = MockGoalSpec(
        summary="test blackboard is correctly reflected in prompts",
        success_predicates=["{{bb.iteration}} >= 2"],
        verify_tools=[],
    )

    # 2) Plans the LLM will return for two iterations
    plan1 = make_single_step_plan(summary="first", tool="dummy_tool_1", key="arg", val="val1")
    plan2 = make_single_step_plan(summary="second", tool="dummy_tool_2", key="id", val="val2")

    # 3) Update MockAgent to return these plans and goal
    mgr.llm.plans = [plan1, plan2]
    mgr.llm.goal = goal

    # Run
    result = mgr.handle_user_request("This is a test and this prompt is not relevant", context={})
    assert "timeline" in result

    # Ensure inference_plan() calls were made (we expect a goal generation and two iterations planning)
    assert len(mgr.llm.goal_calls) == 1
    assert len(mgr.llm.inference_calls) == 2

    # Check that the **second** user prompt contains the BB update from the **first** execution
    first_user_prompt = mgr.llm.inference_calls[0]["user"]
    second_user_prompt = mgr.llm.inference_calls[1]["user"]

    assert "Current blackboard state" in first_user_prompt
    assert "Current blackboard state" in second_user_prompt

    # After iter 1 execution, BB should contain dummy_tool result and must show up in iter 2 prompt
    assert "dummy_tool_1" not in first_user_prompt
    assert "dummy_tool_1" in second_user_prompt
    assert "out1" in second_user_prompt  # Output from dummy_tool_1 run in iter 1
    assert "out2" not in second_user_prompt  # Output from dummy_tool_2


def test_validation_tools_are_called_before_each_iteration(base_manager):
    """
    Test that validation tools are called before each iteration.
    """
    mgr = base_manager

    # 1) LLM goal: use a predicate based on the iteration counter
    goal = MockGoalSpec(
        summary="test validation tools are called",
        mode="objective",
        success_predicates=[
            "{{bb.iteration}} >= 4"
        ],  # As we have validation tool, two bb.iterations are called per plan iteration (validation and plan)
        verify_tools=[FakeStepBase("dummy_validation", args=[])],
    )

    # 2) Plans the LLM will return for two iterations
    plan1 = make_single_step_plan(summary="first", tool="dummy_tool_1", key="arg", val="val1")
    plan2 = make_single_step_plan(summary="second", tool="dummy_tool_2", key="id", val="val2")

    # 3) Update MockAgent to return these plans and goal
    mgr.llm.plans = [plan1, plan2]
    mgr.llm.goal = goal

    # Capture plans passed to execute_plan to check if the timeout was applied
    captured_plans = []

    def capture_plan_hook(plan, exec_result):
        import copy

        captured_plans.append(copy.deepcopy(plan))

    mgr._after_execute_hooks.append(capture_plan_hook)

    # Run
    result = mgr.handle_user_request("This is a test and this prompt is not relevant", context={})
    assert "timeline" in result

    # Ensure inference_plan() calls were made (we expect a goal generation and two iterations planning)
    assert len(mgr.llm.goal_calls) == 1
    assert len(mgr.llm.inference_calls) == 2
    assert len(mgr.llm.validation_calls) == 0  # No calls in 'objective' mode

    # First call: verification tool is called before any plan generation to check if goal is already achieved
    step1 = captured_plans[0].plan[0].steps[0]
    assert step1.tool == "dummy_validation"
    # Second call: plan1 is executed
    step2 = captured_plans[1].plan[0].steps[0]
    assert step2.tool == "dummy_tool_1"
    # Third call: verification tool is called again before plan2 generation
    step3 = captured_plans[2].plan[0].steps[0]
    assert step3.tool == "dummy_validation"
    # Fourth call: plan2 is executed
    step4 = captured_plans[3].plan[0].steps[0]
    assert step4.tool == "dummy_tool_2"


def test_validation_tools_providing_images_are_handled_in_perceptual(base_manager):
    """
    Test that validation tools that provide images are correctly handled and their images
    are passed to the next inference_plan() call if the goal mode is 'perceptual'.
    1) The validation tool must be called before each iteration.
    2) If the goal mode is 'perceptual', the images provided by the validation tool must be passed to the next
       inference_plan() call.
    """
    mgr = base_manager

    # 1) LLM goal: use a predicate based on the iteration counter
    goal = MockGoalSpec(
        summary="test validation tools providing images are correctly handled",
        mode="perceptual",
        success_predicates=[
            "{{bb.iteration}} >= 4"
        ],  # As we have validation tool, two bb.iterations are called per plan iteration (validation and plan)
        verify_tools=[FakeStepBase("dummy_image_validation", args=[])],
    )

    # 2) AIValidation result to be returned by the mock validation inference
    validation = MockAIValidation(success=False, confidence=0.9, explanation="All good in this test")

    # 2) Plans the LLM will return for two iterations
    plan1 = make_single_step_plan(summary="first", tool="dummy_tool_1", key="arg", val="val1")
    plan2 = make_single_step_plan(summary="second", tool="dummy_tool_2", key="id", val="val2")

    # 3) Update MockAgent to return these plans and goal
    mgr.llm.plans = [plan1, plan2]
    mgr.llm.goal = goal
    mgr.llm.success_validation = 2  # Two validation calls will return success=False before success=True
    mgr.llm.validation = validation

    # Run
    result = mgr.handle_user_request("This is a test and this prompt is not relevant", context={})
    assert "timeline" in result

    # Ensure inference_plan() calls were made (we expect a goal generation and two iterations planning)
    assert len(mgr.llm.goal_calls) == 1
    assert len(mgr.llm.inference_calls) == 2
    assert len(mgr.llm.validation_calls) == 3

    # Check that validation inference calls contain the images provided by the validation tool
    first_call = mgr.llm.validation_calls[0]
    second_call = mgr.llm.validation_calls[1]
    third_call = mgr.llm.validation_calls[2]

    assert first_call["images"] == ["img1.png", "img2.png"]
    assert second_call["images"] == ["img1.png", "img2.png"]
    assert third_call["images"] == ["img1.png", "img2.png"]


def test_iterative_manager_applies_timeout_to_all_steps(base_manager):
    """
    Iterative manager allows to control the timeout of all steps executed by this manager.
    """
    # Base manager has step_timeout_ms=1200
    mgr = base_manager

    # 1) LLM goal: use a predicate based on the iteration counter
    goal = MockGoalSpec(
        summary="test timeouts are correctly applied",
        success_predicates=["{{bb.iteration}} >= 2"],
        verify_tools=[],
    )

    # 2) Plans the LLM will return for two iterations
    plan1 = make_single_step_plan(summary="first", tool="dummy_tool_1", key="arg", val="val1")
    plan2 = make_single_step_plan(summary="second", tool="dummy_tool_2", key="id", val="val2", timeout_ms=9000)

    # 3) Update MockAgent to return these plans and goal
    mgr.llm.plans = [plan1, plan2]
    mgr.llm.goal = goal

    # Capture plans passed to execute_plan to check if the timeout was applied
    captured_plans = []

    def capture_plan_hook(plan, exec_result):
        import copy

        captured_plans.append(copy.deepcopy(plan))

    mgr._after_execute_hooks.append(capture_plan_hook)

    # Run
    result = mgr.handle_user_request("This is a test and this prompt is not relevant", context={})
    assert "timeline" in result

    # Ensure inference_plan() calls were made (we expect a goal generation and two iterations planning)
    assert len(mgr.llm.goal_calls) == 1
    assert len(mgr.llm.inference_calls) == 2

    # Check timeouts normalized on both plans before execution
    # First call: plan1
    step1 = captured_plans[0].plan[0].steps[0]
    assert step1.timeout_ms == mgr.step_timeout_ms
    # Second call: plan2 (was 9000, must be clamped to mgr.step_timeout_ms)
    step2 = captured_plans[1].plan[0].steps[0]
    assert step2.timeout_ms == mgr.step_timeout_ms


def test_repeated_plan_is_detected(base_manager):
    """
    If the LLM returns the same plan (string representation), the manager must mark PLAN_REPEATED
    and stop iterating.
    """
    # Base manager has step_timeout_ms=1200
    mgr = base_manager

    # 1) LLM goal: use a predicate based on the iteration counter
    goal = MockGoalSpec(
        summary="test that execution stops on repeated plan",
        # Iterate more than 2 to ensure failure if plan repeated is not detected
        success_predicates=["{{bb.iteration}} >= 5"],
        verify_tools=[],
    )

    # 2) Same plan twice. Only summary is modified as it is not taking into account for repetition detection
    # Execution will fail if more than two iterations are done, as no plan will be available in the mock
    plan1 = make_single_step_plan(summary="first", tool="dummy_tool_1", key="arg", val="val1")
    plan2 = make_single_step_plan(summary="second", tool="dummy_tool_1", key="arg", val="val1")

    # 3) Update MockAgent to return these plans and goal
    mgr.llm.plans = [plan1, plan2]
    mgr.llm.goal = goal

    # Run
    result = mgr.handle_user_request("This is a test and this prompt is not relevant", context={})
    assert "timeline" in result

    # Ensure a PLAN_REPEATED event exists
    assert any(e["event"] == TimelineEvent.PLAN_REPEATED.value for e in result["timeline"]), (
        f"Timeline: {result['timeline']}"
    )
