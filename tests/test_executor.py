import hashlib
import importlib
import numpy as np
import os
import sys
import time
import types
import unittest


# Stub sentence_transformers to avoid heavy dependency during tests
class _DummySentenceTransformer:
    def __init__(self, *args, **kwargs):
        pass
    def encode(self, text, convert_to_numpy=True):
        return None
    def similarity(self, a, b):
        return None
sys.modules.setdefault('sentence_transformers', types.SimpleNamespace(SentenceTransformer=_DummySentenceTransformer))


# Make src-layout importable
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class TestPlanExecutor(unittest.TestCase):
    def setUp(self):
        # Import package modules dynamically
        tools_mod = importlib.import_module("vulcanai.tools")
        registry_mod = importlib.import_module("vulcanai.tool_registry")
        plan_types_mod = importlib.import_module("vulcanai.plan_types")
        executor_mod = importlib.import_module("vulcanai.executor")

        # Keep references
        self.AtomicTool = tools_mod.AtomicTool
        self.ToolRegistry = registry_mod.ToolRegistry
        self.PlanExecutor = executor_mod.PlanExecutor

        # Expose for tests
        self.GlobalPlan = plan_types_mod.GlobalPlan
        self.PlanNode = plan_types_mod.PlanNode
        self.PlanBase = plan_types_mod.PlanBase
        self.Step = plan_types_mod.Step
        self.Arg = plan_types_mod.ArgValue

        # Local dummy embedder
        class LocalDummyEmbedder:
            def embed(self, text: str) -> np.ndarray:
                h = hashlib.sha256(text.encode("utf-8")).digest()
                vec = np.frombuffer(h, dtype=np.uint8).astype(np.float32)[:64]
                norm = np.linalg.norm(vec) or 1.0
                return vec / norm
            def similarity(self, a: np.ndarray, b: np.ndarray) -> float:
                return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-8))
        self.Embedder = LocalDummyEmbedder()

        # Build registry and executor
        self.registry = self.ToolRegistry(embedder=self.Embedder)

        # Define and register tools
        class NavTool(self.AtomicTool):
            name = "go_to_pose"
            description = "Navigate robot to a target location"
            tags = ["navigation", "goal", "go to", "move"]
            input_schema = [("x", "float"), ("y", "float"), ("z", "float")]
            output_schema = {"arrived": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                print(f"Run method of NavTool called with args: {kwargs}")
                return {"arrived": True}
        class DetectTool(self.AtomicTool):
            name = "detect_object"
            description = "Detect an object in the environment"
            tags = ["vision", "perception"]
            input_schema = [("label", "string")]
            output_schema = {"found": "bool", "pose": "dict(x: float, y: float, z: float)"}
            version = "0.1"
            def run(self, **kwargs):
                return {"found": True, "pose": {"x": 4.0, "y": 2.0, "z": 0.0}}
        class SpeakTool(self.AtomicTool):
            name = "speak"
            description = "Speak a text string"
            tags = ["speech", "text"]
            input_schema = [("text", "string")]
            output_schema = {"spoken": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"spoken": True, "spoken_text": kwargs.get("text", "")}
        class ListTool(self.AtomicTool):
            name = "output_list"
            description = "Output a list of items"
            tags = ["output", "list"]
            input_schema = []
            output_schema = {"output": "list"}
            version = "0.1"
            def run(self, **kwargs):
                return {"output": ["apple", "banana", "cherry"]}
        class SleepTool(self.AtomicTool):
            name = "sleep"
            description = "Sleep for a specified duration"
            tags = ["sleep"]
            input_schema = [("duration", "int")]
            output_schema = {"slept": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                time.sleep(kwargs.get("duration", 1))
                return {"slept": True}
        class FlakyTool(self.AtomicTool):
            name = "flaky"
            description = "A tool that fails a few times before succeeding"
            tags = ["flaky"]
            input_schema = []
            output_schema = {"succeeded": "bool"}
            version = "0.1"
            def __init__(self):
                super().__init__()
                self.attempts = 0
            def run(self, **kwargs):
                self.attempts += 1
                if self.attempts < 3:
                    raise RuntimeError("Simulated failure")
                return {"succeeded": True}
        # This tool can be instantiated directly to allow resetting attempts
        self.FlakyTool = FlakyTool

        class CriteriaTool(self.AtomicTool):
            name = "criteria_tool"
            description = "A tool that returns a numeric value"
            tags = ["criteria"]
            input_schema = []
            output_schema = {"value": "int"}
            version = "0.1"
            def __init__(self, return_value):
                super().__init__()
                self.return_value = return_value
            def run(self, **kwargs):
                return {"value": self.return_value}
        # This tool can be instantiated directly to allow modifying returned value
        self.CriteriaTool = CriteriaTool
        class AddTool(self.AtomicTool):
            name = "add"
            description = "Adds two numbers together."
            input_schema = [("a", "float"), ("b", "float")]
            output_schema = {"result": "float"}

            def run(self, a: float, b: float):
                return {"result": a + b}

        self.registry.register_tool(AddTool())
        self.registry.register_tool(NavTool())
        self.registry.register_tool(DetectTool())
        self.registry.register_tool(SpeakTool())
        self.registry.register_tool(ListTool())
        self.registry.register_tool(SleepTool())

        self.exec = self.PlanExecutor(self.registry)

    def test_simple_plan_executes(self):
        """
        Test that:
        - A simple plan with all steps executes successfully.
        - Blackboard contains expected entries after execution.
        - Blackboard values can be evaluated correctly during execution as input args.
        """
        plan = self.GlobalPlan(
            summary="Navigate to a location, detect an object, and speak the result",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="detect_object", args=[self.Arg(key="label", val="mug")]),
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val="{{bb.detect_object.pose.x}}"), self.Arg(key="y", val="{{bb.detect_object.pose.y}}"), self.Arg(key="z", val="{{bb.detect_object.pose.z}}")]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="I have arrived and detected a mug.")]),
                    ],
                )
            ],
        )

        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("detect_object", bb)
        self.assertIn("go_to_pose", bb)
        self.assertIn("speak", bb)
        self.assertTrue(bb["detect_object"]["found"])
        self.assertTrue(bb["go_to_pose"]["arrived"])
        self.assertEqual(bb["detect_object"]["pose"], {"x": 4.0, "y": 2.0, "z": 0.0})

    def test_false_condition_skips_step(self):
        """Test that a step with a False condition is skipped and a non-existing blackboard entry does not return error."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Hello")], condition="{{bb.missing_flag}} == True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertNotIn("speak", bb)

    def test_true_condition_executes_step(self):
        """Test that a step with a True condition executes."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Hello")], condition="True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])

    def test_bb_condition_evaluation(self):
        """Test blackboard condition evaluation."""
        # Test with bb condition at the beggining of string condition and a False condition to skip
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="detect_object", args=[self.Arg(key="label", val="book")]),
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val="{{bb.detect_object.pose.x}}"), self.Arg(key="y", val="{{bb.detect_object.pose.y}}"), self.Arg(key="z", val="{{bb.detect_object.pose.z}}")], condition="{{bb.detect_object.found}} == True"),
                        # Skip next step by making condition False
                        self.Step(tool="speak", args=[self.Arg(key="text", val="I have arrived.")], condition="{{bb.go_to_pose.arrived}} != True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("detect_object", bb)
        self.assertTrue(bb["detect_object"]["found"])
        self.assertEqual(bb["detect_object"]["pose"], {"x": 4.0, "y": 2.0, "z": 0.0})
        self.assertIn("go_to_pose", bb)
        self.assertTrue(bb["go_to_pose"]["arrived"])
        self.assertNotIn("speak", bb)  # Step was skipped

        # Test with bb condition at the end of string condition
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="detect_object", args=[self.Arg(key="label", val="book")]),
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val="{{bb.detect_object.pose.x}}"), self.Arg(key="y", val="{{bb.detect_object.pose.y}}"), self.Arg(key="z", val="{{bb.detect_object.pose.z}}")], condition="True == {{bb.detect_object.found}}"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("detect_object", bb)
        self.assertTrue(bb["detect_object"]["found"])
        self.assertIn("go_to_pose", bb)
        self.assertTrue(bb["go_to_pose"]["arrived"])

        # Test with comparison of two bb fields as condition
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="detect_object", args=[self.Arg(key="label", val="book")]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="I have arrived.")]),
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val="{{bb.detect_object.pose.x}}"), self.Arg(key="y", val="{{bb.detect_object.pose.y}}"), self.Arg(key="z", val="{{bb.detect_object.pose.z}}")], condition="{{bb.speak.spoken}} == {{bb.detect_object.found}}"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("detect_object", bb)
        self.assertTrue(bb["detect_object"]["found"])
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])
        self.assertIn("go_to_pose", bb)
        self.assertTrue(bb["go_to_pose"]["arrived"])

        # Test substitution of an element of a list in the blackboard
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="output_list", args=[]),
                        self.Step(tool="speak",
                            args=[self.Arg(key="text", val="First item is {{bb.output_list.output[0]}}")],
                        ),
                    ],
                )
            ],
        )

        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("output_list", bb)
        self.assertEqual(bb["output_list"]["output"], ["apple", "banana", "cherry"])
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])
        self.assertEqual(bb["speak"]["spoken_text"], "First item is apple")

    def test_parallel_node_execution(self):
        """Test that a parallel PlanNode executes all steps and collects their results."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="PARALLEL",
                    steps=[
                        self.Step(tool="sleep", args=[self.Arg(key="duration", val=1)]),
                        self.Step(tool="sleep", args=[self.Arg(key="duration", val=2)]),
                        self.Step(tool="sleep", args=[self.Arg(key="duration", val=3)]),
                    ],
                )
            ],
        )

        start = time.time()
        result = self.exec.run(plan, {})
        end = time.time()
        elapsed = end - start
        # Check that total time is just over the longest sleep (3s) not the sum (6s)
        self.assertLessEqual(elapsed, 4)
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("sleep", bb)
        self.assertTrue(bb["sleep"]["slept"])

    def test_step_with_retries(self):
        """Test that a step with retries is attempted the specified number of times upon failure."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="flaky", args=[], retry=1),
                    ],
                )
            ],
        )

        r = self.ToolRegistry(embedder=self.Embedder)
        flaky_tool = self.FlakyTool()
        r.register_tool(flaky_tool)
        flaky_exec = self.PlanExecutor(r)

        # Fail after 1 retry (2 attempts total)
        result = flaky_exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("flaky", bb)
        self.assertFalse(bb["flaky"])
        # Check that the step was retried
        self.assertEqual(flaky_tool.attempts, 2)

        # Success after 2 retries (3 attempts total)
        flaky_tool.attempts = 0  # Reset attempts
        plan.plan[0].steps[0].retry = 2
        result = flaky_exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("flaky", bb)
        self.assertTrue(bb["flaky"]["succeeded"])
        # Check that the step was retried
        self.assertEqual(flaky_tool.attempts, 3)

    def test_step_with_criteria(self):
        """Test that a step with success criteria evaluates output correctly."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="criteria_tool", args=[], success_criteria="{{bb.criteria_tool.value}} >= 10"),
                    ],
                )
            ],
        )

        r = self.ToolRegistry(embedder=self.Embedder)
        # First test with a value that does not meet criteria
        r.register_tool(self.CriteriaTool(return_value=5))
        criteria_exec = self.PlanExecutor(r)

        result = criteria_exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("criteria_tool", bb)
        self.assertEqual(bb["criteria_tool"]["value"], 5)

        # Now test with a value that meets criteria
        r = self.ToolRegistry(embedder=self.Embedder)
        r.register_tool(self.CriteriaTool(return_value=15))
        criteria_exec = self.PlanExecutor(r)

        result = criteria_exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("criteria_tool", bb)
        self.assertEqual(bb["criteria_tool"]["value"], 15)

    def test_step_with_timeout(self):
        """Test that a step with a timeout fails if execution exceeds the limit."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="sleep", args=[self.Arg(key="duration", val=3)], timeout_ms=2000),
                    ],
                )
            ],
        )

        # Test with timeout that causes failure
        result = self.exec.run(plan, {})
        self.assertFalse(result["success"])
        # Increase timeout and test for success
        plan.plan[0].steps[0].timeout_ms = 4000
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])

    def test_plan_node_with_false_condition(self):
        """Test that a PlanNode with a False condition is skipped."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="This should be skipped")]),
                    ],
                    condition="False",
                )
            ],
        )

        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertNotIn("speak", bb)  # Step was skipped

    def test_plan_node_with_true_condition(self):
        """Test that a PlanNode with a True condition executes."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="This should be spoken")]),
                    ],
                    condition="True",
                )
            ],
        )

        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])

    def test_plan_node_with_criteria(self):
        """Test that a PlanNode with success criteria evaluates output correctly."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="criteria_tool", args=[]),
                    ],
                    success_criteria="{{bb.criteria_tool.value}} >= 10",
                )
            ],
        )

        r = self.ToolRegistry(embedder=self.Embedder)
        # First test with a value that does not meet criteria
        r.register_tool(self.CriteriaTool(return_value=5))
        criteria_exec = self.PlanExecutor(r)

        result = criteria_exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("criteria_tool", bb)
        self.assertEqual(bb["criteria_tool"]["value"], 5)

        # Now test with a value that meets criteria
        r = self.ToolRegistry(embedder=self.Embedder)
        r.register_tool(self.CriteriaTool(return_value=15))
        criteria_exec = self.PlanExecutor(r)

        result = criteria_exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("criteria_tool", bb)
        self.assertEqual(bb["criteria_tool"]["value"], 15)

    def test_plan_node_with_retries(self):
        """Test that a PlanNode with retries is attempted the specified number of times upon failure."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="flaky", args=[]),
                    ],
                    retry=1,
                )
            ],
        )

        r = self.ToolRegistry(embedder=self.Embedder)
        flaky_tool = self.FlakyTool()
        r.register_tool(flaky_tool)
        flaky_exec = self.PlanExecutor(r)

        # Fail after 1 retry (2 attempts total)
        result = flaky_exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("flaky", bb)
        self.assertFalse(bb["flaky"])
        # Check that the step was retried
        self.assertEqual(flaky_tool.attempts, 2)

        # Success after 2 retries (3 attempts total)
        print("Resetting flaky tool attempts and increasing PlanNode retries")
        flaky_tool.attempts = 0  # Reset attempts
        plan.plan[0].retry = 2
        result = flaky_exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("flaky", bb)
        self.assertTrue(bb["flaky"]["succeeded"])
        # Check that the step was retried
        self.assertEqual(flaky_tool.attempts, 3)

    def test_plan_node_with_timeout(self):
        """Test that a PlanNode with a timeout fails if execution exceeds the limit."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="sleep", args=[self.Arg(key="duration", val=3)]),
                    ],
                    timeout_ms=2000,
                )
            ],
        )

        # Test with timeout that causes failure
        result = self.exec.run(plan, {})
        self.assertFalse(result["success"])
        # Increase timeout and test for success
        plan.plan[0].timeout_ms = 4000
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])

    def test_plan_node_with_unknown_kind(self):
        """Test that a PlanNode with an unknown kind is skipped with a warning."""
        try:
            plan = self.GlobalPlan(
                plan=[
                    self.PlanNode(
                        kind="UNKNOWN",
                        steps=[
                            self.Step(tool="speak", args=[self.Arg(key="text", val="This should be skipped")]),
                        ],
                    )
                ],
            )
            self.fail("PlanNode creation should have failed with ValueError")
        except Exception as e:
            pass

    def test_plan_node_with_on_fail(self):
        """Test that a PlanNode with on_fail executes the on_fail node upon failure."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="criteria_tool", args=[]),
                    ],
                    success_criteria="{{bb.criteria_tool.value}} >= 10",
                    on_fail=self.PlanBase(
                        kind="SEQUENCE",
                        steps=[
                            self.Step(tool="speak", args=[self.Arg(key="text", val="The criteria tool failed.")]),
                        ],
                    ),
                )
            ],
        )

        # Register criteria tool that fails the criteria
        self.registry.register_tool(self.CriteriaTool(return_value=5))
        on_fail_exec = self.PlanExecutor(self.registry)

        result = on_fail_exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("criteria_tool", bb)
        self.assertEqual(bb["criteria_tool"]["value"], 5)
        # Check that the on_fail step executed
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])

    def test_plan_stops_if_middle_step_fails(self):
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Hello")]),
                        self.Step(tool="flaky", args=[], retry=0),  # This will fail
                        self.Step(tool="speak", args=[self.Arg(key="text", val="This should not be spoken")]),
                    ],
                )
            ],
        )

        self.registry.register_tool(self.FlakyTool())
        exec = self.PlanExecutor(self.registry)

        result = exec.run(plan, {})
        self.assertFalse(result["success"])
        bb = result["blackboard"]
        self.assertIn("flaky", bb)
        self.assertFalse(bb["flaky"])
        # Check that the second step was not executed
        self.assertIn("speak", bb)
        self.assertEqual(bb["speak"]["spoken_text"], "Hello")

    def test_tools_can_access_user_bb_elements(self):
        """Test that tools can access user-provided blackboard elements."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="speak", args=[self.Arg(key="text", val="{{bb.user_topic}}")]),
                    ],
                )
            ],
        )

        user_bb = {"user_topic": "VulcanAI"}
        result = self.exec.run(plan, bb=user_bb)
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])
        self.assertEqual(bb["speak"]["spoken_text"], "VulcanAI")

    def test_types_substitution_is_respected(self):
        """Test that when a bb substitution is used, the types are respected."""
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="criteria_tool", args=[]),  # Outputs 5
                        self.Step(tool="add", args=[self.Arg(key="a", val='{{bb.criteria_tool.value}}'), self.Arg(key="b", val=3.5)]),
                    ],
                )
            ],
        )

        criteria_tool = self.CriteriaTool(return_value=5)
        self.registry.register_tool(criteria_tool)
        result = self.exec.run(plan, {})
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("add", bb)
        self.assertEqual(bb["add"]["result"], 8.5)
        self.assertTrue(isinstance(bb["add"]["result"], float))


if __name__ == "__main__":
    unittest.main()
