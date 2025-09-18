import unittest
import os
import sys
import hashlib
import numpy as np
import types
import importlib


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
        GlobalPlan = plan_types_mod.GlobalPlan
        PlanNode = plan_types_mod.PlanNode
        Step = plan_types_mod.Step
        self.PlanExecutor = executor_mod.PlanExecutor

        # Expose for tests
        self.GlobalPlan = GlobalPlan
        self.PlanNode = PlanNode
        self.Step = Step

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
            input_schema = {"pose": "dict(x: float, y: float, z: float)"}
            output_schema = {"arrived": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"arrived": True}
        class DetectTool(self.AtomicTool):
            name = "detect_object"
            description = "Detect an object in the environment"
            tags = ["vision", "perception"]
            input_schema = {"label": "string"}
            output_schema = {"found": "bool", "pose": "dict(x: float, y: float, z: float)"}
            version = "0.1"
            def run(self, **kwargs):
                return {"found": True, "pose": {"x": 4.0, "y": 2.0, "z": 0.0}}
        class SpeakTool(self.AtomicTool):
            name = "speak"
            description = "Speak a text string"
            tags = ["speech", "text"]
            input_schema = {"text": "string"}
            output_schema = {"spoken": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"spoken": True}

        self.registry.register_tool(NavTool())
        self.registry.register_tool(DetectTool())
        self.registry.register_tool(SpeakTool())

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
                        self.Step(tool="go_to_pose", args={"pose": {"x": 1.0, "y": 2.0, "z": 0.0}}, timeout_ms=5000),
                        self.Step(tool="detect_object", args={"label": "mug"}, timeout_ms=3000),
                        self.Step(tool="go_to_pose", args={"pose": "{{bb.detect_object.pose}}"}, timeout_ms=5000),
                        self.Step(tool="speak", args={"text": "I have arrived and detected a mug."}, timeout_ms=2000),
                    ],
                )
            ],
        )

        result = self.exec.run(plan)
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
                        self.Step(tool="speak", args={"text": "Hello"}, condition="{{bb.missing_flag}} == True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan)
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
                        self.Step(tool="speak", args={"text": "Hello"}, condition="True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan)
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
                        self.Step(tool="detect_object", args={"label": "book"}),
                        self.Step(tool="go_to_pose", args={"pose": "{{bb.detect_object.pose}}"}, condition="{{bb.detect_object.found}} == True"),
                        # Skip next step by making condition False
                        self.Step(tool="speak", args={"text": "I have arrived."}, condition="{{bb.go_to_pose.arrived}} != True"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan)
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
                        self.Step(tool="detect_object", args={"label": "book"}),
                        self.Step(tool="go_to_pose", args={"pose": "{{bb.detect_object.pose}}"}, condition="True == {{bb.detect_object.found}}"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan)
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
                        self.Step(tool="detect_object", args={"label": "book"}),
                        self.Step(tool="speak", args={"text": "I have arrived."}),
                        self.Step(tool="go_to_pose", args={"pose": "{{bb.detect_object.pose}}"}, condition="{{bb.speak.spoken}} == {{bb.detect_object.found}}"),
                    ],
                )
            ]
        )
        result = self.exec.run(plan)
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("detect_object", bb)
        self.assertTrue(bb["detect_object"]["found"])
        self.assertIn("speak", bb)
        self.assertTrue(bb["speak"]["spoken"])
        self.assertIn("go_to_pose", bb)
        self.assertTrue(bb["go_to_pose"]["arrived"])

    def test_parallel_node_execution(self):
        """Test that a parallel PlanNode executes all steps and collects their results."""
        import time
        class SleepTool(self.AtomicTool):
            name = "sleep"
            description = "Sleep for a specified duration"
            tags = ["sleep"]
            input_schema = {"duration": "int"}
            output_schema = {"slept": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                time.sleep(kwargs.get("duration", 1))
                return {"slept": True}
        plan = self.GlobalPlan(
            plan=[
                self.PlanNode(
                    kind="PARALLEL",
                    steps=[
                        self.Step(tool="sleep", args={"duration": 1}, timeout_ms=5000),
                        self.Step(tool="sleep", args={"duration": 2}, timeout_ms=5000),
                        self.Step(tool="sleep", args={"duration": 3}, timeout_ms=5000),
                    ],
                )
            ],
        )

        r = self.ToolRegistry(embedder=self.Embedder)
        r.register_tool(SleepTool())
        sleep_exec = self.PlanExecutor(r)

        start = time.time()
        result = sleep_exec.run(plan)
        end = time.time()
        elapsed = end - start
        # Check that total time is just over the longest sleep (3s) not the sum (6s)
        self.assertLessEqual(elapsed, 4)
        self.assertTrue(result["success"])
        bb = result["blackboard"]
        self.assertIn("sleep", bb)
        self.assertTrue(bb["sleep"]["slept"])


if __name__ == "__main__":
    unittest.main()
