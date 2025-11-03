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

import hashlib
import importlib
import numpy as np
import os
import sys
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


class TestPlanValidator(unittest.TestCase):
    def setUp(self):
        # Import package modules dynamically
        tools_mod = importlib.import_module("vulcanai.tools.tools")
        registry_mod = importlib.import_module("vulcanai.tools.tool_registry")
        plan_types_mod = importlib.import_module("vulcanai.core.plan_types")
        validator_mod = importlib.import_module("vulcanai.core.validator")

        # Keep references
        self.AtomicTool = tools_mod.AtomicTool
        self.ToolRegistry = registry_mod.ToolRegistry
        self.Validator = validator_mod.PlanValidator

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
        class EmptyTool(self.AtomicTool):
            name = "empty"
            description = "Empty tool."
            input_schema = []
            output_schema = {"result": "bool"}

            def run(self):
                return {"result": True}
        class DetectTool(self.AtomicTool):
            name = "detect_object"
            description = "Detect an object in the environment"
            tags = ["vision", "perception"]
            input_schema = [("label", "string")]
            output_schema = {"found": "bool", "pose": "dict(x: float, y: float, z: float)"}
            version = "0.1"
            def run(self, **kwargs):
                return {"found": True, "pose": {"x": 4.0, "y": 2.0, "z": 0.0}}
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
        class SpeakTool(self.AtomicTool):
            name = "speak"
            description = "Speak a text string"
            tags = ["speech", "text"]
            input_schema = [("text", "string")]
            output_schema = {"spoken": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"spoken": True, "spoken_text": kwargs.get("text", "")}

        class TypesTool(self.AtomicTool):
            name = "types"
            description = "Get the types of the non-string input arguments"
            tags = ["type", "input"]
            input_schema = [
                ("int", "int"),
                ("integer", "integer"),
                ("float", "float"),
                ("bool", "bool"),
                ("boolean", "boolean"),
            ]
            output_schema = {"types": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"types": True}

        self.registry.register_tool(EmptyTool())
        self.registry.register_tool(DetectTool())
        self.registry.register_tool(NavTool())
        self.registry.register_tool(SpeakTool())
        self.registry.register_tool(TypesTool())

        self.validator = self.Validator(self.registry)

    def test_validator_correct_plan(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location, detect an object, and speak the result",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="detect_object", args=[self.Arg(key="label", val="mug")]),
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=3.0), self.Arg(key="y", val=4.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="I have arrived and detected a mug.")]),
                    ],
                )
            ],
        )
        try:
            self.validator.validate(plan)
        except Exception as e:
            self.fail(f"Validation failed: {e}")

    def test_validator_non_existing_tool(self):
        plan = self.GlobalPlan(
            summary="Non existent tool used",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="this_tool_does_not_exist", args=[]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("not found in registry", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_non_existing_key(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="non_existing_key", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("not defined in tool", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_missing_key(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("Tool 'go_to_pose' expects 3 arguments", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_extra_key(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="w", val=1.0), self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("Tool 'go_to_pose' expects 3 arguments", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_no_arguments_accepted(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="empty", args=[self.Arg(key="random", val=1.0)]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("does not accept arguments, but arguments were provided", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_wrong_last_bracket_missing(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Arrived at {{bb.missing_brace")]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("Blackboard reference in argument", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_wrong_first_bracket_missing(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Arrived at bb.missing_brace}}")]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("Blackboard reference in argument", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_wrong_all_brackets_missing(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="go_to_pose", args=[self.Arg(key="x", val=1.0), self.Arg(key="y", val=2.0), self.Arg(key="z", val=0.0)]),
                        self.Step(tool="speak", args=[self.Arg(key="text", val="Arrived at bb.missing_brace")]),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn("Blackboard reference in argument", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

    def test_validator_correct_types(self):
        plan = self.GlobalPlan(
            summary="Check types of arguments",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="types", args=[
                            self.Arg(key="int", val=4),
                            self.Arg(key="integer", val=2),
                            self.Arg(key="float", val=4.2),
                            self.Arg(key="bool", val=True),
                            self.Arg(key="boolean", val=False),
                            ]
                        ),
                    ],
                )
            ],
        )
        try:
            self.validator.validate(plan)
        except Exception as e:
            self.fail("Test 'test_validator_correct_types' should not fail")

        # Cast of int to float is accepted, not the other way around
        plan.plan[0].steps[0].args[2] = self.Arg(key="float", val=4)  # Pass a int instead of float
        try:
            self.validator.validate(plan)
        except Exception as e:
            self.fail("Test 'test_validator_correct_types' should not fail for int->float cast")

        # Non-string types can accept string bb references
        plan.plan[0].steps[0].args[2] = self.Arg(key="float", val="{{bb.tool.float_value}}")
        try:
            self.validator.validate(plan)
        except Exception as e:
            self.fail("Test 'test_validator_correct_types' should not fail for bb reference in string format")

    def test_validator_wrong_types(self):
        plan = self.GlobalPlan(
            summary="Navigate to a location",
            plan=[
                self.PlanNode(
                    kind="SEQUENCE",
                    steps=[
                        self.Step(tool="types", args=[
                            self.Arg(key="int", val="4"),
                            # Add only one wrong type on each test, rest should be correct
                            self.Arg(key="integer", val=2),
                            self.Arg(key="float", val=4.2),
                            self.Arg(key="bool", val=True),
                            self.Arg(key="boolean", val=False),
                            ]
                        ),
                    ],
                )
            ],
        )
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'int' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

        plan.plan[0].steps[0].args[0] = self.Arg(key="int", val=4.2)
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'int' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

        plan.plan[0].steps[0] = self.Step(tool="types", args=[
                                    self.Arg(key="int", val=4),
                                    self.Arg(key="integer", val="2"),
                                    self.Arg(key="float", val=4.2),
                                    self.Arg(key="bool", val=True),
                                    self.Arg(key="boolean", val=False)])
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'integer' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

        plan.plan[0].steps[0] = self.Step(tool="types", args=[
                                    self.Arg(key="int", val=4),
                                    self.Arg(key="integer", val=2),
                                    self.Arg(key="float", val="42.0"),
                                    self.Arg(key="bool", val=True),
                                    self.Arg(key="boolean", val=False)])
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'float' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

        plan.plan[0].steps[0] = self.Step(tool="types", args=[
                                    self.Arg(key="int", val=4),
                                    self.Arg(key="integer", val=2),
                                    self.Arg(key="float", val=42.0),
                                    self.Arg(key="bool", val="true"),
                                    self.Arg(key="boolean", val=False)])
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'bool' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")

        plan.plan[0].steps[0] = self.Step(tool="types", args=[
                                    self.Arg(key="int", val=4),
                                    self.Arg(key="integer", val=2),
                                    self.Arg(key="float", val=42.0),
                                    self.Arg(key="bool", val=True),
                                    self.Arg(key="boolean", val="false")])
        try:
            fail = False
            self.validator.validate(plan)
        except Exception as e:
            fail = True
            self.assertIn(f"Argument 'boolean' of tool 'types' expects type", str(e))
        self.assertTrue(fail, "Validator did not catch non-existing key error")


if __name__ == "__main__":
    unittest.main()
