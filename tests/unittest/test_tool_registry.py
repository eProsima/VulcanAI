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
import io
import numpy as np
import os
import re
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


# Add src/ to sys.path for src-layout imports
CURRENT_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, "resources"))
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class TestToolRegistry(unittest.TestCase):
    def setUp(self):
        # Import package modules dynamically to avoid static import issues
        tool_registry_mod = importlib.import_module("vulcanai.tools.tool_registry")
        tools_mod = importlib.import_module("vulcanai.tools.tools")
        plan_types_mod = importlib.import_module("vulcanai.core.plan_types")

        # Keep references we need
        self.ToolRegistry = tool_registry_mod.ToolRegistry
        self.AtomicTool = tools_mod.AtomicTool
        self.CompositeTool = tools_mod.CompositeTool
        self.ValidationTool = tools_mod.ValidationTool
        self.Arg = plan_types_mod.ArgValue

        # Lightweight embedder matching expected API
        class LocalDummyEmbedder:
            def embed(self, text: str) -> np.ndarray:
                h = hashlib.sha256(text.encode("utf-8")).digest()
                vec = np.frombuffer(h, dtype=np.uint8).astype(np.float32)[:64]
                norm = np.linalg.norm(vec) or 1.0
                return vec / norm
            def similarity(self, a: np.ndarray, b: np.ndarray) -> float:
                return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-8))
        self.Embedder = LocalDummyEmbedder

        # Define dummy tools inside setUp to use imported base class
        class NavTool(self.AtomicTool):
            name = "go_to_pose"
            description = "Navigate robot to a target location"
            tags = ["navigation", "goal", "go", "move"]
            input_schema = [("x", "float"), ("y", "float"), ("z", "float")]
            output_schema = {"arrived": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"arrived": True}
        class DetectTool(self.AtomicTool):
            name = "detect_object"
            description = "Detect an object in the environment"
            tags = ["vision", "perception"]
            input_schema = [("label", "string")]
            output_schema = {"found": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"found": True}
        class SpeakTool(self.AtomicTool):
            name = "speak"
            description = "Speak a text string"
            tags = ["speech", "text"]
            input_schema = [("text", "string")]
            output_schema = {"spoken": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"spoken": True}
        class ComplexTool(self.CompositeTool):
            name = "complex_action"
            description = "A complex action using multiple tools"
            tags = ["complex", "action"]
            input_schema = [("param", "string")]
            output_schema = {"result": "bool"}
            version = "0.1"
            dependencies = ["go_to_pose", "detect_object", "speak"]
            def run(self, **kwargs):
                return {"result": True}

        class TestValidationTool(self.ValidationTool):
            name = "test_validation"
            description = "A test validation tool"
            tags = ["validation"]
            input_schema = [("param", "string")]
            output_schema = {"result": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"result": True}

        # Save for use
        self.NavTool = NavTool
        self.DetectTool = DetectTool
        self.SpeakTool = SpeakTool
        self.ComplexTool = ComplexTool
        self.TestValidationTool = TestValidationTool

        # Build registry with dummy embedder and register tools
        self.registry = self.ToolRegistry(embedder=self.Embedder())
        self.registry.register_tool(self.NavTool())
        self.registry.register_tool(self.DetectTool())
        self.registry.register_tool(self.SpeakTool())

    def test_top_k_returns_expected_count(self):
        """Test that top_k returns up to k tools."""
        res = self.registry.top_k("go to the room", k=2)
        self.assertEqual(len(res), 2+1)  # +1 for help tool

    def test_top_k_returns_expected_count_validation(self):
        """Test that top_k returns up to k tools even if there are more non-validation tools registered."""
        self.registry.register_tool(self.TestValidationTool())
        res = self.registry.top_k("go to the room", k=2, validation=True)
        self.assertEqual(len(res), 1)

    def test_top_k_returns_registered_instances(self):
        """Test that top_k returns only registered tool instances."""
        res = self.registry.top_k("detect", k=5)
        names = {t.name for t in res}
        for n in names:
            self.assertIn(n, self.registry.tools)

    def test_top_k_returns_registered_instances_validation(self):
        """Test that top_k returns only registered tool instances."""
        res = self.registry.top_k("detect", k=5, validation=True)
        names = {t.name for t in res}
        for n in names:
            self.assertIn(n, self.registry.tools)

    def test_empty_registry(self):
        """Test that top_k on empty registry returns empty list."""
        r = self.ToolRegistry(embedder=self.Embedder())
        res = r.top_k("anything", k=3)
        # When no tools are registered, only the help tool is present.
        # However, top_k will return an empty list because a tool registry with only the help tool
        # has no purpose and should report an error.
        self.assertEqual(res, [])
        self.assertEqual(len(r.tools), 1)  # Only help tool

    def test_k_greater_than_tools(self):
        """Test that k greater than number of tools returns all tools skipping embedding."""
        # Mock embedder to track calls
        class MockEmbedder(self.Embedder):
            def __init__(self):
                super().__init__()
                self.embed_call_count = 0
            def embed(self, text: str) -> np.ndarray:
                self.embed_call_count += 1
                return super().embed(text)
        r = self.ToolRegistry(embedder=MockEmbedder())
        r.register_tool(self.NavTool())
        r.register_tool(self.DetectTool())
        r.register_tool(self.SpeakTool())
        self.assertEqual(r.embedder.embed_call_count, 3)
        res = r.top_k("go to the room", k=10)
        self.assertEqual(len(res), 3+1)  # +1 for help tool
        # Check that self.embedder.embed has not been called during top_k
        self.assertEqual(r.embedder.embed_call_count, 3)
        res = r.top_k("detect", k=2)
        self.assertEqual(len(res), 2+1)  # +1 for help tool
        self.assertEqual(r.embedder.embed_call_count, 4)

    # Test registers
    def test_register_tool(self):
        """Test that tools can be registered and are present in the registry."""
        r = self.ToolRegistry(embedder=self.Embedder())
        self.assertEqual(len(r.tools), 0+1)  # +1 for help tool
        r.register_tool(self.NavTool())
        self.assertEqual(len(r.tools), 1+1)  # +1 for help tool
        r.register_tool(self.DetectTool())
        self.assertEqual(len(r.tools), 2+1)  # +1 for help tool
        r.register_tool(self.SpeakTool())
        self.assertEqual(len(r.tools), 3+1)  # +1 for help tool
        self.assertIn("go_to_pose", r.tools)
        self.assertIn("detect_object", r.tools)
        self.assertIn("speak", r.tools)
        self.assertNotIn("nonexistent_tool", r.tools)
        r.register_tool(self.TestValidationTool())
        self.assertEqual(len(r.tools), 4+1)
        self.assertIn("test_validation", r.tools)

    def test_register_tool_from_file(self):
        # Register tools from test_tools.py
        self.registry.discover_tools_from_file(os.path.join(RESOURCES_DIR, "test_tools.py"))
        self.assertEqual(len(self.registry.tools), 7+1)  # +1 for help tool (3 existing + 4 new)
        self.assertEqual(len(self.registry._index), 7)   # (3 existing + 4 new)

    def test_register_tool_from_nonexistent_file(self):
        r = self.ToolRegistry(embedder=self.Embedder())
        r.discover_tools_from_file("/path/does/not/exist.py")
        self.assertEqual(len(r.tools), 0+1)  # +1 for help tool

    def test_register_composite_tool_solves_deps(self):
        """Test that registering a composite tool correctly resolves its dependencies."""
        # Check that the composite tool has no resolved deps before registration
        self.assertEqual(len(self.registry.tools), 3+1)  # +1 for help tool
        self.assertEqual(len(self.ComplexTool().dependencies), 3)
        self.assertEqual(len(self.ComplexTool().resolved_deps), 0)
        # Register the composite tool
        self.registry.register_tool(self.ComplexTool())
        self.assertEqual(len(self.registry.tools), 4+1)  # +1 for help tool
        self.assertIn("complex_action", self.registry.tools)
        self.assertEqual(len(self.registry.tools.get("complex_action").resolved_deps), 3)

    def test_register_composite_tool_fails_reports_error(self):
        """Test that registering a composite tool reports an error if there are no dependencies."""
        buf = io.StringIO()
        sys.stdout = buf
        # Check that the composite tool has no resolved deps before registration
        r = self.ToolRegistry(embedder=self.Embedder())
        self.assertEqual(len(r.tools), 0+1)  # +1 for help tool
        self.assertEqual(len(self.ComplexTool().dependencies), 3)
        self.assertEqual(len(self.ComplexTool().resolved_deps), 0)
        # Register the composite tool
        r.register_tool(self.ComplexTool())
        self.assertEqual(len(r.tools), 1+1)  # +1 for help tool
        self.assertIn("complex_action", r.tools)
        self.assertEqual(len(self.ComplexTool().resolved_deps), 0)
        # Capture output to check for error messages
        sys.stdout = sys.__stdout__  # Reset stdout
        output = buf.getvalue()
        output = re.sub(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])", "", output)  # Remove ANSI codes
        self.assertIn("Dependency 'go_to_pose' for tool 'complex_action' not found", output)
        self.assertIn("Dependency 'detect_object' for tool 'complex_action' not found", output)
        self.assertIn("Dependency 'speak' for tool 'complex_action' not found", output)

    def test_register_composite_tool_solves_deps_from_file(self):
        """Test that registering a composite tool from file correctly resolves its dependencies."""
        r = self.ToolRegistry(embedder=self.Embedder())
        buf = io.StringIO()
        sys.stdout = buf
        r.discover_tools_from_file(os.path.join(RESOURCES_DIR, "test_tools.py"))
        r.discover_tools_from_file(os.path.join(RESOURCES_DIR, "test_composite_tool.py"))
        sys.stdout = sys.__stdout__  # Reset stdout
        output = buf.getvalue()
        output = re.sub(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])", "", output)  # Remove ANSI codes
        self.assertNotIn("Dependency 'file_tool' for tool 'complex_file_action' not found", output)
        self.assertNotIn("Dependency 'new_file_tool' for tool 'complex_file_action' not found", output)
        self.assertNotIn("Dependency 'other_file_tool' for tool 'complex_file_action' not found", output)
        self.assertEqual(len(r.tools), 5+1) # +1 for help tool

    def test_register_composite_tool_fails_from_file(self):
        """Test that registering a composite tool from file reports an error if there are no dependencies."""
        buf = io.StringIO()
        sys.stdout = buf
        # Register tool from test_composite_tool.py and expect errors for missing deps
        r = self.ToolRegistry(embedder=self.Embedder())
        r.discover_tools_from_file(os.path.join(RESOURCES_DIR, "test_composite_tool.py"))
        sys.stdout = sys.__stdout__  # Reset stdout
        output = buf.getvalue()
        output = re.sub(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])", "", output)  # Remove ANSI codes
        self.assertIn("Dependency 'file_tool' for tool 'complex_file_action' not found", output)
        self.assertIn("Dependency 'new_file_tool' for tool 'complex_file_action' not found", output)
        self.assertIn("Dependency 'other_file_tool' for tool 'complex_file_action' not found", output)
        self.assertEqual(len(r.tools), 1+1)  # +1 for help tool

    def test_register_validation_tool(self):
        """Test that validation tools can be registered and are present in the registry."""
        r = self.ToolRegistry(embedder=self.Embedder())
        self.assertEqual(len(r.tools), 0+1)  # +1 for help tool
        self.assertEqual(len(r._index), 0)
        r.register_tool(self.TestValidationTool())
        self.assertEqual(len(r.tools), 1+1)  # +1 for help tool
        self.assertEqual(len(r._index), 1)
        self.assertIn("test_validation", r.validation_tools)


if __name__ == "__main__":
    unittest.main()
