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


# Add src/ to sys.path for src-layout imports
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class TestToolRegistry(unittest.TestCase):
    def setUp(self):
        # Import package modules dynamically to avoid static import issues
        tool_registry_mod = importlib.import_module("vulcanai.tool_registry")
        tools_mod = importlib.import_module("vulcanai.tools")

        # Keep references we need
        self.ToolRegistry = tool_registry_mod.ToolRegistry
        self.ITool = tools_mod.ITool

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
        class NavTool(self.ITool):
            name = "go_to_pose"
            description = "Navigate robot to a target location"
            tags = ["navigation", "goal", "go", "move"]
            input_schema = {"pose": "geometry_msgs/Pose"}
            output_schema = {"arrived": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"arrived": True}
        class DetectTool(self.ITool):
            name = "detect_object"
            description = "Detect an object in the environment"
            tags = ["vision", "perception"]
            input_schema = {"label": "string"}
            output_schema = {"found": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"found": True}
        class SpeakTool(self.ITool):
            name = "speak"
            description = "Speak a text string"
            tags = ["speech", "text"]
            input_schema = {"text": "string"}
            output_schema = {"spoken": "bool"}
            version = "0.1"
            def run(self, **kwargs):
                return {"spoken": True}

        # Save for use
        self.NavTool = NavTool
        self.DetectTool = DetectTool
        self.SpeakTool = SpeakTool

        # Build registry with dummy embedder and register tools
        self.registry = self.ToolRegistry(embedder=self.Embedder())
        self.registry.register_tool(self.NavTool())
        self.registry.register_tool(self.DetectTool())
        self.registry.register_tool(self.SpeakTool())

    def test_top_k_returns_expected_count(self):
        """Test that top_k returns up to k tools."""
        res = self.registry.top_k("go to the room", k=2)
        self.assertEqual(len(res), 2)

    def test_top_k_returns_registered_instances(self):
        """Test that top_k returns only registered tool instances."""
        res = self.registry.top_k("detect", k=5)
        names = {t.name for t in res}
        for n in names:
            self.assertIn(n, self.registry.tools)

    def test_empty_registry(self):
        """Test that top_k on empty registry returns empty list."""
        r = self.ToolRegistry(embedder=self.Embedder())
        res = r.top_k("anything", k=3)
        self.assertEqual(res, [])

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
        self.assertEqual(len(res), 3)
        # Check that self.embedder.embed has not been called during top_k
        self.assertEqual(r.embedder.embed_call_count, 3)
        res = r.top_k("detect", k=2)
        self.assertEqual(len(res), 2)
        self.assertEqual(r.embedder.embed_call_count, 4)

    # Test registers
    def test_register_tool(self):
        """Test that tools can be registered and are present in the registry."""
        r = self.ToolRegistry(embedder=self.Embedder())
        self.assertEqual(len(r.tools), 0)
        r.register_tool(self.NavTool())
        self.assertEqual(len(r.tools), 1)
        r.register_tool(self.DetectTool())
        self.assertEqual(len(r.tools), 2)
        r.register_tool(self.SpeakTool())
        self.assertEqual(len(r.tools), 3)
        print(r.tools)
        self.assertIn("go_to_pose", r.tools)
        self.assertIn("detect_object", r.tools)
        self.assertIn("speak", r.tools)
        self.assertNotIn("nonexistent_tool", r.tools)

    def test_register_tool_from_file(self):
        # Register tools from test_tools.py
        self.registry.discover_tools_from_file(os.path.join(CURRENT_DIR, "resources", "test_tools.py"))
        print(len(self.registry.tools))
        self.assertEqual(len(self.registry.tools), 6)

    def test_register_tool_from_nonexistent_file(self):
        r = self.ToolRegistry(embedder=self.Embedder())
        r.discover_tools_from_file("/path/does/not/exist.py")
        print(r.tools)
        self.assertEqual(len(r.tools), 0)


if __name__ == "__main__":
    unittest.main()
