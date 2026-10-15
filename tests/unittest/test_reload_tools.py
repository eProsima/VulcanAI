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

"""Run with the VulcanAI Python dependencies installed (including in Docker)."""

import importlib
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

SOURCE = Path(__file__).resolve().parents[2] / "src"
if SOURCE.is_dir():
    sys.path.insert(0, str(SOURCE))
sys.modules.setdefault("sentence_transformers", types.SimpleNamespace(SentenceTransformer=Mock()))

HEADER = "from vulcanai import AtomicTool, CompositeTool, vulcanai_tool\n"


def atomic(name="move", result="old", description="Move"):
    return f"""@vulcanai_tool
class Move(AtomicTool):
    name={name!r}
    description={description!r}
    input_schema=[]
    output_schema={{"value":"string"}}
    def run(self): return {{"value":{result!r}}}
"""


COMPOSITE = """@vulcanai_tool
class Draw(CompositeTool):
    name="draw"
    description="Draw"
    input_schema=[]
    output_schema={"value":"string"}
    dependencies=["move"]
    def run(self): return self.resolved_deps["move"].run()
"""


class ReloadTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "tools.py"
        self.embedder = Mock()
        self.embedder.embed.side_effect = lambda doc: np.array([len(doc)], dtype=float)
        ToolRegistry = importlib.import_module("vulcanai.tools.tool_registry").ToolRegistry
        self.registry = ToolRegistry(embedder=self.embedder, logger=Mock(), default_tools=False)
        self.path.write_text(HEADER + atomic() + COMPOSITE)
        self.registry.discover_tools_from_file(str(self.path))

    def reload(self):
        return self.registry.reload_tools_from_files()

    def test_reload_uses_new_source_and_rebinds_dependencies(self):
        old = self.registry.tools["move"]
        self.path.write_text(HEADER + atomic(result="new") + COMPOSITE)
        result = self.reload()
        self.assertEqual(result["updated"], ["draw", "move"])
        self.assertIsNot(self.registry.tools["move"], old)
        self.assertEqual(self.registry.tools["draw"].run(), {"value": "new"})
        self.assertIs(self.registry.tools["draw"].resolved_deps["move"], self.registry.tools["move"])
        self.assertEqual(len([n for n, _ in self.registry._index if n == "move"]), 1)

    def test_inactive_choices_preserved_and_new_tools_activated(self):
        self.registry.deactivate_tool("move")
        self.path.write_text(
            HEADER + atomic(result="new") + COMPOSITE + atomic("extra").replace("class Move", "class Extra")
        )
        self.reload()
        self.assertNotIn("move", self.registry.tools)
        self.assertIn("move", self.registry.deactivated_tools)
        self.assertEqual(self.registry.tools["draw"].run(), {"value": "new"})
        self.assertIn("extra", self.registry.tools)

    def test_deleted_and_renamed_tools_removed(self):
        self.path.write_text(HEADER + atomic("renamed"))
        result = self.reload()
        self.assertEqual(result["removed"], ["draw", "move"])
        self.assertEqual(set(self.registry.tools), {"help", "renamed"})
        self.assertEqual([n for n, _ in self.registry._index], ["renamed"])
        self.assertIs(self.registry.help_tool.available_tools, self.registry.tools)

    def test_syntax_error_preserves_registry(self):
        previous = dict(self.registry.tools)
        self.path.write_text("def invalid(:")
        with self.assertRaises(SyntaxError):
            self.reload()
        self.assertEqual(self.registry.tools, previous)
        self.assertEqual(self.registry.tools["draw"].run(), {"value": "old"})

    def test_missing_dependency_preserves_registry(self):
        previous = dict(self.registry.tools)
        self.path.write_text(HEADER + COMPOSITE)
        with self.assertRaisesRegex(ValueError, "missing dependencies"):
            self.reload()
        self.assertEqual(self.registry.tools, previous)

    def test_same_stem_files_and_cross_file_dependency(self):
        self.path.write_text(HEADER + atomic())
        self.reload()
        other = Path(self.tmp.name) / "sub" / "tools.py"
        other.parent.mkdir()
        other.write_text(HEADER + COMPOSITE)
        self.registry.discover_tools_from_file(str(other))
        self.path.write_text(HEADER + atomic(result="new"))
        self.registry.reload_tools_from_files([str(self.path)])
        self.assertEqual(self.registry.tools["draw"].run(), {"value": "new"})

    def test_collision_with_another_file_preserves_registry(self):
        other = Path(self.tmp.name) / "other.py"
        other.write_text(HEADER + atomic("other"))
        self.registry.discover_tools_from_file(str(other))
        previous = dict(self.registry.tools)
        self.path.write_text(HEADER + atomic("other"))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.registry.reload_tools_from_files([str(self.path)])
        self.assertEqual(self.registry.tools, previous)

    def test_empty_file_removes_decorated_tools(self):
        self.path.write_text(HEADER)
        self.reload()
        self.assertEqual(set(self.registry.tools), {"help"})

    def test_multi_file_failure_does_not_partially_update(self):
        other = Path(self.tmp.name) / "other.py"
        other.write_text(HEADER + atomic("other"))
        self.registry.discover_tools_from_file(str(other))
        self.path.write_text(HEADER + atomic(result="new") + COMPOSITE)
        other.write_text("invalid syntax!")
        with self.assertRaises(SyntaxError):
            self.reload()
        self.assertEqual(self.registry.tools["move"].run(), {"value": "old"})

    def test_failed_discovery_restores_existing_module(self):
        module = self.registry._file_modules[str(self.path)]
        self.path.write_text("broken syntax!")
        self.registry.discover_tools_from_file(str(self.path))
        self.assertIs(sys.modules[module.__name__], module)
        self.assertEqual(self.registry.tools["move"].run(), {"value": "old"})

    def test_unrelated_broken_composite_does_not_block_reload(self):
        from vulcanai.tools.tools import CompositeTool

        class Broken(CompositeTool):
            name = "unrelated"
            description = "Already registered with a missing dependency"
            dependencies = ["not_registered"]

            def run(self):
                return {}

        self.registry.register_tool(Broken())
        self.path.write_text(HEADER + atomic(result="new") + COMPOSITE)
        self.reload()
        self.assertEqual(self.registry.tools["draw"].run(), {"value": "new"})


if __name__ == "__main__":
    unittest.main()
