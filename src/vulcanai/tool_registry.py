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

import importlib
import numpy as np
import sys
from importlib.metadata import entry_points
from pathlib import Path
from types import ModuleType
from typing import Dict, Tuple, Type

from vulcanai.tools import ITool
from vulcanai.embedder import SBERTEmbedder


def vulcanai_tool(cls: Type[ITool]):
    if not issubclass(cls, ITool):
        raise TypeError(f"{cls.__name__} must inherit from ITool")
    setattr(cls, "__is_vulcanai_tool__", True)
    return cls


class ToolRegistry:
    """Holds all known tools and performs vector search over metadata."""
    def __init__(self, embedder=None):
        # Dictionary of tool name -> tool instance
        self.tools: Dict[str, ITool] = {}
        # Embedding model for tool metadata
        self.embedder = embedder or SBERTEmbedder()
        # Simple in-memory index of (name, embedding)
        self._index: list[Tuple[str, np.ndarray]] = []
        # List of modules where tools can be loaded from
        self._loaded_modules: list[ModuleType] = []

    def register_tool(self, tool: ITool):
        """Register a single tool instance."""
        # Avoid duplicates
        if tool.name in self.tools:
            return
        self.tools[tool.name] = tool
        emb = self.embedder.embed(self._doc(tool))
        self._index.append((tool.name, emb))
        print(f"Registered tool: {tool.name}")

    def register(self):
        """Register all loaded classes marked with @vulcanai_tool."""
        for module in self._loaded_modules:
            for name in dir(module):
                tool = getattr(module, name, None)
                if isinstance(tool, type) and issubclass(tool, ITool):
                    if getattr(tool, "__is_vulcanai_tool__", False):
                        self.register_tool(tool())

    def load_tools_from_file(self, path: str):
        """Dynamically load a Python file with @vulcanai_tool classes."""
        try:
            path = Path(path)
            module_name = path.stem

            spec = importlib.util.spec_from_file_location(module_name, str(path))
            if spec is None:
                raise ImportError(f"Cannot import {path}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            self._loaded_modules.append(module)
        except Exception as e:
            print(f"Error loading tools from {path}: {e}")

    def discover_tools_from_file(self, path: str):
        """Load tools from a Python file and register them."""
        self.load_tools_from_file(path)
        self.register()

    def discover_tools_from_entry_points(self, group: str = "custom_tools"):
        """Load tools from Python entry points."""
        for ep in entry_points(group=group):
            module = importlib.import_module(ep.module)
            self._loaded_modules.append(module)
        self.register()

    def discover_ros(self):
        # Query ROS graph for /<tool>/tool_info services and register dynamically
        # NOTE: Implement a small client that calls tool_info and constructs a proxy
        ...

    def top_k(self, query: str, k: int = 5) -> list[ITool]:
        """Return top-k tools most relevant to the query."""
        if not self._index:
            print("No tools registered.")
            return []

        # If k > number of tools, return all
        if k > len(self._index):
            return list(self.tools.values())

        q_vec = self.embedder.embed(query)
        scores = []
        for name, vec in self._index:
            # Cosine similarity
            sim = float(np.dot(q_vec, vec) / (np.linalg.norm(q_vec) * np.linalg.norm(vec) + 1e-8))
            print(f"Cosine Similarity [{name}]: {sim}")
            scores.append((sim, name))
            print(f"Similarity built-in: {self.embedder.similarity(q_vec, vec)}")

        # Sort by similarity
        scores.sort(reverse=True, key=lambda x: x[0])
        print("Scores:", scores)
        names = [name for _, name in scores[:k]]
        return [self.tools[name] for name in names]

    @staticmethod
    def _doc(tool: ITool) -> str:
        # Text used for embeddings
        return f"{tool.name}\n{tool.description}\n{tool.tags}\n{tool.input_schema}\n"
