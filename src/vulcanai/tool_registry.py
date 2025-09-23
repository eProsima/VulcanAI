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

from vulcanai.embedder import SBERTEmbedder
from vulcanai.logger import VulcanAILogger
from vulcanai.plan_types import ArgValue
from vulcanai.tools import ITool


def vulcanai_tool(cls: Type[ITool]):
    """Class decorator to mark a class as a VulcanAI tool."""
    if not issubclass(cls, ITool):
        raise TypeError(f"{cls.__name__} must inherit from ITool")
    setattr(cls, "__is_vulcanai_tool__", True)
    return cls


class HelpTool(ITool):
    """A tool that provides help information."""
    name = "help"
    description = "Provides help information for using the library. It can list all available tools or" \
                    " give info about the usage of a specific tool if <tool_name> is provided as an argument."
    tags = ["help", "info", "documentation", "usage", "developer", "manual", "available tools"]
    input_schema = [("tool", "string")]
    output_schema = {"info": "str"}
    version = "0.1"

    available_tools: Dict[str, ITool] = {}

    def run(self, **kwargs):
        help_msg = (
            "This is the VulcanAI help tool. Use it to get information about available tools.\n"
            "Example queries:\n"
            "  - 'List all available tools'\n"
            "  - 'Provide info about the detect_pose tool'\n\n"
            "Info requested:\n"
        )

        tool_name = kwargs.get("tool")
        if tool_name:
            tool = self.available_tools.get(tool_name)
            if tool:
                help_msg += f"\nTool: {tool.name}: {tool.description}\n"
                help_msg += f"  Inputs: {tool.input_schema}\n  Outputs: {tool.output_schema}\n"
                help_msg += f"  Tags: {tool.tags}\n  Version: {tool.version}\n"
            else:
                help_msg += f"\nNo tool found with the name '{tool_name}'."
        else:
            help_msg += "\nAvailable tools:\n"
            for tool in self.available_tools.values():
                help_msg += f"- {tool.name}: {tool.description}\n"

        return help_msg


class ToolRegistry:
    """Holds all known tools and performs vector search over metadata."""
    def __init__(self, embedder=None, logger=None):
        # Logging function
        self.logger = logger or VulcanAILogger().log_registry
        # Dictionary of tool name -> tool instance
        self.tools: Dict[str, ITool] = {}
        # Embedding model for tool metadata
        self.embedder = embedder or SBERTEmbedder()
        # Simple in-memory index of (name, embedding)
        self._index: list[Tuple[str, np.ndarray]] = []
        # List of modules where tools can be loaded from
        self._loaded_modules: list[ModuleType] = []
        # Add help_tool to registry but not to index
        self.help_tool = HelpTool()
        self.tools[self.help_tool.name] = self.help_tool

    def register_tool(self, tool: ITool):
        """Register a single tool instance."""
        # Avoid duplicates
        if tool.name in self.tools:
            return
        self.tools[tool.name] = tool
        emb = self.embedder.embed(self._doc(tool))
        self._index.append((tool.name, emb))
        self.logger(f"Registered tool: {tool.name}")
        self.help_tool.available_tools = self.tools

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
            self.logger(f"Error loading tools from {path}: {e}", error=True)

    def discover_tools_from_file(self, path: str):
        """Load tools from a Python file and register them."""
        self.load_tools_from_file(path)
        self.register()
        self.help_tool.available_tools = self.tools

    def discover_tools_from_entry_points(self, group: str = "custom_tools"):
        """Load tools from Python entry points."""
        for ep in entry_points(group=group):
            module = importlib.import_module(ep.module)
            self._loaded_modules.append(module)
        self.register()
        self.help_tool.available_tools = self.tools

    def discover_ros(self):
        # Query ROS graph for /<tool>/tool_info services and register dynamically
        # NOTE: Implement a small client that calls tool_info and constructs a proxy
        ...

    def top_k(self, query: str, k: int = 5) -> list[ITool]:
        """Return top-k tools most relevant to the query."""
        if not self._index:
            self.logger("No tools registered.", error=True)
            return []

        # If k > number of tools, return all
        if k > len(self._index):
            return list(self.tools.values())

        q_vec = self.embedder.embed(query)
        scores = []
        for name, vec in self._index:
            scores.append((self.embedder.similarity(q_vec, vec), name))

        # Sort by similarity
        scores.sort(reverse=True, key=lambda x: x[0])
        names = [name for _, name in scores[:k]]
        # Add help tool as it always available
        names.append("help")
        return [self.tools[name] for name in names]

    @staticmethod
    def _doc(tool: ITool) -> str:
        # Text used for embeddings
        return f"{tool.name}\n{tool.description}\n{tool.tags}\n{tool.input_schema}\n"
