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
from typing import Dict, List, Tuple, Type

from vulcanai.tools.embedder import SBERTEmbedder
from vulcanai.console.logger import VulcanAILogger
from vulcanai.tools.tools import ITool, CompositeTool


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
                    " give info about the usage of a specific tool if 'tool_name' is provided as an argument."
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
        # Logging function from the class VulcanConsole
        self.logger = logger or VulcanAILogger.default()
        # Dictionary of tools (name -> tool instance)
        self.tools: Dict[str, ITool] = {}
        # Dictionary of deactivated_tools (name -> tool instance)
        self.deactivated_tools: Dict[str, ITool] = {}
        # Embedding model for tool metadata
        self.embedder = embedder or SBERTEmbedder()
        # Simple in-memory index of (name, embedding)
        self._index: list[Tuple[str, np.ndarray]] = []
        # List of modules where tools can be loaded from
        self._loaded_modules: list[ModuleType] = []
        # Add help_tool to registry but not to index
        self.help_tool = HelpTool()
        self.tools[self.help_tool.name] = self.help_tool
        # Validation tools list to retrieve validation tools separately
        self.validation_tools: List[str] = []

    def register_tool(self, tool: ITool, solve_deps: bool = True):
        """Register a single tool instance."""
        # Avoid duplicates
        if tool.name in self.tools:
            return

        self.tools[tool.name] = tool
        if tool.is_validation_tool:
            self.validation_tools.append(tool.name)
        emb = self.embedder.embed(self._doc(tool))
        self._index.append((tool.name, emb))
        self.logger.log_registry(f"Registered tool: [registry]{tool.name}[/registry]")
        self.help_tool.available_tools = self.tools
        if solve_deps:
            # Get class of tool
            if issubclass(type(tool), CompositeTool):
                self._resolve_dependencies(tool)

    def activate_tool(self, tool_name) -> bool:
        """Activate a singles tool instance."""
        # Check if the tool is already active
        if tool_name in self.tools:
            return False
        # Check if the tool is deactivated
        if tool_name not in self.deactivated_tools:
            self.logger.log_registry(f"Tool [registry]'{tool_name}'[/registry] " + \
                        f"not found in the deactivated tools list.", error=True)
            return False

        # Add the tool to the active tools
        self.tools[tool_name] = self.deactivated_tools[tool_name]

        # Removed the tool from the deactivated tools
        del self.deactivated_tools[tool_name]

        return True

    def deactivate_tool(self, tool_name) -> bool:
        """Deactivate a singles tool instance."""
        # Check if the tool is already deactivated
        if tool_name in self.deactivated_tools:
            return False
        # Check if the tool is active
        if tool_name not in self.tools:
            self.logger.log_registry(f"Tool [registry]'{tool_name}'[/registry] "+ \
                        f"not found in the active tools list.", error=True)
            return False

        # Add the tool to the deactivated tools
        self.deactivated_tools[tool_name] = self.tools[tool_name]

        # Removed the tool from the active tools
        del self.tools[tool_name]

        return True

    def register(self):
        """Register all loaded classes marked with @vulcanai_tool."""
        composite_classes = []
        for module in self._loaded_modules:
            for name in dir(module):
                tool = getattr(module, name, None)
                if isinstance(tool, type) and issubclass(tool, ITool):
                    if getattr(tool, "__is_vulcanai_tool__", False):
                        if issubclass(tool, CompositeTool):
                            composite_classes.append(tool)
                        else:
                            self.register_tool(tool(), solve_deps=False)
        # Register composite tools after atomic ones to resolve dependencies
        for tool_cls in composite_classes:
            tool = tool_cls()
            self.register_tool(tool, solve_deps=True)

    def _resolve_dependencies(self, tool: CompositeTool):
        """Resolve and attach dependencies for a CompositeTool."""
        for dep_name in tool.dependencies:
            dep_tool = self.tools.get(dep_name)
            if dep_tool is None:
                self.logger.log_registry(f"ERROR. Dependency '{dep_name}' for tool '{tool.name}' not found.", error=True)
            else:
                tool.resolved_deps[dep_name] = dep_tool


    def _load_tools_from_file(self, path: str):
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
            self.logger.log_registry(f"Could not load tools from {path}: {e}", error=True)
    def discover_tools_from_file(self, path: str):
        """Load tools from a Python file and register them."""
        self._load_tools_from_file(path)
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

    def top_k(self, query: str, k: int = 5, validation: bool = False) -> list[ITool]:
        """Return top-k tools most relevant to the query."""
        if not self._index:
            self.logger.log_registry("No tools registered.", error=True)
            return []

        # Filter tools based on validation flag
        all_names: set = set(self.tools.keys())
        val_names: set = set(self.validation_tools or [])
        # Keep only names that actually exist in tools
        val_names &= all_names
        nonval_names: set = all_names - val_names

        active_names: set = val_names if validation else nonval_names
        if not active_names:
            # If there is no tool for the requested category, be explicit and return []
            self.logger.log_registry(
                f"No matching tools for the requested mode ({'validation' if validation else 'action'}).",
                error=True
            )
            return []
        # If k > number of ALL tools, return required tools
        if k > len(self._index):
            return [self.tools[name] for name in active_names]

        filtered_index = [(name, vec) for (name, vec) in self._index if name in active_names]
        if not filtered_index:
            # Index might be stale; log and return []

            self.logger.log_registry("Index has no entries for the selected tool subset.",
                        error=True)
            return []
        # If k > number of required tools, return required tools
        if k > len(filtered_index):
            return [self.tools[name] for name in active_names]

        q_vec = self.embedder.embed(query)
        scores = []
        for name, vec in filtered_index:
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
