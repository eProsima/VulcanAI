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

import ast
import importlib
import inspect
import sys
import textwrap
import traceback
from importlib.metadata import entry_points
from pathlib import Path
from types import ModuleType
from typing import Dict, List, Tuple, Type

import numpy as np

from vulcanai.console.logger import VulcanAILogger
from vulcanai.tools.embedder import SBERTEmbedder
from vulcanai.tools.tools import CompositeTool, ITool


def vulcanai_tool(cls: Type[ITool]):
    """Class decorator to mark a class as a VulcanAI tool."""
    if not issubclass(cls, ITool):
        raise TypeError(f"{cls.__name__} must inherit from ITool")
    setattr(cls, "__is_vulcanai_tool__", True)
    return cls


class HelpTool(ITool):
    """A tool that provides help information."""

    name = "help"
    tool_description = (
        "Provides help information for using the library. It can list all available tools or"
        " give info about the usage of a specific tool if 'tool_name' is provided as an argument."
    )
    description = (
        "Provides help information for using the library. It can list all available tools or"
        " give info about the usage of a specific tool if 'tool_name' is provided as an argument."
    )
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

    def __init__(self, embedder=None, logger=None, default_tools=True):
        # Logging function from the class VulcanConsole
        self.logger = logger or VulcanAILogger.default()
        # Dictionary of tools (name -> tool instance)
        self.tools: Dict[str, ITool] = {}
        # Dictionary of deactivated_tools (name -> tool instance)
        self.deactivated_tools: Dict[str, ITool] = {}
        # Embedding model for tool metadata
        self.embedder = embedder or SBERTEmbedder(logger=self.logger)
        # Simple in-memory index of (name, embedding)
        self._index: list[Tuple[str, np.ndarray]] = []
        # List of modules where tools can be loaded from
        self._loaded_modules: list[ModuleType] = []
        # Add help_tool to registry but not to index
        self.help_tool = HelpTool()
        self.tools[self.help_tool.name] = self.help_tool
        # Validation tools list to retrieve validation tools separately
        self.validation_tools: List[str] = []

        # Default tools
        if default_tools:
            try:
                self.discover_tools_from_entry_points("ros2_default_tools")
            except ImportError as e:
                self.logger.log_msg(f"[error]{e}[/error]")
                raise

    def register_tool(self, tool: ITool, solve_deps: bool = True, log: bool = True):
        """Register a single tool instance."""
        # Avoid duplicates
        if tool.name in self.tools:
            return

        self.tools[tool.name] = tool
        if tool.is_validation_tool:
            self.validation_tools.append(tool.name)
        emb = self.embedder.embed(self._doc(tool))
        self._index.append((tool.name, emb))
        if log:
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
            self.logger.log_registry(
                f"Tool [registry]'{tool_name}'[/registry] " + "not found in the deactivated tools list.", error=True
            )
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
            self.logger.log_registry(
                f"Tool [registry]'{tool_name}'[/registry] " + "not found in the active tools list.", error=True
            )
            return False

        # Add the tool to the deactivated tools
        self.deactivated_tools[tool_name] = self.tools[tool_name]

        # Removed the tool from the active tools
        del self.tools[tool_name]

        return True

    def register(self):
        """Register all loaded classes marked with @vulcanai_tool."""
        before = set(self.tools.keys())
        composite_classes = []
        for module in self._loaded_modules:
            for name in dir(module):
                tool = getattr(module, name, None)
                if isinstance(tool, type) and issubclass(tool, ITool):
                    if getattr(tool, "__is_vulcanai_tool__", False):
                        # Skip tools with wrong attributes, they would fail when executed
                        if not self.check_tool_class(tool):
                            continue
                        if issubclass(tool, CompositeTool):
                            composite_classes.append(tool)
                        else:
                            instance = self._instantiate_tool(tool)
                            if instance is not None:
                                self.register_tool(instance, solve_deps=False, log=False)
        # Register composite tools after atomic ones to resolve dependencies
        for tool_cls in composite_classes:
            tool = self._instantiate_tool(tool_cls)
            if tool is not None:
                self.register_tool(tool, solve_deps=True, log=False)

        newly_registered = [name for name in self.tools if name not in before]
        self._log_tools_grouped(newly_registered)

    def _get_tool_by_name(self, tool_name: str) -> ITool | None:
        """Return a tool whether it is currently active or deactivated."""
        return self.tools.get(tool_name) or self.deactivated_tools.get(tool_name)

    def _resolve_dependencies(self, tool: CompositeTool):
        """Resolve and attach dependencies for a CompositeTool."""
        for dep_name in tool.dependencies:
            dep_tool = self.tools.get(dep_name)
            if dep_tool is None:
                self.logger.log_registry(
                    f"ERROR. Dependency '{dep_name}' for tool '{tool.name}' not found.", error=True
                )
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
            # Highlight the file name and the failing line, not the whole path
            location = self._error_location(e, path)
            self.logger.log_registry(
                f"Could not load tools from {self._highlight_file(path)}{location}: {e}", error=True
            )

    @staticmethod
    def _error_location(error: Exception, path) -> str:
        """Return ' (line N)' with the line of the tools file where 'error' was raised, or ''."""
        lineno = None
        # Syntax errors are raised by the compiler, so the line is not in the traceback
        if isinstance(error, SyntaxError) and error.filename == str(path):
            lineno = error.lineno
        else:
            # Last frame inside the tools file (errors can also come from modules it imports)
            for frame in traceback.extract_tb(error.__traceback__):
                if frame.filename == str(path):
                    lineno = frame.lineno
        return f" [error](line {lineno})[/error]" if lineno else ""

    @staticmethod
    def _highlight_file(path) -> str:
        """Return the path with only the file name highlighted in red."""
        if not path:
            return "<unknown file>"
        path = Path(path)
        return f"{path.parent}/[error]{path.name}[/error]"

    @staticmethod
    def _class_source_info(cls):
        """
        Return (file, 'class' line, {attribute: ast node}, line offset) of a tool class,
        used to point to the line of a wrong attribute. None if there is no source.
        """
        try:
            file = inspect.getsourcefile(cls)
            lines, start = inspect.getsourcelines(cls)
            class_node = ast.parse(textwrap.dedent("".join(lines))).body[0]
        except (OSError, TypeError, SyntaxError, IndexError):
            return None

        attrs = {}
        for node in getattr(class_node, "body", []):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        attrs[target.id] = node
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                attrs[node.target.id] = node
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                attrs[node.name] = node
        # Node lines are relative to the class source, which starts at its decorators
        offset = start - 1
        return file, offset + class_node.lineno, attrs, offset

    @staticmethod
    def _attr_line(source_info, attr: str, index: int = None):
        """Line of 'attr' in the class (or of its 'index'-th entry), else the class line."""
        if source_info is None:
            return None
        _, class_line, attrs, offset = source_info
        node = attrs.get(attr)
        if node is None:
            return class_line
        line = node.lineno
        value = getattr(node, "value", None)
        if index is not None:
            if isinstance(value, (ast.List, ast.Tuple)) and index < len(value.elts):
                line = value.elts[index].lineno
            elif isinstance(value, ast.Dict) and index < len(value.keys) and value.keys[index] is not None:
                line = value.keys[index].lineno
        return offset + line

    def check_tool_class(self, cls) -> bool:
        """
        Check the attributes of a tool class before registering it, so a wrong
        definition is reported with its line instead of failing at execution time.

        Errors make the tool be skipped. Warnings are only reported.
        Return True if the tool can be registered.
        """
        # Types the executor can cast plan arguments to
        from vulcanai.core.executor import TYPE_CAST

        errors = []  # (attribute, entry index, message)
        warnings = []

        def type_name_of(value) -> str:
            return type(value).__name__

        name = getattr(cls, "name", None)
        if not isinstance(name, str) or not name.strip():
            errors.append(("name", None, f"'name' must be a non-empty string, got {name!r}"))

        description = getattr(cls, "description", None)
        if not isinstance(description, str) or not description.strip():
            errors.append(("description", None, f"'description' must be a non-empty string, got {description!r}"))

        tags = getattr(cls, "tags", [])
        if not isinstance(tags, (list, tuple)):
            errors.append(("tags", None, f"'tags' must be a list of strings, got {type_name_of(tags)}"))
        else:
            for i, tag in enumerate(tags):
                if not isinstance(tag, str):
                    errors.append(("tags", i, f"tag {i} must be a string, got {tag!r}"))

        input_keys = []
        input_schema = getattr(cls, "input_schema", [])
        if not isinstance(input_schema, (list, tuple)):
            errors.append(
                (
                    "input_schema",
                    None,
                    "'input_schema' must be a list of (\"key\", \"type\") tuples, "
                    + f"got {type_name_of(input_schema)}",
                )
            )
        else:
            for i, entry in enumerate(input_schema):
                if not isinstance(entry, (list, tuple)) or len(entry) != 2:
                    errors.append(
                        ("input_schema", i, f"input {i} must be a (\"key\", \"type\") tuple, got {entry!r}")
                    )
                    continue
                key, schema_type = entry
                if not isinstance(key, str) or not key:
                    errors.append(("input_schema", i, f"input {i} key must be a non-empty string, got {key!r}"))
                elif key in input_keys:
                    errors.append(("input_schema", i, f"input '{key}' is defined more than once"))
                else:
                    input_keys.append(key)
                if not isinstance(schema_type, str):
                    errors.append(("input_schema", i, f"type of input '{key}' must be a string, got {schema_type!r}"))
                elif schema_type.removesuffix("?") not in TYPE_CAST:
                    warnings.append(
                        (
                            "input_schema",
                            i,
                            f"input '{key}' has unknown type '{schema_type}', its value is passed as a string. "
                            + f"Known types: {', '.join(TYPE_CAST)} (add '?' for optional inputs)",
                        )
                    )

        input_defaults = getattr(cls, "input_defaults", {})
        if not isinstance(input_defaults, dict):
            errors.append(
                ("input_defaults", None, f"'input_defaults' must be a dict, got {type_name_of(input_defaults)}")
            )
        elif isinstance(input_schema, (list, tuple)):
            for i, key in enumerate(input_defaults):
                if key not in input_keys:
                    errors.append(("input_defaults", i, f"default '{key}' is not an input of 'input_schema'"))

        output_schema = getattr(cls, "output_schema", {})
        if not isinstance(output_schema, dict):
            errors.append(
                (
                    "output_schema",
                    None,
                    "'output_schema' must be a dict like {\"key\": \"type\"}, "
                    + f"got {type_name_of(output_schema)} {output_schema!r}",
                )
            )
        else:
            for i, (key, schema_type) in enumerate(output_schema.items()):
                if not isinstance(key, str) or not isinstance(schema_type, str):
                    errors.append(
                        ("output_schema", i, f"output {key!r}: {schema_type!r} must map a string key to a string type")
                    )

        if issubclass(cls, CompositeTool):
            dependencies = getattr(cls, "dependencies", [])
            if not isinstance(dependencies, (list, tuple)) or not all(isinstance(d, str) for d in dependencies):
                errors.append(("dependencies", None, f"'dependencies' must be a list of tool names, got {dependencies!r}"))

        missing = sorted(getattr(cls, "__abstractmethods__", ()))
        if missing:
            errors.append(("run", None, f"missing implementation of {', '.join(m + '()' for m in missing)}"))

        if not errors and not warnings:
            return True

        source_info = self._class_source_info(cls)
        file_str = self._highlight_file(source_info[0] if source_info else None)
        label = name if isinstance(name, str) and name else cls.__name__

        def location(attr, index) -> str:
            line = self._attr_line(source_info, attr, index)
            return f" [error](line {line})[/error]" if line else ""

        for attr, index, msg in warnings:
            self.logger.log_registry(
                f"[warning]Warning[/warning] in tool '{label}' {file_str}{location(attr, index)}: {msg}"
            )
        for attr, index, msg in errors:
            self.logger.log_registry(f"Invalid tool '{label}' {file_str}{location(attr, index)}: {msg}", error=True)
        if errors:
            self.logger.log_registry(f"Tool '{label}' not registered.", error=True)
            return False
        return True

    def _instantiate_tool(self, cls):
        """Create the tool instance, reporting the failing line instead of stopping the registration."""
        try:
            return cls()
        except Exception as e:
            file = inspect.getsourcefile(cls) if inspect.isclass(cls) else None
            self.logger.log_registry(
                f"Could not create tool '{getattr(cls, 'name', cls.__name__)}' {self._highlight_file(file)}"
                + f"{self._error_location(e, file)}: {e}",
                error=True,
            )
            return None

    def discover_tools_from_file(self, path: str):
        """Load tools from a Python file and register them."""
        self._load_tools_from_file(path)
        self.register()
        self.help_tool.available_tools = self.tools

    def discover_tools_from_entry_points(self, group: str = "custom_tools"):
        """Load tools from Python entry points."""
        eps = entry_points()
        group_eps = list(eps.select(group=group)) if hasattr(eps, "select") else list(eps.get(group, []))
        for ep in group_eps:
            try:
                module = importlib.import_module(ep.module)
                self._loaded_modules.append(module)
            except Exception as e:
                self.logger.log_registry(
                    f"Failed importing EP {ep.name} ({ep.value}){self.logger.exception_location(e)}: {e!r}", error=True
                )
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
                f"No matching tools for the requested mode ({'validation' if validation else 'action'}).", error=True
            )
            return []
        # If k > number of ALL tools, return required tools
        if k > len(self._index):
            return [self.tools[name] for name in active_names]

        filtered_index = [(name, vec) for (name, vec) in self._index if name in active_names]
        if not filtered_index:
            # Index might be stale; log and return []

            self.logger.log_registry("Index has no entries for the selected tool subset.", error=True)
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
        input_defaults = getattr(tool, "input_defaults", {}) or {}
        inputs = []
        for key, type_name in tool.input_schema:
            optional = isinstance(type_name, str) and type_name.endswith("?")
            base_type = type_name[:-1] if optional else type_name
            role = "optional" if optional else "required"
            default = f", default={input_defaults[key]}" if key in input_defaults else ""
            inputs.append(f"{key}:{base_type}:{role}{default}")
        return f"{tool.name}\n{tool.description}\n{tool.tags}\n{inputs}\n"

    def group_tool_names(self, tool_names: list) -> list:
        """Group tool names by each tools `group_name` attribute.

        Tools that declare a `group_name` are collected under that group; tools
        without the attribute are emitted standalone. Group metadata is looked
        up from both active and deactivated tools so `/edit_tools` can preserve
        group structure when an entire group is disabled. Registration order is
        preserved, with each group emitted on the first occurrence of one of
        its members.

        Returns a list of (name, subtools) tuples:
        - For groups: (group_name, [sorted subtool display names])
        - For standalone tools: (tool_name, None)
        """
        result = []
        group_tools: dict = {}
        order = []

        for name in tool_names:
            tool = self._get_tool_by_name(name)
            group = getattr(tool, "group_name", None) if tool is not None else None
            if group:
                if group not in group_tools:
                    group_tools[group] = []
                    order.append(("group", group))
                display = name[len(group) + 1 :] if name.startswith(group + "_") else name
                group_tools[group].append(display)
            else:
                order.append(("tool", name))

        for kind, item in order:
            if kind == "tool":
                result.append((item, None))
            else:
                result.append((item, sorted(group_tools[item])))

        return result

    def _log_tools_grouped(self, tool_names: list):
        """Log a batch of newly registered tools, grouped by common prefix."""
        grouped = self.group_tool_names(tool_names)
        for entry_name, subtools in grouped:
            if subtools is None:
                self.logger.log_registry(f"Registered tool: [registry]{entry_name}[/registry]")
            else:
                self.logger.log_registry(f"Registered group of tools: [registry]{entry_name}[/registry]")
                for subtool in subtools:
                    self.logger.log_registry(
                        f"Registered tool: [registry]{subtool}[/registry]",
                        indent=1,
                    )
