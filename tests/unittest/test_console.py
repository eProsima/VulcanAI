import importlib.util
import os
import re
import sys
import time
import types
import unittest
from unittest.mock import patch

from textual.widgets import Input

from vulcanai.console.console import VulcanConsole
from vulcanai.console.logger import VulcanAILogger
from vulcanai.core.executor import Blackboard


# Stub sentence_transformers to avoid heavyweight imports when loading tools from files
class _DummySentenceTransformer:
    def __init__(self, *args, **kwargs):
        pass

    def encode(self, text, convert_to_numpy=True):
        return None

    def similarity(self, a, b):
        return None


sys.modules.setdefault("sentence_transformers", types.SimpleNamespace(SentenceTransformer=_DummySentenceTransformer))


# Make src-layout importable
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, os.path.pardir, "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)
RESOURCES_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.path.pardir, "resources"))

# region TOOLS

tools_mod = importlib.import_module("vulcanai.tools.tools")


class FakeTool(tools_mod.AtomicTool):
    """Small tool descriptor used by FakeRegistry for /tools output tests."""

    def __init__(self, name: str, description: str = "Fake tool description",
                 is_validation_tool=True, provide_images=True):
        self.name = name
        self.description = description
        self.tags = ["empty", "fake", "tool", "move"]
        self.input_schema = [("input", "string")]
        self.output_schema = {"output": "bool"}
        self.version = "0.1"
        self.is_validation_tool = is_validation_tool
        self.provide_images = provide_images

    def run(self, **kwargs):
        return {"arrived": True}


# endregion

# region LLM

class FakeLLM:
    def __init__(self, logger=None):
        self.logger = logger

        self.goal_return = None
        self.goal_error = None
        self.validation_return = None
        self.validation_error = None

    def inference_goal(self, system_prompt, user_prompt, history):
        if self.goal_error:
            raise self.goal_error
        return self.goal_return

    def inference_validation(self, system_prompt, user_prompt, images, history):
        if self.validation_error:
            raise self.validation_error
        return self.validation_return

    def set_hooks(self, _hooks) -> None:
        self.logger.log_manager("LLM hooks set.")

# region REGISTRY


class FakeRegistry:
    """In-memory registry used to avoid real tool discovery in most command tests."""

    def __init__(self):
        self.tools = {
            "nav": FakeTool("nav", "Navigate to target pose."),
            "scan": FakeTool("scan", "Scan surroundings."),
        }
        self.deactivated_tools = {
            "old_tool": FakeTool("old_tool", "Deprecated tool."),
        }

    def activate_tool(self, _tool) -> bool:
        return True

    def deactivate_tool(self, _tool) -> bool:
        return True


class FileToolsRegistry:
    """Registry that discovers @vulcanai_tool classes from a Python file."""

    def __init__(self):
        self.tools = {}
        self.deactivated_tools = {}

    def discover_tools_from_file(self, path: str) -> None:
        spec = importlib.util.spec_from_file_location(f"file_tools_{id(self)}", path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot import {path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        for symbol_name in dir(module):
            symbol = getattr(module, symbol_name)
            if isinstance(symbol, type) and getattr(symbol, "__is_vulcanai_tool__", False):
                tool = symbol()
                self.tools[tool.name] = tool

    def activate_tool(self, _tool) -> bool:
        return True

    def deactivate_tool(self, _tool) -> bool:
        return True


class ThreeToolsStatefulRegistry:
    """In-memory registry with real activate/deactivate state transitions."""

    def __init__(self):
        self.tools = {
            "nav": FakeTool("nav", "Navigate to target pose."),
            "scan": FakeTool("scan", "Scan surroundings."),
            "pick": FakeTool("pick", "Pick an object."),
        }
        self.deactivated_tools = {}

    def activate_tool(self, tool_name) -> bool:
        if tool_name in self.tools:
            return False
        if tool_name not in self.deactivated_tools:
            return False
        self.tools[tool_name] = self.deactivated_tools[tool_name]
        del self.deactivated_tools[tool_name]
        return True

    def deactivate_tool(self, tool_name) -> bool:
        if tool_name in self.deactivated_tools:
            return False
        if tool_name not in self.tools:
            return False
        self.deactivated_tools[tool_name] = self.tools[tool_name]
        del self.tools[tool_name]
        return True


# region MANAGER


class FakeManager:
    """Minimal manager API expected by VulcanConsole command handlers."""

    def __init__(self, model: str, k: int, logger):
        self.k = k
        self.logger = logger
        self.history = []
        self.history_depth = 0
        self.bb = {}
        self.registry = FakeRegistry()
        self.llm = FakeLLM(logger)
        self.logger.log_manager(f"Using OpenAI API with model: {model}")

    def register_tools_from_file(self, _tool_file_path: str) -> None:
        return

    def register_tools_from_entry_points(self, _entry_point: str) -> None:
        return

    def add_user_context(self, _context: str) -> None:
        return

    def update_k_index(self, value: int) -> None:
        self.k = value

    def update_history_depth(self, value: int) -> None:
        self.history_depth = value


class QuietFakeManager(FakeManager):
    """Fake manager variant that avoids logging during __init__."""

    def __init__(self, model: str, k: int, logger):
        class _FakeExecutor:
            def run(self, _plan, _bb):
                return {"blackboard": {"ok": True}}

        self.k = k
        self.logger = logger
        self.history = []
        self.history_depth = 0
        self.bb = {}
        self.registry = FakeRegistry()
        self.llm = FakeLLM(logger)
        self.executor = _FakeExecutor()


class FileToolsFakeManager(FakeManager):
    """Fake manager that supports file-based tool discovery and query logging."""

    def __init__(self, model: str, k: int, logger):
        super().__init__(model=model, k=k, logger=logger)
        self.registry = FileToolsRegistry()

    def register_tools_from_file(self, path: str) -> None:
        self.registry.discover_tools_from_file(path)

    def handle_user_request(self, user_text: str, context):
        self.logger.log_manager(f"Fake LLM received query: {user_text}")
        bb = Blackboard()
        bb["fake_query"] = user_text
        return {"plan": {"summary": "fake"}, "success": True, "blackboard": bb}


class QuietFileToolsFakeManager(FileToolsFakeManager):
    """File-tools manager variant without threaded logging side effects."""

    def __init__(self, model: str, k: int, logger):
        self.k = k
        self.logger = logger
        self.history = []
        self.history_depth = 0
        self.bb = {}
        self.registry = FileToolsRegistry()
        self.llm = FakeLLM(logger)

    def handle_user_request(self, user_text: str, context):
        bb = Blackboard()
        bb["fake_query"] = user_text
        return {"plan": {"summary": "fake"}, "success": True, "blackboard": bb}


# region TESTS

def normalize_log(msg: str) -> str:
    """
    Remove rich/style markup while preserving human-readable content.
    """
    ret = re.sub(r"\[/?#[0-9a-fA-F]{6}\]", "", msg)
    ret = re.sub(r"\[/?(?:bold|italic|underline|reverse|dim)\]", "", ret)
    ret = re.sub(r"<[^>]+>", "", ret)
    ret = re.sub(r"\s+", " ", ret).strip()
    return ret


class FakeSink:
    def __init__(self):
        self.messages = []

    def write(self, msg: str, color: str = "") -> None:
        self.messages.append(msg)


# tests helper
class StartupTestVulcanConsole(VulcanConsole):
    """Console variant that boots deterministically for command/startup tests."""

    async def bootstrap(self) -> None:
        # -- Initialize manager -----------------------------------------------

        # Emit the same user-facing startup lines the real app emits.
        self.logger.log_console("Initializing Manager <bold>'PlanManager'</bold>...")
        self.manager = FakeManager(model=self.model, k=self.k, logger=self.logger)
        self.logger.log_console(f"Manager initialized with model <bold>'{self.model.replace('ollama-', '')}</bold>'")
        self._update_variables_panel()

        # -- Add the commands (non-blocking, runs in event loop) --------------

        self.commands = {
            "/help": self.cmd_help,
            "/tools": self.cmd_tools,
            "/edit_tools": self.cmd_edit_tools,
            "/change_k": self.cmd_change_k,
            "/history": self.cmd_history_index,
            "/show_history": self.cmd_show_history,
            "/clear_history": self.cmd_clear_history,
            "/plan": self.cmd_plan,
            "/rerun": self.cmd_rerun,
            "/bb": self.cmd_blackboard_state,
            "/clear": self.cmd_clear,
            "/exit": self.cmd_quit,
        }

        # Tab matches initialization
        self.tab_matches = []
        self.tab_index = 0

        # -- Spinner controller --
        try:
            self.manager.llm.set_hooks(self.hooks)
        except Exception:
            pass

        # Add user context (non-blocking)
        self.manager.add_user_context(self.user_context)
        # Add console to blackboard
        self.manager.bb["console"] = self

        self.is_ready = True
        self.logger.log_console("VulcanAI Interactive Console")
        self.logger.log_console("Use <bold>'Ctrl+Q'</bold> to quit.")

        # Activate the terminal input
        self.set_input_enabled(True)

    def cmd_rerun(self, args) -> None:
        # Test-safe /rerun implementation that avoids threaded worker complexity.
        selected_plan = 0
        if len(args) == 0:
            # No index specified. Last plan selected
            selected_plan = len(self.plans_list) - 1
        elif len(args) != 1 or not args[0].isdigit():
            self.logger.log_console("Usage: /rerun 'int'")
            return
        else:
            selected_plan = int(args[0])
            if selected_plan < -1:
                self.logger.log_console("Usage: /rerun 'int' [int >= -1].")
                return

        if not self.plans_list:
            self.logger.log_console("No plan to rerun.")
            return

        self.logger.log_console(f"Rerunning {selected_plan}-th plan...")
        # Simulate rerun of the latest plan
        self.logger.log_console("Output of rerun: {'ok': True}")


# tests helper
class FileToolsConsole(VulcanConsole):
    """Console variant that loads tools from file and handles a fake user query."""

    async def bootstrap(self) -> None:
        self.manager = FileToolsFakeManager(model=self.model, k=self.k, logger=self.logger)
        self.commands = {
            "/help": self.cmd_help,
            "/tools": self.cmd_tools,
            "/edit_tools": self.cmd_edit_tools,
            "/change_k": self.cmd_change_k,
            "/history": self.cmd_history_index,
            "/show_history": self.cmd_show_history,
            "/clear_history": self.cmd_clear_history,
            "/plan": self.cmd_plan,
            "/rerun": self.cmd_rerun,
            "/bb": self.cmd_blackboard_state,
            "/clear": self.cmd_clear,
            "/exit": self.cmd_quit,
        }
        self.tab_matches = []
        self.tab_index = 0

        for tool_file_path in self.register_from_file:
            self.manager.register_tools_from_file(tool_file_path)

        self.manager.llm.set_hooks(self.hooks)
        self.manager.add_user_context(self.user_context)
        self.manager.bb["console"] = self

        self.is_ready = True
        self.logger.log_console("VulcanAI Interactive Console")
        self.logger.log_console("Use <bold>'Ctrl+Q'</bold> to quit.")
        self.set_input_enabled(True)

# TERMINAL INPUT
class TestConsoleInputOutput(unittest.IsolatedAsyncioTestCase):
    """End-to-end Textual UI tests focused on terminal output behavior."""

    def _new_lines(self, app, start_index: int) -> list[str]:
        """Return only the lines appended after a known index."""
        return app.left_pannel.document.text.splitlines()[start_index:]

    def _assert_new_lines_contains(self, app, start_index: int, expected_text: str) -> None:
        """Assertion helper to check incremental log output."""
        new_lines = self._new_lines(app, start_index)
        self.assertTrue(
            any(expected_text in line for line in new_lines),
            f"Expected '{expected_text}' in new lines, got: {new_lines}",
        )

    @staticmethod
    def _fake_init_manager_for_real_console(console) -> None:
        # Keep startup behavior deterministic without external manager dependencies.
        console.call_from_thread(console.logger.log_console, "Initializing Manager <bold>'PlanManager'</bold>...")
        console.manager = QuietFakeManager(model=console.model, k=console.k, logger=console.logger)
        console.call_from_thread(console.logger.log_manager, f"Using OpenAI API with model: {console.model}")
        console.call_from_thread(
            console.logger.log_console,
            f"Manager initialized with model <bold>'{console.model.replace('ollama-', '')}</bold>'",
        )
        console.call_from_thread(console._update_variables_panel)

    @staticmethod
    def _fake_init_manager_for_real_console_file_tools(console) -> None:
        # Deterministic startup with file-tools discovery support.
        console.call_from_thread(console.logger.log_console, "Initializing Manager <bold>'PlanManager'</bold>...")
        console.manager = QuietFileToolsFakeManager(model=console.model, k=console.k, logger=console.logger)
        console.call_from_thread(console.logger.log_manager, f"Using OpenAI API with model: {console.model}")
        console.call_from_thread(
            console.logger.log_console,
            f"Manager initialized with model <bold>'{console.model.replace('ollama-', '')}</bold>'",
        )
        console.call_from_thread(console._update_variables_panel)

    async def test_simple_input(self):
        # Simulate typing into the input widget and verify unknown-command output in log panel.
        # cmd input: /echo hello
        # expected output: "Unknown command: ..."
        app = StartupTestVulcanConsole(model="gpt-5-nano")

        # Textual test runner
        async with app.run_test() as pilot:
            # Lets the event loop process pending startup work
            await pilot.pause()
            # Waits one more cycle so async bootstrap updates finish before assertions.
            await pilot.pause()
            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            await pilot.click("#cmd")
            await pilot.press("/", "e", "c", "h", "o", "space", "h", "e", "l", "l", "o", "enter")
            # Wait for the command to finish
            await pilot.pause()

        lines = self._new_lines(app, start)
        self.assertEqual(lines[0], "[USER] >>> /echo hello")
        self.assertEqual(lines[1], "Unknown command: /echo. Type '/help'.")

    async def test_simple_startup(self):
        # Validate startup (manager initilization)
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        cmd_disabled = True

        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            cmd_disabled = app.query_one("#cmd", Input).disabled

        lines = app.left_pannel.document.text.splitlines()
        expected_subsequence = [
            "Initializing Manager 'PlanManager'...",
            "[MANAGER] Using OpenAI API with model: gpt-5-nano",
            "Manager initialized with model 'gpt-5-nano'",
            "[MANAGER] LLM hooks set.",
            "VulcanAI Interactive Console",
            "Use 'Ctrl+Q' to quit.",
        ]

        line_idx = 0
        for expected in expected_subsequence:
            while line_idx < len(lines) and lines[line_idx] != expected:
                line_idx += 1
            self.assertLess(line_idx, len(lines), f"Missing startup line: {expected}")
            line_idx += 1

        self.assertTrue(app.is_ready)
        self.assertFalse(cmd_disabled)

    async def test_simple_command_help(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/help")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Available commands:")

    async def test_simple_command_tools(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/tools")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Available tools:")
            self._assert_new_lines_contains(app, start, "nav:")
            self._assert_new_lines_contains(app, start, "scan:")

    async def test_simple_command_tools_from_file(self):
        # Ensure tools decorated in tests/resources/test_tools.py are shown by /tools.
        # Then submit a free-text query and assert manager/query output lines.
        app = FileToolsConsole(
            model="gpt-5-nano",
            register_from_file=[os.path.join(RESOURCES_DIR, "test_tools.py")],
        )

        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/tools")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "file_tool:")
            self._assert_new_lines_contains(app, start, "new_file_tool:")
            self._assert_new_lines_contains(app, start, "other_file_tool:")
            self._assert_new_lines_contains(app, start, "another_validation_tool:")

    async def test_simple_command_edit_tools(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            with patch.object(app, "open_checklist") as checklist_mock:
                # Tool testing
                app.handle_command("/edit_tools")
                checklist_mock.assert_called_once()
                call_args = checklist_mock.call_args.args
                self.assertEqual(call_args[0], ["- nav", "- scan", "- old_tool"])
                self.assertEqual(call_args[1], 2)

    async def test_simple_command_tools_deactivate_and_activate(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.manager.registry = ThreeToolsStatefulRegistry()

            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/tools")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Available tools:")
            self._assert_new_lines_contains(app, start, "nav:")
            self._assert_new_lines_contains(app, start, "scan:")
            self._assert_new_lines_contains(app, start, "pick:")

            self.assertTrue(app.manager.registry.deactivate_tool("scan"))
            self.assertIn("scan", app.manager.registry.deactivated_tools)
            self.assertNotIn("scan", app.manager.registry.tools)

            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/tools")
            await pilot.pause()
            deactivated_tools_lines = "\n".join(self._new_lines(app, start))
            self.assertIn("nav:", deactivated_tools_lines)
            self.assertIn("pick:", deactivated_tools_lines)
            self.assertNotIn("scan:", deactivated_tools_lines)

            self.assertTrue(app.manager.registry.activate_tool("scan"))
            self.assertIn("scan", app.manager.registry.tools)
            self.assertNotIn("scan", app.manager.registry.deactivated_tools)

            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/tools")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "nav:")
            self._assert_new_lines_contains(app, start, "scan:")
            self._assert_new_lines_contains(app, start, "pick:")

    async def test_simple_command_change_k(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/change_k")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Current 'k' is 7")

            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/change_k bad")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Usage: /change_k 'int'")

            app.handle_command("/change_k 3")
            await pilot.pause()
            self.assertEqual(app.manager.k, 3)

    async def test_simple_command_history(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/history")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Current 'history depth' is 0")

            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/history bad")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Usage: /history 'int'")

            app.handle_command("/history 4")
            await pilot.pause()
            self.assertEqual(app.manager.history_depth, 4)

    async def test_simple_command_show_history(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.manager.history = []

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/show_history")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "No history available.")

            app.manager.history = [("ignored_line\nPick object", "Sample plan summary")]
            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/show_history")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Current history:")
            self._assert_new_lines_contains(app, start, "[USER] >>> Pick object")
            self._assert_new_lines_contains(app, start, "Plan summary: Sample plan summary")

    async def test_simple_command_clear_history(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.history = ["/help", "/tools"]
            app.history_index = 1
            app.plans_list = [{"plan": 1}]

            # Tool testing
            app.handle_command("/clear_history")
            await pilot.pause()

            self.assertEqual(app.history, [])
            self.assertIsNone(app.history_index)
            self.assertEqual(app.plans_list, [])
            # Textual log
            self._assert_new_lines_contains(app, 0, "History cleared.")

    async def test_simple_command_plan(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.plans_list = []

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/plan")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "No plan has been generated yet.")

            app.plans_list = [{"step": "demo"}]
            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/plan")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Last generated plan:")
            self._assert_new_lines_contains(app, start, "{'step': 'demo'}")

    async def test_simple_command_rerun(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.plans_list = []

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/rerun")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "No plan to rerun.")

            app.plans_list = [{"step": "demo"}]
            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/rerun")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Rerunning 0-th plan...")
            self._assert_new_lines_contains(app, start, "Output of rerun: {'ok': True}")

    async def test_simple_command_bb(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.last_bb = None

            # Store the Initialization of the manager to check the terminal input logs
            start = len(app.left_pannel.document.text.splitlines())
            # Tool testing
            app.handle_command("/bb")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "No blackboard available.")

            app.last_bb = {"key": "<value>"}
            start = len(app.left_pannel.document.text.splitlines())
            app.handle_command("/bb")
            await pilot.pause()
            # Textual log
            self._assert_new_lines_contains(app, start, "Lastest blackboard state:")
            self._assert_new_lines_contains(app, start, "key")
            self._assert_new_lines_contains(app, start, "value")

    async def test_simple_command_clear(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            app.logger.log_console("Temporary output before clear")
            await pilot.pause()
            self.assertTrue(len(app.left_pannel.document.text) > 0)

            # Tool testing
            app.handle_command("/clear")
            await pilot.pause()
            self.assertEqual(app.left_pannel.document.text, "")

    async def test_simple_command_exit(self):
        app = StartupTestVulcanConsole(model="gpt-5-nano")
        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()
            with patch.object(app, "exit") as exit_mock:
                app.handle_command("/exit")
                exit_mock.assert_called_once()

    async def test_simple_query_output(self):
        # Ensure tools decorated in tests/resources/test_tools.py are shown by /tools.
        # Then submit a free-text query and assert manager/query output lines.
        app = FileToolsConsole(
            model="gpt-5-nano",
            register_from_file=[os.path.join(RESOURCES_DIR, "test_tools.py")],
        )

        async with app.run_test() as pilot:
            await pilot.pause()
            await pilot.pause()

            await pilot.click("#cmd")
            await pilot.press(
                "t",
                "h",
                "i",
                "s",
                "space",
                "i",
                "s",
                "space",
                "a",
                "space",
                "q",
                "u",
                "e",
                "r",
                "y",
                "enter",
            )
            await pilot.pause()
            await pilot.pause()

        lines = app.left_pannel.document.text.splitlines()
        self.assertIn("[USER] >>> this is a query", lines)
        self.assertIn("[MANAGER] Fake LLM received query: this is a query", lines)
        self.assertIn("Output of plan: {'fake_query': 'this is a query'}", lines)

    async def test_vulcanai_input(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                # Lets the event loop process pending startup work
                await pilot.pause()
                # Waits one more cycle so async bootstrap updates finish before assertions.
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                await pilot.click("#cmd")
                await pilot.press("/", "e", "c", "h", "o", "space", "h", "e", "l", "l", "o", "enter")
                # Wait for the command to finish
                await pilot.pause()

        lines = self._new_lines(app, start)
        self.assertEqual(lines[0], "[USER] >>> /echo hello")
        self.assertEqual(lines[1], "Unknown command: /echo. Type '/help'.")

    async def test_vulcanai_startup(self):
        # Validate startup logs using the real VulcanConsole in headless mode.
        app = VulcanConsole(model="gpt-5-nano")
        cmd_disabled = True

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                cmd_disabled = app.query_one("#cmd", Input).disabled

        lines = app.left_pannel.document.text.splitlines()
        expected_subsequence = [
            "Initializing Manager 'PlanManager'...",
            "[MANAGER] Using OpenAI API with model: gpt-5-nano",
            "Manager initialized with model 'gpt-5-nano'",
            "[MANAGER] LLM hooks set.",
            "VulcanAI Interactive Console",
            "Use 'Ctrl+Q' to quit.",
        ]

        line_idx = 0
        for expected in expected_subsequence:
            while line_idx < len(lines) and lines[line_idx] != expected:
                line_idx += 1
            self.assertLess(line_idx, len(lines), f"Missing startup line: {expected}")
            line_idx += 1

        self.assertTrue(app.is_ready)
        self.assertFalse(cmd_disabled)

    async def test_vulcanai_command_help(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/help")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Available commands:")

    async def test_vulcanai_command_tools(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/tools")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Available tools:")
                self._assert_new_lines_contains(app, start, "nav:")
                self._assert_new_lines_contains(app, start, "scan:")

    async def test_vulcanai_command_tools_from_file(self):
        app = VulcanConsole(
            model="gpt-5-nano",
            register_from_file=[os.path.join(RESOURCES_DIR, "test_tools.py")],
        )

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console_file_tools):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/tools")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "file_tool:")
                self._assert_new_lines_contains(app, start, "new_file_tool:")
                self._assert_new_lines_contains(app, start, "other_file_tool:")
                self._assert_new_lines_contains(app, start, "another_validation_tool:")
                self.assertNotIn("no_decorator_tool", app.left_pannel.document.text)

    async def test_vulcanai_command_change_k(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/change_k")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Current 'k' is 7")

                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/change_k bad")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Usage: /change_k 'int'")

                app.handle_command("/change_k 3")
                await pilot.pause()
                self.assertEqual(app.manager.k, 3)

    async def test_vulcanai_command_edit_tools(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                with patch.object(app, "open_checklist") as checklist_mock:
                    app.handle_command("/edit_tools")
                    checklist_mock.assert_called_once()
                    call_args = checklist_mock.call_args.args
                    self.assertEqual(call_args[0], ["- nav", "- scan", "- old_tool"])
                    self.assertEqual(call_args[1], 2)

    async def test_vulcanai_command_history(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/history")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Current 'history depth' is 0")

                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/history bad")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Usage: /history 'int'")

                app.handle_command("/history 4")
                await pilot.pause()
                self.assertEqual(app.manager.history_depth, 4)

    async def test_vulcanai_command_show_history(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.manager.history = []

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/show_history")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "No history available.")

                app.manager.history = [("ignored_line\nPick object", "Sample plan summary")]
                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/show_history")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Current history:")
                self._assert_new_lines_contains(app, start, "[USER] >>> Pick object")
                self._assert_new_lines_contains(app, start, "Plan summary: Sample plan summary")

    async def test_vulcanai_command_clear_history(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.history = ["/help", "/tools"]
                app.history_index = 1
                app.plans_list = [{"plan": 1}]

                app.handle_command("/clear_history")
                await pilot.pause()

                self.assertEqual(app.history, [])
                self.assertIsNone(app.history_index)
                self.assertEqual(app.plans_list, [])
                # Textual log
                self._assert_new_lines_contains(app, 0, "History cleared.")

    async def test_vulcanai_command_plan(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.plans_list = []

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/plan")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "No plan has been generated yet.")

                app.plans_list = [{"step": "demo"}]
                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/plan")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Last generated plan:")
                self._assert_new_lines_contains(app, start, "{'step': 'demo'}")

    async def test_vulcanai_command_rerun(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.plans_list = []

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/rerun")
                await pilot.pause()
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "No plan to rerun.")

                app.plans_list = [{"step": "demo"}]
                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/rerun")
                await pilot.pause()
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Rerunning 0-th plan...")
                self._assert_new_lines_contains(app, start, "Output of rerun: {'ok': True}")

    async def test_vulcanai_command_bb(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.last_bb = None

                # Store the Initialization of the manager to check the terminal input logs
                start = len(app.left_pannel.document.text.splitlines())
                # Tool testing
                app.handle_command("/bb")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "No blackboard available.")

                app.last_bb = {"key": "<value>"}
                start = len(app.left_pannel.document.text.splitlines())
                app.handle_command("/bb")
                await pilot.pause()
                # Textual log
                self._assert_new_lines_contains(app, start, "Lastest blackboard state:")
                self._assert_new_lines_contains(app, start, "key")
                self._assert_new_lines_contains(app, start, "value")

    async def test_vulcanai_command_clear(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                app.logger.log_console("Temporary output before clear")
                await pilot.pause()
                self.assertTrue(len(app.left_pannel.document.text) > 0)

                app.handle_command("/clear")
                await pilot.pause()
                self.assertEqual(app.left_pannel.document.text, "")

    async def test_vulcanai_command_exit(self):
        app = VulcanConsole(model="gpt-5-nano")

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                with patch.object(app, "exit") as exit_mock:
                    app.handle_command("/exit")
                    exit_mock.assert_called_once()

    async def test_vulcanai_query_output(self):
        app = VulcanConsole(
            model="gpt-5-nano",
            register_from_file=[os.path.join(RESOURCES_DIR, "test_tools.py")],
        )

        with patch.object(VulcanConsole, "init_manager", self._fake_init_manager_for_real_console_file_tools):
            async with app.run_test(headless=True) as pilot:
                await pilot.pause()
                await pilot.pause()
                await pilot.click("#cmd")
                await pilot.press(
                    "t",
                    "h",
                    "i",
                    "s",
                    "space",
                    "i",
                    "s",
                    "space",
                    "a",
                    "space",
                    "q",
                    "u",
                    "e",
                    "r",
                    "y",
                    "enter",
                )
                await pilot.pause()
                await pilot.pause()

        lines = app.left_pannel.document.text.splitlines()
        self.assertIn("[USER] >>> this is a query", lines)
        self.assertIn("Output of plan: {'fake_query': 'this is a query'}", lines)

    # TODO.
    # async def test_vulcanai_save_plan(self):
    #
    # async def test_vulcanai_load_plan(self):
    #  gpt-5-nano, ollama-llama3.1:8b
    #  default_tools?

# endregion

