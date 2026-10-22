# vulcanai_mcp_server.py
#
# Exposes VulcanAI's tools to Claude over MCP. Claude itself writes the GlobalPlan;
# VulcanAI only validates and executes it (no local LLM involved).
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
from pathlib import Path
from typing import Literal

# Everything written to stderr (VulcanAI log events, ROS logs, tool prints) goes to this file,
# which the VulcanAI viewer console (vulcanai_mcp_viewer.py) follows live in its own terminal.
# stdout stays reserved for the MCP protocol.
# One file per server process, so several Claude sessions do not overwrite each other's log
MCP_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = MCP_DIR.parents[2]
LOG_DIR = MCP_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_FILE = LOG_DIR / f"vulcanai_mcp_{os.getpid()}.log"
# Remove the logs of servers that are no longer running
for _old in LOG_DIR.glob("vulcanai_mcp_*.log"):
    _pid = _old.stem.rsplit("_", 1)[-1]
    if _pid.isdigit() and not Path(f"/proc/{_pid}").exists():
        _old.unlink(missing_ok=True)
_log_fd = os.open(LOG_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
os.dup2(_log_fd, 2)


VIEWER_SCRIPT = "vulcanai_mcp_viewer.py"


def _viewer_pids() -> list[int]:
    """PIDs of the VulcanAI viewer consoles attached to this server."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                args = f.read().decode(errors="replace").split("\0")
        except OSError:
            continue
        if any(a.endswith(VIEWER_SCRIPT) for a in args) and str(os.getpid()) in args:
            pids.append(int(entry))
    return pids


def _open_viewer() -> str:
    """Open a terminal window with the VulcanAI viewer console. Returns a status message."""
    if _viewer_pids():
        return "The VulcanAI console is already open."
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return "Cannot open the VulcanAI console: no graphical display available."
    viewer = MCP_DIR / VIEWER_SCRIPT
    # If the viewer crashes, keep the window open to show the error
    script = (
        shlex.join(
            [
                "env",
                f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
                sys.executable,
                str(viewer),
                str(LOG_FILE),
                "--pid",
                str(os.getpid()),
            ]
        )
        + " || { echo; echo '[VulcanAI viewer failed. Press Enter to close]'; read; }"
    )
    if shutil.which("gnome-terminal"):
        cmd = ["gnome-terminal", "--title=VulcanAI (Claude)", "--", "bash", "-c", script]
    elif shutil.which("x-terminal-emulator"):
        cmd = ["x-terminal-emulator", "-e", "bash", "-c", script]
    else:
        return "Cannot open the VulcanAI console: no terminal emulator found."
    subprocess.Popen(
        cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
    )
    return "VulcanAI console opened."


def _close_viewer() -> str:
    """Close the viewer consoles (they exit cleanly on SIGTERM, closing their window)."""
    pids = _viewer_pids()
    if not pids:
        return "The VulcanAI console is not open."
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    return "VulcanAI console closed."


# The console is closed by default: the user opens it with the vulcanai_console tool.
# Set VULCANAI_VIEWER=1 to open it at startup.
if os.environ.get("VULCANAI_VIEWER", "0") == "1":
    _open_viewer()

# The turtlesim demo is not colcon-built: import it straight from source.
DEMO_DIR = WORKSPACE_DIR / "vulcanexus/code/vulcanai_turtlesim/vulcanai_turtlesim_demo"
sys.path.insert(0, str(DEMO_DIR))

# These imports need DEMO_DIR in sys.path, so they cannot go at the top (E402)
import anyio  # noqa: E402
import rclpy  # noqa: E402
from mcp.server.mcpserver import MCPServer  # noqa: E402
from vulcanai_turtlesim_demo.ros2_node import SharedNode  # noqa: E402

from vulcanai.console.logger import VulcanAILogger  # noqa: E402
from vulcanai.core.executor import Blackboard, PlanExecutor  # noqa: E402
from vulcanai.core.plan_types import GlobalPlan  # noqa: E402
from vulcanai.core.validator import PlanValidator  # noqa: E402
from vulcanai.tools.tool_registry import ToolRegistry  # noqa: E402

EVENT_MARKER = "\x1eVULCANAI "  # must match vulcanai_mcp_viewer.py


def emit_event(event_type: str, **data):
    """Write an event line to stderr (the log file) for the VulcanAI viewer console."""
    sys.stderr.write(EVENT_MARKER + json.dumps({"type": event_type, **data}) + "\n")
    sys.stderr.flush()


class ViewerSink:
    def write(self, msg, color=""):
        emit_event("log", msg=msg, color=color)


# Keep VulcanAI's own <tag> markup (what the Textual console renders) instead of Rich markup
VulcanAILogger._rich_markup = False
logger = VulcanAILogger(sink=ViewerSink())
VulcanAILogger._default_instance = logger  # anything calling .default() also goes to the viewer

rclpy.init()
node = SharedNode("vulcanai_mcp_node")

registry = ToolRegistry(logger=logger, default_tools=True)
registry.discover_tools_from_file(str(DEMO_DIR / "vulcanai_turtlesim_demo/turtlesim_tools.py"))
validator = PlanValidator(registry)
executor = PlanExecutor(registry, logger=logger)
# Persists across calls, so a plan can reference outputs of earlier plans ({{bb.tool.key}})
bb = Blackboard()
bb["main_node"] = node
_lock = threading.Lock()
logger.log_msg(
    f"VulcanAI MCP server ready (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}). "
    "Waiting for plans from Claude...",
    color="console",
)

mcp = MCPServer(
    "vulcanai",
    instructions=(
        "Controls a ROS 2 turtlesim simulation through VulcanAI. "
        "Call list_vulcanai_tools first to learn the tools and their arguments, "
        "then send a GlobalPlan to execute_vulcanai_plan."
    ),
)


def _clean_bb(data):
    out = {}
    for k, v in data.items():
        if k in ("main_node", "console"):
            continue
        try:
            json.dumps(v)
            out[k] = v
        except TypeError:
            out[k] = repr(v)
    return out


@mcp.tool()
def list_vulcanai_tools() -> str:
    """List the VulcanAI robot tools with their inputs and outputs.
    Input types ending in '?' are optional. Call this before building a plan."""
    logger.log_msg("Claude requested the tool list", color="console")
    tools = []
    for t in registry.tools.values():
        if t.name == "help":
            continue
        tools.append(
            {
                "name": t.name,
                "description": t.description,
                "inputs": [
                    {"key": k, "type": ty.rstrip("?"), "required": not ty.endswith("?")} for k, ty in t.input_schema
                ],
                "defaults": getattr(t, "input_defaults", {}) or {},
                "outputs": t.output_schema,
            }
        )
    return json.dumps(tools, indent=1)


@mcp.tool()
def vulcanai_console(action: Literal["open", "close", "status"]) -> str:
    """Open or close the VulcanAI terminal console window, which shows live the plans Claude sends,
    the tool executions and the ROS logs. Use "status" to check whether it is open.
    Only open or close it when the user asks for it."""
    if action == "open":
        msg = _open_viewer()
    elif action == "close":
        msg = _close_viewer()
    else:
        msg = "The VulcanAI console is open." if _viewer_pids() else "The VulcanAI console is closed."
    logger.log_msg(f"vulcanai_console({action}): {msg}", color="console")
    return msg


@mcp.tool()
async def execute_vulcanai_plan(plan: GlobalPlan) -> str:
    """Validate and execute a VulcanAI GlobalPlan on the robot.

    - plan: list of PlanNodes, run in order. Each PlanNode has kind "SEQUENCE" or "PARALLEL"
      and a list of steps; each Step names a tool and its args as [{"key": ..., "val": ...}].
    - Give numeric/bool args as JSON numbers/booleans, not strings.
    - To use the output of an earlier step, set val to "{{bb.<tool_name>.<output_key>}}".
    - condition / success_criteria / timeout_ms / retry / on_fail are optional; only use them when needed.
    Returns whether it succeeded and the outputs of each tool (the blackboard)."""
    emit_event("plan_start", summary=plan.summary or "")
    logger.log_user(plan.summary or "(plan from Claude)")
    logger.log_manager(f"Plan received from Claude:\n{plan}")
    try:
        validator.validate(plan)
    except Exception as e:
        logger.log_validator(f"Plan validation: {e}", error=True)
        emit_event("plan_end")
        return json.dumps({"success": False, "error": f"Invalid plan: {e}"})

    def _run():
        with _lock:
            return executor.run(plan, bb)

    try:
        res = await anyio.to_thread.run_sync(_run)
    except Exception as e:
        logger.log_manager(f"Error executing plan: {e}", error=True)
        emit_event("plan_end")
        return json.dumps({"success": False, "error": str(e)})
    results = _clean_bb(res.get("blackboard", {}))
    logger.log_manager(f"Plan finished. success={res.get('success')}\n{json.dumps(results, default=str)}")
    logger.log_msg("─" * 60)
    emit_event("plan_end")
    return json.dumps({"success": res.get("success"), "results": results}, default=str)


if __name__ == "__main__":
    mcp.run()  # stdio
