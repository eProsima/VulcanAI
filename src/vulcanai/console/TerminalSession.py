from typing import Any, Protocol, Optional, List
from dataclasses import dataclass
import os
import subprocess
import sys


def write_terminal_sequence(sequence: str) -> None:
    """
    Write a raw escape sequence to the active terminal.

    Used to change the color of the terminal.
    - Change the terminal color using the same color of VulcanAI
    - Restore the terminal color
    """
    if not sys.stdout.isatty():
        return
    try:
        sys.stdout.write(sequence)
        sys.stdout.flush()
    except Exception:
        # best-effort: ignore
        pass


class TerminalAdapter(Protocol):
    """
    Protocol implemented by terminal-specific adapters.

    Parent class for each terminal adapter.
    Done:
    - GNOME

    TODO:
    - Terminator
    - Zsh
    - Kitty
    - XTerm
    - Alackritty
    - Alacritty
    """

    name: str

    def detect(self) -> bool: ...

    def apply(self) -> Any: ...

    def restore(self, state: Any) -> None: ...

# region TERMINALS

    # region gnome

def _run_gsettings(*args: str) -> Optional[str]:
    """
    @brief Run gsettings and return trimmed stdout on success.
    @param args Positional arguments forwarded to ``gsettings``.
    @return Command stdout without trailing whitespace, or ``None`` on failure.
    """
    try:
        completed = subprocess.run(
            ["gsettings", *args],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


@dataclass
class GnomeState:
    """@brief State required to restore GNOME Terminal settings."""

    schema: str
    scrollbar_policy_backup: str


class GnomeTerminalAdapter:
    """@brief GNOME Terminal adapter that hides and restores the scrollbar."""

    name = "gnome-terminal"

    def detect(self) -> bool:
        """
        @brief Detect whether the current terminal is GNOME Terminal.
        @return ``True`` when GNOME Terminal environment markers are found.
        """
        terminal_emulator = os.environ.get("TERMINAL_EMULATOR", "").lower()
        term_program = os.environ.get("TERM_PROGRAM", "").lower()
        return (
            "gnome-terminal" in terminal_emulator
            or "gnome-terminal" in term_program
            or "GNOME_TERMINAL_SCREEN" in os.environ
        )

    def apply(self) -> Optional[GnomeState]:
        """
        @brief Hide GNOME scrollbar and return state for later restoration.
        @return ``GnomeState`` when the change is applied/confirmed, else ``None``.
        """
        profile_id = _run_gsettings("get", "org.gnome.Terminal.ProfilesList", "default")
        if not profile_id:
            return None
        profile_id = profile_id.strip("'")
        if not profile_id:
            return None

        # GNOME stores per-profile keys under this dynamic schema path.
        schema = f"org.gnome.Terminal.Legacy.Profile:/org/gnome/terminal/legacy/profiles:/:{profile_id}/"
        current_policy = _run_gsettings("get", schema, "scrollbar-policy")
        if not current_policy:
            return None

        # set only if needed
        if current_policy != "'never'":
            _run_gsettings("set", schema, "scrollbar-policy", "never")

        return GnomeState(schema=schema, scrollbar_policy_backup=current_policy)

    def restore(self, state: Optional[GnomeState]) -> None:
        """
        @brief Restore the scrollbar policy captured by ``apply``.
        @param state Previously saved state; no-op when ``None``.
        @return None
        """
        if not state:
            return
        restore_value = state.scrollbar_policy_backup.strip("'")
        if restore_value:
            _run_gsettings("set", state.schema, "scrollbar-policy", restore_value)

    # endregion

    # region TERMINATOR
# TODO
    # endregion

    # region ZSH
# TODO
    # endregion

# endregion

# region SESSION

@dataclass
class TerminalSessionConfig:
    """@brief Runtime options controlling generic terminal tweaks."""

    # Background color used by OSC 11 (set default background color).
    bg_color: str = "#121212"
    # Emit OSC sequences to set and later reset background color.
    force_bg: bool = True
    # Emit DEC private mode sequence to hide/show scrollbar.
    hide_scrollbar: bool = True


class TerminalSession:
    """
    Session helper that applies terminal tweaks and safely restores them.
    """

    def __init__(self, adapters: List[TerminalAdapter], config: TerminalSessionConfig):
        """
        Build a terminal session with a list of adapters.
        """
        self.adapters = adapters
        self.config = config
        self._active: list[tuple[TerminalAdapter, Any]] = []

    def start(self) -> None:
        """
        Apply generic and adapter-specific terminal tweaks.
        """
        # Generic sequences (independent from specific emulators)
        if self.config.force_bg:
            # OSC 11: set default background color.
            write_terminal_sequence(f"\x1b]11;{self.config.bg_color}\x07")
        if self.config.hide_scrollbar:
            # DECSET private mode 30: hide scrollbar where supported.
            write_terminal_sequence("\x1b[?30l")

        # Terminal-specific adapters
        for adapter in self.adapters:
            if adapter.detect():
                state = adapter.apply()
                self._active.append((adapter, state))

    def end(self) -> None:
        """
        Restore adapter state and generic terminal tweaks.
        """
        # Restore adapters in reverse order
        for adapter, state in reversed(self._active):
            try:
                adapter.restore(state)
            except Exception:
                pass
        self._active.clear()

        # Restore generic sequences
        if self.config.hide_scrollbar:
            # DECRST private mode 30: show scrollbar again.
            write_terminal_sequence("\x1b[?30h")
        if self.config.force_bg:
            # OSC 111: reset default background color.
            write_terminal_sequence("\x1b]111\x07")

    def __enter__(self):
        """
        Context-manager entrypoint.
        """
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        """
        Context-manager exitpoint; always restores terminal state.
        """
        self.end()
        return False

# endregion