# Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Optional, Protocol


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
        pass


class TerminalAdapter(Protocol):
    """
    Abstract parent class to enhance VulcanAI visualization in each terminal.
    Currently supported: Gnome
    Not yet implemented: Terminator, Zsh
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
        is_gnome = (
            "GNOME_TERMINAL_SCREEN" in os.environ
            or "gnome-terminal" in os.environ.get("TERMINAL_EMULATOR", "").lower()
            or "gnome-terminal" in os.environ.get("TERM_PROGRAM", "").lower()
        )
        return is_gnome

    def apply(self) -> Optional[GnomeState]:
        """
        @brief Hide GNOME scrollbar and return state for later restoration.
        @return ``GnomeState`` when the change is applied/confirmed, else ``None``.
        """
        # The return value could be None, empty string or string with just single quotes
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

    def __init__(
        self,
        config: Optional[TerminalSessionConfig] = None,
        adapters: Optional[list[TerminalAdapter]] = None,
    ):
        self.config = config if config is not None else TerminalSessionConfig()
        self.adapters = adapters if adapters is not None else [GnomeTerminalAdapter()]
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
