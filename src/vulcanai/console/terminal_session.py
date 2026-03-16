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
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
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
    Currently supported: Gnome, Terminator
    Not yet implemented: Zsh
    """

    name: str

    def detect(self) -> bool: ...

    def apply(self) -> Any: ...

    def restore(self, state: Any) -> None: ...


# region TERMINALS

# region gnome


@dataclass
class GnomeState:
    """@brief State required to restore GNOME Terminal settings."""

    schema: str
    scrollbar_policy_backup: str


class GnomeTerminalAdapter:
    """@brief GNOME Terminal adapter that hides and restores the scrollbar."""

    name = "gnome-terminal"

    @staticmethod
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
        profile_id = self._run_gsettings("get", "org.gnome.Terminal.ProfilesList", "default")
        if not profile_id:
            return None
        profile_id = profile_id.strip("'")
        if not profile_id:
            return None

        # GNOME stores per-profile keys under this dynamic schema path.
        schema = f"org.gnome.Terminal.Legacy.Profile:/org/gnome/terminal/legacy/profiles:/:{profile_id}/"
        current_policy = self._run_gsettings("get", schema, "scrollbar-policy")
        if not current_policy:
            return None

        # set only if needed
        if current_policy != "'never'":
            self._run_gsettings("set", schema, "scrollbar-policy", "never")

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
            self._run_gsettings("set", state.schema, "scrollbar-policy", restore_value)


# endregion

# region terminator

@dataclass
class TerminatorState:
    terminal_uuid: str
    previous_profile: str


class TerminatorTerminalAdapter:
    """@brief Terminator adapter that switches to a hidden-scroll profile temporarily."""

    name = "terminator"
    # Matches any top-level section like [profiles], [global_config], [layouts].
    # Needed to detect where the [profiles] block ends.
    _TOP_LEVEL_SECTION_RE = re.compile(r"^\s*\[[^\[\]].*\]\s*$")
    # Matches profile headers inside [profiles], e.g. "  [[default]]".
    # Group 1 stores indentation so cloned profiles preserve style.
    # Group 2 stores the profile name.
    _PROFILE_HEADER_RE = re.compile(r"^(\s*)\[\[(.+?)\]\]\s*$")
    # Matches generic "key = value" rows and captures indentation.
    # Used when appending missing keys with consistent formatting.
    _KEY_VALUE_RE = re.compile(r"^(\s*)[A-Za-z0-9_]+\s*=")
    # Matches the specific scrollbar setting row.
    # Used to rewrite current value to "disabled" without touching other keys.
    _SCROLLBAR_RE = re.compile(r"^(\s*)scrollbar_position\s*=")

    def __init__(self, config: "TerminalSessionConfig"):
        self._config = config

    # -- Utils ----------------------------------------------------------------

    @staticmethod
    def _run(*args: str) -> bool:
        """
        Execute a command and return success status.

        @return ``True`` when process exits with code ``0``, else ``False``.
        """
        try:
            completed = subprocess.run(
                [*args],
                check=False,
                capture_output=True,
                text=True,
            )
        except Exception:
            return False
        return completed.returncode == 0

    @staticmethod
    def _config_path() -> Path:
        """
        Resolve Terminator config file location.

        @return Absolute path to ``terminator/config`` under ``XDG_CONFIG_HOME`` or ``~/.config``.
        """
        config_root = os.environ.get("XDG_CONFIG_HOME") or os.path.join(os.path.expanduser("~"), ".config")
        return Path(config_root) / "terminator" / "config"

    @classmethod
    def _ensure_hidden_profile(cls, config_path: Path, base_profile: str, hidden_profile: str) -> bool:
        """
        Ensure hidden profile exists and has ``scrollbar_position = disabled``.

        @return ``True`` when config is ready for profile switching, else ``False``.
        """
        try:
            lines = config_path.read_text(encoding="utf-8").splitlines(keepends=True)
        except Exception:
            return False

        profiles_start = next((i for i, line in enumerate(lines) if line.strip() == "[profiles]"), None)
        if profiles_start is None:
            return False

        profiles_end = len(lines)
        for index in range(profiles_start + 1, len(lines)):
            if cls._TOP_LEVEL_SECTION_RE.match(lines[index]) and lines[index].strip() != "[profiles]":
                profiles_end = index
                break

        profile_headers: list[tuple[str, int, str]] = []
        for index in range(profiles_start + 1, profiles_end):
            header_match = cls._PROFILE_HEADER_RE.match(lines[index].rstrip("\r\n"))
            if header_match:
                profile_headers.append((header_match.group(2).strip(), index, header_match.group(1)))

        if not profile_headers:
            return False

        blocks: dict[str, tuple[int, int, str]] = {}
        for idx, (name, start_idx, indent) in enumerate(profile_headers):
            end_idx = profile_headers[idx + 1][1] if idx + 1 < len(profile_headers) else profiles_end
            blocks[name] = (start_idx, end_idx, indent)

        def ensure_disabled(block: list[str], section_indent: str) -> list[str]:
            """
            Update one profile block so scrollbar is always disabled.
            """
            updated = [block[0]]
            scrollbar_found = False
            key_indent = None
            for line in block[1:]:
                stripped = line.rstrip("\r\n")
                if key_indent is None:
                    key_match = cls._KEY_VALUE_RE.match(stripped)
                    if key_match:
                        key_indent = key_match.group(1)
                scrollbar_match = cls._SCROLLBAR_RE.match(stripped)
                if scrollbar_match:
                    updated.append(f"{scrollbar_match.group(1)}scrollbar_position = disabled\n")
                    scrollbar_found = True
                else:
                    updated.append(line)
            if not scrollbar_found:
                indent = key_indent if key_indent is not None else f"{section_indent}  "
                updated.append(f"{indent}scrollbar_position = disabled\n")
            return updated

        changed = False
        if hidden_profile in blocks:
            hidden_start, hidden_end, hidden_indent = blocks[hidden_profile]
            hidden = ensure_disabled(lines[hidden_start:hidden_end], hidden_indent)
            if hidden != lines[hidden_start:hidden_end]:
                lines = lines[:hidden_start] + hidden + lines[hidden_end:]
                changed = True
        else:
            if base_profile not in blocks:
                return False
            base_start, base_end, base_indent = blocks[base_profile]
            hidden = [f"{base_indent}[[{hidden_profile}]]\n", *lines[base_start:base_end][1:]]
            lines = lines[:profiles_end] + ensure_disabled(hidden, base_indent) + lines[profiles_end:]
            changed = True

        if changed:
            try:
                config_path.write_text("".join(lines), encoding="utf-8")
            except Exception:
                return False
        return True

    # -------------------------------------------------------------------------

    def detect(self) -> bool:
        """
        @brief Detect whether current terminal is Terminator.
        @return ``True`` when Terminator environment markers are present.
        """
        return (
            "TERMINATOR_UUID" in os.environ
            or "terminator" in os.environ.get("TERMINAL_EMULATOR", "").lower()
            or "terminator" in os.environ.get("TERM_PROGRAM", "").lower()
        )

    def apply(self) -> Optional[TerminatorState]:
        """
        @brief Switch current Terminator tab to hidden-scroll profile.
        @return ``(uuid, base_profile)`` when switching succeeds, else ``None``.
        """
        terminal_uuid = os.environ.get("TERMINATOR_UUID")
        if not terminal_uuid:
            return None

        if not shutil.which("remotinator"):
            return None

        previous_profile = self._resolve_previous_profile()
        if not previous_profile:
            return None

        config_path = self._config_path()
        if not config_path.is_file():
            return None

        if not self._ensure_hidden_profile(
            config_path=config_path,
            base_profile=previous_profile,
            hidden_profile=self._config.terminator_profile_hidden,
        ):
            return None

        switched = self._run(
            "remotinator",
            "switch_profile",
            "-u",
            terminal_uuid,
            "-p",
            self._config.terminator_profile_hidden,
        )
        if not switched:
            return None

        return TerminatorState(
            terminal_uuid=terminal_uuid,
            previous_profile=previous_profile,
        )

    def _resolve_previous_profile(self) -> Optional[str]:
        candidates = (
            self._config.terminator_profile_current,
            os.environ.get("VULCANAI_TERMINATOR_PROFILE"),
            self._config.terminator_profile_base,
        )
        for profile in candidates:
            if profile is None:
                continue
            normalized = profile.strip()
            if normalized:
                return normalized
        return None

    def restore(self, state: Optional[TerminatorState]) -> None:
        """
        @brief Restore previous Terminator profile.
        @param state Previously saved state; no-op when ``None``.
        @return None
        """
        if not state:
            return
        if not shutil.which("remotinator"):
            return

        self._run(
            "remotinator",
            "switch_profile",
            "-u",
            state.terminal_uuid,
            "-p",
            state.previous_profile,
        )


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
    # Terminator profile to restore once session ends.
    terminator_profile_base: str = "default"
    # Terminator profile used while session is running.
    terminator_profile_hidden: str = "vulcanai-no-scroll"

    terminator_profile_current: Optional[str] = None


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
        self.adapters = (
            adapters if adapters is not None else [GnomeTerminalAdapter(), TerminatorTerminalAdapter(self.config)]
        )
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
