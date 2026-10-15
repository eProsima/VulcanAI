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

"""Read-only, context-aware completion for the ROS 2 CLI in VulcanConsole."""

import os
import re
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

# Fallbacks keep the default tools discoverable even without ROS on PATH.
COMMANDS = {
    "action": ("info", "list", "send_goal", "type"),
    "interface": ("list", "package", "packages", "show"),
    "node": ("info", "list"),
    "param": ("delete", "describe", "dump", "get", "list", "load", "set"),
    "pkg": ("executables", "list"),
    "service": ("call", "echo", "find", "info", "list", "type"),
    "topic": ("bw", "delay", "echo", "find", "hz", "info", "list", "pub", "type"),
}


@dataclass
class Completion:
    start: int
    end: int
    matches: list[str]

    def apply(self, value):
        if not self.matches:
            return value, self.end
        replacement = os.path.commonprefix(self.matches)
        # Replace only the current token, preserving text after the cursor.
        tail = value[self.end :]
        if len(self.matches) == 1 and not tail and not replacement.endswith("/"):
            replacement += " "
        return value[: self.start] + replacement + tail, self.start + len(replacement)


class Ros2Completer:
    def __init__(self, runner=None, ttl=2.0):
        self.runner = runner or self._run
        self.ttl = ttl
        self.cache = {}

    @staticmethod
    def _run(args):
        return subprocess.run(args, capture_output=True, text=True, timeout=3, check=True).stdout

    def query(self, *args):
        key = tuple(args)
        cached = self.cache.get(key)
        if cached and time.monotonic() - cached[0] < self.ttl:
            return cached[1]
        try:
            output = self.runner(["ros2", *args])
        except (OSError, subprocess.SubprocessError):
            output = ""
        self.cache[key] = (time.monotonic(), output)
        return output

    def commands(self, *group):
        help_text = self.query(*group, "--help")
        section = help_text.split("Commands:", 1)[-1] if "Commands:" in help_text else ""
        discovered = re.findall(r"^  ([a-z][a-z0-9_]*)\s{2,}\S", section, re.MULTILINE)
        fallback = COMMANDS.get(group[0], ()) if group else COMMANDS
        return sorted(set(discovered) | set(fallback))

    def lines(self, *args):
        return [line.strip() for line in self.query(*args).splitlines() if line.strip()]

    def interfaces(self, kind):
        return [line for line in self.lines("interface", "list") if f"/{kind}/" in line]

    def candidates(self, context, prefix):
        if not context:
            return ["ros2"]
        if context == ["ros2"]:
            return self.commands()
        group = context[1]
        if len(context) == 2:
            return self.commands(group)
        verb = context[2]
        args = context[3:]
        index = len(args)
        if prefix.startswith("-"):
            return sorted(set(re.findall(r"(?<!\w)--?[a-zA-Z][\w-]*", self.query(group, verb, "--help"))))
        if group == "service":
            if verb in ("call", "echo", "info", "type") and index == 0:
                return self.lines("service", "list")
            if verb == "find" and index == 0:
                return self.interfaces("srv")
            if verb == "call" and index == 1:
                return self.lines("service", "type", args[0]) or self.interfaces("srv")
            if verb == "call" and index == 2 and args[1] == "std_srvs/srv/Empty":
                return ["{}"]
        if group == "topic":
            if verb in ("bw", "delay", "echo", "hz", "info", "type", "pub") and index == 0:
                return self.lines("topic", "list")
            if verb == "find" and index == 0:
                return self.interfaces("msg")
            if verb in ("pub", "echo") and index == 1:
                return self.lines("topic", "type", args[0]) or self.interfaces("msg")
        if group == "action":
            if verb in ("info", "send_goal", "type") and index == 0:
                return self.lines("action", "list")
            if verb == "send_goal" and index == 1:
                return self.lines("action", "type", args[0]) or self.interfaces("action")
        if group == "node" and verb == "info" and index == 0:
            return self.lines("node", "list")
        if group == "param":
            if verb in ("delete", "describe", "dump", "get", "list", "load", "set") and index == 0:
                return self.lines("node", "list")
            if verb in ("delete", "describe", "get", "set") and index == 1:
                return self.lines("param", "list", args[0])
            if verb == "load" and index == 1:
                return self.paths(prefix)
        if group == "interface":
            if verb in ("package",) and index == 0:
                return self.lines("interface", "packages")
            if verb == "show" and index == 0:
                return [line for line in self.lines("interface", "list") if "/" in line]
        if group == "pkg" and verb == "executables" and index == 0:
            return self.lines("pkg", "list")
        return []

    @staticmethod
    def paths(prefix):
        expanded = Path(os.path.expanduser(prefix))
        directory = expanded if prefix.endswith("/") else expanded.parent
        try:
            parent = prefix.rsplit("/", 1)[0] + "/" if "/" in prefix else ""
            return [parent + path.name + ("/" if path.is_dir() else "") for path in directory.iterdir()]
        except OSError:
            return []

    def complete(self, value, cursor):
        before = value[:cursor]
        spans = list(re.finditer(r"\S+", before))
        if not spans:
            return Completion(cursor, cursor, [])
        start = spans[-1].start() if not before[-1].isspace() else cursor
        # Include the rest of a token when the cursor is in its middle.
        end = cursor
        while end < len(value) and not value[end].isspace():
            end += 1
        prefix = value[start:cursor]
        try:
            context = shlex.split(value[:start])
        except ValueError:
            return Completion(start, end, [])
        if context and context[0] != "ros2":
            return Completion(start, end, [])
        # run/launch take a package directly, rather than a subcommand.
        if context == ["ros2", "run"] or context == ["ros2", "launch"]:
            options = self.lines("pkg", "list")
        elif len(context) == 3 and context[:2] == ["ros2", "run"]:
            options = [line.split()[-1] for line in self.lines("pkg", "executables", context[2])]
        elif len(context) == 3 and context[:2] == ["ros2", "launch"]:
            locations = self.lines("pkg", "prefix", "--share", context[2])
            options = []
            if locations:
                options = [p.name for p in (Path(locations[0]) / "launch").glob("*launch*") if p.is_file()]
        else:
            options = self.candidates(context, prefix)
        return Completion(start, end, sorted({option for option in options if option.startswith(prefix)}))
