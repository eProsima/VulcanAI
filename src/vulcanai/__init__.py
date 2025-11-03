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

from importlib import import_module
from types import ModuleType

_SUBPACKAGES = ("core", "tools", "console", "models")
_submods: dict[str, ModuleType] = {
    name: import_module(f"{__name__}.{name}") for name in _SUBPACKAGES
}

__all__ = sorted({sym for m in _submods.values() for sym in getattr(m, "__all__", ())})

_MODULE_INDEX: dict[str, ModuleType] = {}
for pkg in _SUBPACKAGES:
    mod = _submods[pkg]
    for sym in getattr(mod, "__all__", ()):
        _MODULE_INDEX.setdefault(sym, mod)

def __getattr__(name: str):
    mod = _MODULE_INDEX.get(name)
    if not mod:
        raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
    return getattr(mod, name)

def __dir__():
    return sorted(list(globals().keys()) + __all__)
