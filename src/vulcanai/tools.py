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

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

from vulcanai.plan_types import ArgValue


class ITool(ABC):
    """
    Abstract class containing base metadata every tool must provide.
    All tools must inherit from this interface to ensure consistency during LLMs calls.
    """
    # Name given to the tool
    name: str
    # Brief description of the tool's purpose
    description: str
    # List of keywords associated with the tool
    tags: list[str] = []
    # JSON schemas for input validation and output formatting.
    # Only used for documentation and LLM prompt generation.
    input_schema: List[Tuple[str, str]] = []  # List of (key, type) pairs, simulating a ArgValue list
    output_schema: Dict[str, str] = {}
    # Tool version
    version: str = "0.1.0"
    # Enable ROS 2 dynamic discovery of this tool
    # This will create a ROS 2 service with the tool metadata
    enable_ros_discovery: bool = False
    # Blackboard shared memory (injected at execution time)
    bb: Optional[Dict[str, Any]] = None

    @abstractmethod
    def run(self, **kwargs) -> Dict[str, Any]:
        """Execute the tool synchronously."""
        ...


class AtomicTool(ITool):
    """Atomic tool with a single capability."""
    pass

class CompositeTool(ITool):
    """
    Composite tool used to define more complex actions.
    It reuses existing tools and their capabilities, which must be listed as dependencies.
    """
    # Names of tools this composite tool depends on
    dependencies: list[str] = []
