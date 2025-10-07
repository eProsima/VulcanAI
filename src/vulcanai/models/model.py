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

import base64
from typing import Any
from abc import ABC, abstractmethod

from vulcanai.core.plan_types import GlobalPlan


class IModel(ABC):
    """Abstract class for models."""
    # Model instance
    model: Any = None
    # Model name
    model_name: str = ""
    # Logger function
    logger: Any = None

    @abstractmethod
    def plan_inference(self, **kwargs) -> GlobalPlan:
        """Generate plan."""
        ...

    @abstractmethod
    def goal_inference(self, **kwargs):
        """Generate goal."""
        ...

    def _read_image(self, image_path: str) -> bytes:
        with open(image_path, "rb") as image_file:
            return image_file.read()

    def _encode_image(self, image_path: str) -> str:
        return base64.b64encode(self._read_image(image_path)).decode("utf-8")
