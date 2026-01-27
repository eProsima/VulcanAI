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
from abc import ABC, abstractmethod
from typing import Any, Optional, TypeVar

from vulcanai.core.plan_types import AIValidation, GlobalPlan, GoalSpec

T = TypeVar("T", GlobalPlan, GoalSpec, AIValidation)


class IModel(ABC):
    """Abstract class for models."""

    # Model instance
    model: Any = None
    # Model name
    model_name: str = ""
    # Logger function
    logger: Any = None
    # Optional hooks for LLM/VLM activity
    hooks: Any = None

    def plan_inference(
        self, system_prompt: str, user_prompt: str, images: list[str], history: list[tuple[str, str]]
    ) -> Optional[GlobalPlan]:
        """
        Call the generic inference with GlobalPlan as response type.

        :param system_prompt: System message.
        :param user_prompt: User message.
        :param images: Optional image paths or URLs.
        :param history: Optional (user_text, plan_summary) tuples to reconstruct conversational context.
        :return: Parsed response object of type GlobalPlan, or None on error.
        """
        return self._inference(
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            response_cls=GlobalPlan,
            images=images,
            history=history,
        )

    def goal_inference(
        self, system_prompt: str, user_prompt: str, history: list[tuple[str, str]]
    ) -> Optional[GoalSpec]:
        """
        Call the generic inference with GoalSpec as response type (no images).

        :param system_prompt: System message.
        :param user_prompt: User message.
        :param history: Optional (user_text, plan_summary) tuples to reconstruct conversational context.
        :return: Parsed response object of type GoalSpec, or None on error.
        """
        return self._inference(
            system_prompt=system_prompt, user_prompt=user_prompt, response_cls=GoalSpec, images=None, history=history
        )

    def validation_inference(
        self, system_prompt: str, user_prompt: str, images: list[str], history: list[tuple[str, str]]
    ) -> Optional[AIValidation]:
        """
        Call the generic inference with AIValidation as response type (no history).

        :param system_prompt: System message.
        :param user_prompt: User message.
        :param images: Optional image paths or URLs.
        :return: Parsed response object of type AIValidation, or None on error.
        """
        return self._inference(
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            response_cls=AIValidation,
            images=images,
            history=history,
        )

    @abstractmethod
    def _inference(self, **kwargs) -> Optional[T]:
        """Generate an AIValidation to verify if the user's goal has been achieved."""
        ...

    def _read_image(self, image_path: str) -> bytes:
        with open(image_path, "rb") as image_file:
            return image_file.read()

    def _encode_image(self, image_path: str) -> str:
        return base64.b64encode(self._read_image(image_path)).decode("utf-8")


class IModelHooks(ABC):
    """No-op base hooks for LLM activity."""

    def on_request_start(self) -> None:
        pass

    def on_request_end(self) -> None:
        pass
