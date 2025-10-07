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

from enum import Enum

from vulcanai.console.logger import VulcanAILogger
from vulcanai.core.plan_types import GlobalPlan


class Brand(str, Enum):
    gpt = "gpt"
    gemini = "gemini"


class Agent:
    """Interface to operate the LLM."""
    def __init__(self, model_name: str, logger=None):
        self.brand: Brand = self._detect_brand(model_name)
        self.model = None
        self.logger = logger or VulcanAILogger.log_manager
        self._load_model(model_name)

    def inference(
            self,
            system_context: str,
            user_prompt: str,
            images: list[str],
            history: list[tuple[str, str]]
        ) -> GlobalPlan:
        """Perform inference using the selected LLM model."""
        if self.model is None:
            raise RuntimeError("LLM model was not loaded correctly.")

        plan: GlobalPlan = self.model.plan_inference(
            system_prompt=system_context,
            user_prompt=user_prompt,
            images=images,
            history=history
        )

        return plan

    def _detect_brand(self, model_name: str) -> Brand:
        m = model_name.lower()
        if m.startswith(("gpt-", "o")):
            return Brand.gpt
        if m.startswith(("gemini-", "gemma-")):
            return Brand.gemini
        else:
            raise NotImplementedError(f"Model {model_name} not supported.")

    def _load_model(self, model_name: str):
        if self.brand == Brand.gpt:
            from vulcanai.models.openai import OpenAIModel
            self.logger(f"Using OpenAI API with model: {model_name}")
            self.model = OpenAIModel(model_name, self.logger)

        elif self.brand == Brand.gemini:
            from vulcanai.models.gemini import GeminiModel
            self.logger(f"Using Gemini API with model: {model_name}")
            self.model = GeminiModel(model_name, self.logger)

        else:
            raise NotImplementedError(f"LLM brand {self.brand} not supported.")
