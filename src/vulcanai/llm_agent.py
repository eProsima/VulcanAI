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

import time
from typing import Literal
from openai import OpenAI

from vulcanai.plan_types import GlobalPlan


LLMBrand = Literal["gpt", "ollama"]


class LLMAgent:
    """Interface to operate the LLM."""
    def __init__(self, model: str, logger=None):
        self.model = model
        self.llm_brand: LLMBrand = "gpt"  # TODO: detect from model string
        self.logger = logger or print
        self._load_model(model)

    def inference(self, system_context: str, user_text: str) -> GlobalPlan:
        # Call the LLM to get a plan
        if self.llm_brand == "gpt":
            response = self._gpt_inference(system_context, user_text)
            plan: GlobalPlan = response.choices[0].message.parsed
        else:
            raise NotImplementedError(f"LLM brand {self.llm_brand} not supported yet.")

        return plan

    def _load_model(self, model: str):
        if self.llm_brand == "gpt":
            self.logger(f"Using OpenAI API with model: {model}")
            try:
                self.llm = OpenAI()
            except Exception as e:
                self.logger(f"Missing API Key: {e}")
                return
        else:
            raise NotImplementedError(f"LLM brand {self.llm_brand} not supported yet.")

    def _gpt_inference(self, system_prompt: str, user_prompt: str) -> str:
        start = time.time()
        completion = self.llm.chat.completions.parse(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            response_format=GlobalPlan,
        )
        end = time.time()
        self.logger(f"LLM response time: {end - start:.3f} seconds")
        input_tokens = completion.usage.prompt_tokens
        output_tokens = completion.usage.completion_tokens
        price = (0.05 * input_tokens + 0.4 * output_tokens) / 1000000
        self.logger(f"Prompt tokens: {input_tokens}, Completion tokens: {output_tokens}. Price: {price} €")

        return completion
