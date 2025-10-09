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


import mimetypes
import time
from openai import OpenAI

from vulcanai.core.plan_types import GlobalPlan, GoalSpec
from vulcanai.models.model import IModel


class OpenAIModel(IModel):
    """ Wrapper for OpenAI models. """
    def __init__(self, model_name: str, logger=None):
        super().__init__()
        self.logger = logger
        self.model_name = model_name
        try:
            self.model = OpenAI()
        except Exception as e:
            self.logger(f"Missing OpenAI API Key: {e}", error=True)

    def plan_inference(
            self,
            system_prompt: str,
            user_prompt: str,
            images: list[str],
            history: list[tuple[str, str]]
    ) -> GlobalPlan:
        """ Override plan inference method. """

        start = time.time()
        user_content = [{"type": "text", "text": user_prompt}]
        # Add images as base64 if any
        if images:
            for image_path in images:
                if image_path.startswith("http"):
                    user_content.append({"type": "image_url", "image_url": {"url": image_path}})
                else:
                    base64_image = self._encode_image(image_path)
                    mime = mimetypes.guess_type(image_path)[0] or "image/png"
                    user_content.append({
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{mime};base64,{base64_image}",
                        },
                    })
        # Create messages with history if any
        messages = [{"role": "system", "content": system_prompt}]
        if history:
            for user_text, plan_summary in history:
                messages.append({"role": "user", "content": user_text})
                messages.append({"role": "assistant", "content": f"Action plan: {plan_summary}"})
        messages.append({"role": "user", "content": user_content})

        completion = self.model.chat.completions.parse(
            model=self.model_name,
            messages=messages,
            response_format=GlobalPlan,
        )
        plan = None
        try:
            plan = completion.choices[0].message.parsed
        except Exception as e:
            self.logger(f"Failed to get parsed plan from GPT response: {e}", error=True)

        end = time.time()
        self.logger(f"GPT response time: {end - start:.3f} seconds")
        input_tokens = completion.usage.prompt_tokens
        output_tokens = completion.usage.completion_tokens
        self.logger(f"Prompt tokens: {input_tokens}, Completion tokens: {output_tokens}")

        return plan

    def goal_inference(
            self,
            system_prompt: str,
            user_prompt: str,
            history: list[tuple[str, str]]
    ) -> GoalSpec:
        """ Override goal inference method. """

        start = time.time()
        # Create messages with history if any
        messages = [{"role": "system", "content": system_prompt}]
        if history:
            for user_text, plan_summary in history:
                messages.append({"role": "user", "content": user_text})
                messages.append({"role": "assistant", "content": f"Action plan: {plan_summary}"})
        messages.append({"role": "user", "content": user_prompt})

        self.logger(f"[DEBUG] Sending messages to GPT for goal: {messages}")

        completion = self.model.chat.completions.parse(
            model=self.model_name,
            messages=messages,
            response_format=GoalSpec,
        )
        goal = None
        try:
            goal = completion.choices[0].message.parsed
        except Exception as e:
            self.logger(f"Failed to get parsed goal from GPT response: {e}", error=True)

        end = time.time()
        self.logger(f"GPT response time: {end - start:.3f} seconds")
        input_tokens = completion.usage.prompt_tokens
        output_tokens = completion.usage.completion_tokens
        self.logger(f"Prompt tokens: {input_tokens}, Completion tokens: {output_tokens}")

        return goal
