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
import mimetypes
import os
import time
from enum import Enum

from vulcanai.console.logger import VulcanAILogger
from vulcanai.core.plan_types import GlobalPlan


class LLMBrand(str, Enum):
    gpt = "gpt"
    gemini = "gemini"


class LLMAgent:
    """Interface to operate the LLM."""
    def __init__(self, model: str, logger=None):
        self.model = model
        self.llm_brand: LLMBrand = self._detect_llm_brand(model)
        self.llm = None
        self.logger = logger or VulcanAILogger().log_manager
        self._load_model(model)

    def inference(self, system_context: str, user_text: str, images: list[str], history: list[tuple[str, str]]) -> GlobalPlan:
        """Perform inference using the selected LLM model."""
        if self.llm is None:
            raise RuntimeError("LLM model was not loaded correctly.")
        if self.llm_brand == LLMBrand.gpt:
            plan: GlobalPlan = self._gpt_inference(system_context, user_text, images, history)
        elif self.llm_brand == LLMBrand.gemini:
            plan: GlobalPlan = self._gemini_inference(system_context, user_text, images, history)
        else:
            raise NotImplementedError(f"LLM brand '{self.llm_brand.value}' not supported.")

        return plan

    def _detect_llm_brand(self, model: str) -> LLMBrand:
        m = model.lower()
        if m.startswith(("gpt-", "o")):
            return LLMBrand.gpt
        if m.startswith(("gemini-", "gemma-")):
            return LLMBrand.gemini
        else:
            raise NotImplementedError(f"LLM model {model} not supported.")

    def _load_model(self, model: str):
        if self.llm_brand == LLMBrand.gpt:
            from openai import OpenAI
            self.logger(f"Using OpenAI API with model: {model}")
            try:
                self.llm = OpenAI()
            except Exception as e:
                self.logger(f"Missing OpenAI API Key: {e}", error=True)
            return
        if self.llm_brand == LLMBrand.gemini:
            from google import genai
            self.logger(f"Using Gemini API with model: {model}")
            try:
                self.llm = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
            except Exception as e:
                self.logger(f"Missing Gemini API Key: {e}", error=True)
            return
        else:
            raise NotImplementedError(f"LLM brand {self.llm_brand} not supported.")

    def _gpt_inference(self, system_prompt: str, user_prompt: str, images: list[str], history: list[tuple[str, str]]) -> GlobalPlan:
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

        completion = self.llm.chat.completions.parse(
            model=self.model,
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

    def _gemini_inference(self, system_prompt: str, user_prompt: str, images: list[str], history: list[tuple[str, str]]) -> GlobalPlan:
        from google.genai import types as gtypes
        start = time.time()
        user_content = [gtypes.Part.from_text(text=user_prompt)]
        # Add images as base64 if any
        if images:
            for image_path in images:
                if image_path.startswith("http"):
                    import requests
                    img = requests.get(image_path)
                    if img.status_code != 200:
                        self.logger(f"Failed to fetch image from URL: {image_path}", error=True)
                        continue
                    user_content.append(gtypes.Part.from_bytes(data=img.content, mime_type=img.headers.get("Content-Type", "image/png")))
                else:
                    img_bytes = self._read_image(image_path)
                    mime = mimetypes.guess_type(image_path)[0] or "image/png"
                    user_content.append(gtypes.Part.from_bytes(
                        data=img_bytes,
                        mime_type=mime,
                    ))
        # Create contents with history if any
        contents = []
        if history:
            for user_text, plan_summary in history:
                contents.append(gtypes.Content(role="user", parts=[gtypes.Part.from_text(text=user_text)]))
                contents.append(gtypes.Content(role="assistant", parts=[gtypes.Part.from_text(text=f"Action plan: {plan_summary}")]))
        contents.append(gtypes.Content(role="user", parts=user_content))
        cfg = gtypes.GenerateContentConfig(
            response_mime_type="application/json",
            response_schema=GlobalPlan,
            system_instruction=[system_prompt],
            candidate_count=1,
        )

        response = self.llm.models.generate_content(
            model=self.model,
            contents=contents,
            config=cfg,
        )
        plan = None
        try:
            plan = response.parsed
        except Exception as e:
            self.logger(f"Failed to get parsed plan from Gemini response, falling back to text: {e}", error=True)

        # Fallback to get GlobalPlan from text if the parsed field is not available
        if plan is None:
            raw = None
            if not raw and getattr(response, "candidates", None):
                raw = response.candidates[0].content.parts[0].text
            if not raw:
                raise RuntimeError("Failed to get text plan from Gemini response.")

            try:
                plan = GlobalPlan.model_validate_json(raw)
            except Exception as e:
                try:
                    import json
                    plan = GlobalPlan(**json.loads(raw))
                except Exception as e:
                    self.logger(f"Failed to parse raw plan JSON: {e}", error=True)

        end = time.time()
        self.logger(f"Gemini response time: {end - start:.3f} seconds")
        usage = getattr(response, "usage_metadata", None)
        if usage:
            self.logger(f"Prompt tokens: {usage.prompt_token_count}, Completion tokens: {usage.candidates_token_count}")

        return plan

    def _read_image(self, image_path: str) -> bytes:
        with open(image_path, "rb") as image_file:
            return image_file.read()

    def _encode_image(self, image_path: str) -> str:
        return base64.b64encode(self._read_image(image_path)).decode("utf-8")
