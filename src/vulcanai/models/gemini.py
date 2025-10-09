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
import os
import time
from google import genai
from google.genai import types as gtypes

from vulcanai.core.plan_types import GlobalPlan, GoalSpec
from vulcanai.models.model import IModel

class GeminiModel(IModel):
    """ Wrapper for most of Google models, Gemini mainly. """
    def __init__(self, model_name:str, logger=None):
        super().__init__()
        self.logger = logger
        self.model_name = model_name
        try:
            self.model = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
        except Exception as e:
            self.logger(f"Missing Gemini API Key: {e}", error=True)

    def plan_inference(
            self,
            system_prompt: str,
            user_prompt: str,
            images: list[str],
            history: list[tuple[str, str]]
    ) -> GlobalPlan:
        """ Override plan inference method. """

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

        response = self.model.models.generate_content(
            model=self.model_name,
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

    def goal_inference(
            self,
            system_prompt: str,
            user_prompt: str,
            history: list[tuple[str, str]]
    ) -> GoalSpec:
        """ Override goal inference method. """

        start = time.time()
        user_content = [gtypes.Part.from_text(text=user_prompt)]
        # Create contents with history if any
        contents = []
        if history:
            for user_text, plan_summary in history:
                contents.append(gtypes.Content(role="user", parts=[gtypes.Part.from_text(text=user_text)]))
                contents.append(gtypes.Content(role="assistant", parts=[gtypes.Part.from_text(text=f"Action plan: {plan_summary}")]))
        contents.append(gtypes.Content(role="user", parts=user_content))
        cfg = gtypes.GenerateContentConfig(
            response_mime_type="application/json",
            response_schema=GoalSpec,
            system_instruction=[system_prompt],
            candidate_count=1,
        )

        response = self.model.models.generate_content(
            model=self.model_name,
            contents=contents,
            config=cfg,
        )
        goal = None
        try:
            goal = response.parsed
        except Exception as e:
            self.logger(f"Failed to get parsed goal from Gemini response, falling back to text: {e}", error=True)

        # Fallback to get GoalSpec from text if the parsed field is not available
        if goal is None:
            raw = None
            if not raw and getattr(response, "candidates", None):
                raw = response.candidates[0].content.parts[0].text
            if not raw:
                raise RuntimeError("Failed to get text goal from Gemini response.")

            try:
                goal = GoalSpec.model_validate_json(raw)
            except Exception as e:
                try:
                    import json
                    goal = GoalSpec(**json.loads(raw))
                except Exception as e:
                    self.logger(f"Failed to parse raw goal JSON: {e}", error=True)

        end = time.time()
        self.logger(f"Gemini response time: {end - start:.3f} seconds")
        usage = getattr(response, "usage_metadata", None)
        if usage:
            self.logger(f"Prompt tokens: {usage.prompt_token_count}, Completion tokens: {usage.candidates_token_count}")

        return goal
