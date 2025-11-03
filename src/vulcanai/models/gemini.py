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

from google import genai
from google.genai import types as gtypes
from typing import Any, Dict, Iterable, Optional, Type, TypeVar
import mimetypes
import os
import time

from vulcanai.core.plan_types import AIValidation, GlobalPlan, GoalSpec
from vulcanai.models.model import IModel, IModelHooks

T = TypeVar('T', GlobalPlan, GoalSpec, AIValidation)


class GeminiModel(IModel):
    """ Wrapper for most of Google models, Gemini mainly. """
    def __init__(self, model_name:str, logger=None, hooks: Optional[IModelHooks] = None):
        super().__init__()
        self.logger = logger
        self.model_name = model_name
        self.hooks = hooks
        try:
            self.model = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
        except Exception as e:
            self.logger(f"Missing Gemini API Key: {e}", error=True)

    def _inference(
        self,
        *,
        system_prompt: str,
        user_prompt: str,
        response_cls: Type[T],
        images: Optional[Iterable[str]] = None,
        history: Optional[list[tuple[str, str]]] = None,
    ) -> Optional[T]:
        """
        Generic inference template function that parameterizes the response class.

        :param system_prompt: System message.
        :param user_prompt: User message.
        :param response_cls: Target structured output type (e.g., GlobalPlan, GoalSpec, AIValidation).
        :param images: Optional image paths or URLs; included as image_url content parts.
        :param history: Optional (user_text, plan_summary) tuples to reconstruct conversational context.
        :return: Parsed response object of type T, or None on error.
        """
        start = time.time()

        # Build user content (text + optional images)
        user_content = self._build_user_content(user_prompt, images)

        # Build messages (optional history + current user)
        messages = self._build_messages(user_content, history)

        # Configuration for content generation
        cfg = gtypes.GenerateContentConfig(
            response_mime_type="application/json",
            response_schema=response_cls,
            system_instruction=[system_prompt],
            candidate_count=1,
        )

        # Notify hooks of request start
        try:
            self.hooks.on_request_start()
        except Exception as e:
            pass

        response = self.model.models.generate_content(
            model=self.model_name,
            contents=messages,
            config=cfg,
        )
        parsed_response: Optional[T] = None
        try:
            parsed_response = response.parsed
        except Exception as e:
            self.logger(f"Failed to get parsed goal from Gemini response, falling back to text: {e}", error=True)
        finally:
            # Notify hooks of request end
            try:
                self.hooks.on_request_end()
            except Exception as e:
                pass

        # Fallback to get GoalSpec from text if the parsed field is not available
        if parsed_response is None:
            raw = None
            if not raw and getattr(response, "candidates", None):
                raw = response.candidates[0].content.parts[0].text
            if not raw:
                raise RuntimeError(f"Failed to get {response_cls.__name__} from Gemini response.")

            try:
                parsed_response = GoalSpec.model_validate_json(raw)
            except Exception as e:
                try:
                    import json
                    parsed_response = GoalSpec(**json.loads(raw))
                except Exception as e:
                    self.logger(f"Failed to parse raw {response_cls.__name__} JSON: {e}", error=True)

        end = time.time()
        self.logger(f"Gemini response time: {end - start:.3f} seconds")
        usage = getattr(response, "usage_metadata", None)
        if usage:
            self.logger(f"Prompt tokens: {usage.prompt_token_count}, Completion tokens: {usage.candidates_token_count}")

        return parsed_response


    def _build_user_content(self, user_text: str, images: Optional[Iterable[str]]) -> list[gtypes.Part]:
        """Compose user content list with text first and optional images as image_url parts."""
        content: list[gtypes.Part] = [gtypes.Part.from_text(text=user_text)]
        if images:
            for image_path in images:
                if isinstance(image_path, str) and image_path.startswith("http"):
                    import requests
                    img = requests.get(image_path)
                    if img.status_code != 200:
                        self.logger(f"Failed to fetch image from URL: {image_path}", error=True)
                        continue
                    content.append(gtypes.Part.from_bytes(data=img.content, mime_type=img.headers.get("Content-Type", "image/png")))
                else:
                    try:
                        img_bytes = self._read_image(image_path)
                        mime = mimetypes.guess_type(image_path)[0] or "image/png"
                        content.append(gtypes.Part.from_bytes(
                            data=img_bytes,
                            mime_type=mime,
                        ))
                    except Exception as e:
                        # Fail soft on a single bad image but continue with others
                        self.logger(f"Image '{image_path}' could not be encoded: {e}", error=True)
        return content


    def _build_messages(
        self,
        user_content: list[gtypes.Part],
        history: Optional[list[tuple[str, str]]],
    ) -> list[gtypes.Content]:
        """Construct the messages array with system + optional history + current user content."""
        messages: list[gtypes.Content] = []

        # Replay short history as (user, assistant) turns
        if history:
            for user_text, plan_summary in history:
                messages.append(gtypes.Content(role="user", parts=[gtypes.Part.from_text(text=user_text)]))
                messages.append(gtypes.Content(role="assistant", parts=[gtypes.Part.from_text(text=f"Action plan: {plan_summary}")]))

        # Append current user turn (text + images)
        messages.append(gtypes.Content(role="user", parts=user_content))
        return messages
