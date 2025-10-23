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


from openai import OpenAI
from typing import Any, Dict, Iterable, Optional, Type, TypeVar
import mimetypes
import time

from vulcanai.core.plan_types import AIValidation, GlobalPlan, GoalSpec
from vulcanai.models.model import IModel, IModelHooks

# Generic type variable for response classes
T = TypeVar('T', GlobalPlan, GoalSpec, AIValidation)


class OpenAIModel(IModel):
    """ Wrapper for OpenAI models. """
    def __init__(self, model_name: str, logger=None, hooks: Optional[IModelHooks] = None):
        super().__init__()
        self.logger = logger
        self.model_name = model_name
        self.hooks = hooks
        try:
            self.model = OpenAI()
        except Exception as e:
            self.logger(f"Missing OpenAI API Key: {e}", error=True)

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

        # Build messages (system + optional history + current user)
        messages = self._build_messages(system_prompt, user_content, history)

        # Notify hooks of request start
        try:
            self.hooks.on_request_start()
        except Exception as e:
            pass

        # Call OpenAI with response_format bound to the desired schema/class
        try:
            completion = self.model.chat.completions.parse(
                model=self.model_name,
                messages=messages,
                response_format=response_cls,
            )
        except Exception as e:
            self.logger(f"OpenAI API error: {e}", error=True)
            return None
        finally:
            # Notify hooks of request end
            try:
                self.hooks.on_request_end()
            except Exception as e:
                pass

        # Extract parsed object safely
        parsed: Optional[T] = None
        try:
            parsed = completion.choices[0].message.parsed
        except Exception as e:
            self.logger(f"Failed to parse response into {response_cls.__name__}: {e}", error=True)

        end = time.time()
        self.logger(f"GPT response time: {end - start:.3f} seconds")
        try:
            input_tokens = completion.usage.prompt_tokens
            output_tokens = completion.usage.completion_tokens
            self.logger(f"Prompt tokens: {input_tokens}, Completion tokens: {output_tokens}")
        except Exception:
            pass

        return parsed

    def _build_user_content(self, user_text: str, images: Optional[Iterable[str]]) -> list[Dict[str, Any]]:
        """Compose user content list with text first and optional images as image_url parts."""
        content: list[Dict[str, Any]] = [{"type": "text", "text": user_text}]
        if images:
            for image_path in images:
                if isinstance(image_path, str) and image_path.startswith("http"):
                    content.append({"type": "image_url", "image_url": {"url": image_path}})
                else:
                    try:
                        base64_image = self._encode_image(image_path)
                        mime = mimetypes.guess_type(image_path)[0] or "image/png"
                        content.append({
                            "type": "image_url",
                            "image_url": {"url": f"data:{mime};base64,{base64_image}"},
                        })
                    except Exception as e:
                        # Fail soft on a single bad image but continue with others
                        self.logger(f"Image '{image_path}' could not be encoded: {e}", error=True)
        return content

    def _build_messages(
        self,
        system_prompt: str,
        user_content: list[Dict[str, Any]],
        history: Optional[list[tuple[str, str]]],
    ) -> list[Dict[str, Any]]:
        """Construct the messages array with system + optional history + current user content."""
        messages: list[Dict[str, Any]] = [{"role": "system", "content": system_prompt}]

        # Replay short history as (user, assistant) turns
        if history:
            for user_text, plan_summary in history:
                messages.append({"role": "user", "content": user_text})
                messages.append({"role": "assistant", "content": f"Action plan: {plan_summary}"})

        # Append current user turn (text + images)
        messages.append({"role": "user", "content": user_content})
        return messages
