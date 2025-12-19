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

import ollama
from typing import Any, Dict, Iterable, Optional,Type, TypeVar
import time

from vulcanai.core.plan_types import AIValidation, GlobalPlan, GoalSpec
from vulcanai.models.model import IModel, IModelHooks

# Generic type variable for response classes
T = TypeVar('T', GlobalPlan, GoalSpec, AIValidation)


class OllamaModel(IModel):

    # Color of the class [MANAGER] in the textual terminal
    class_color = "#0d87c0"

    """ Wrapper for Ollama models. """
    def __init__(self, model_name: str, logger=None, hooks: Optional[IModelHooks] = None):
        super().__init__()
        self.logger = logger
        self.model_name = model_name
        self.hooks = hooks
        try:
            self.model = ollama
        except Exception as e:
            # Print in textual terminal:
            # [MANAGER] ERROR. Missing a API Key: <exception>
            self.logger(f"ERROR. Missing a API Key: {e}", log_type="manager", log_color=0)

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

        # Call Ollama with response_format bound to the desired schema/class
        try:
            completion = self.model.chat(
                model=self.model_name,
                messages=messages,
                format=response_cls.model_json_schema(),
                options={"temperature": 0.1}
            )
        except Exception as e:
            # Print in textual terminal:
            # [MANAGER] ERROR. Ollama API: <exception>
            self.logger(f"ERROR. Ollama API: {e}", log_type="manager", log_color=0)
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
            parsed = response_cls.model_validate_json(completion.message.content)
        except Exception as e:
            # Print in textual terminal:
            # [MANAGER] ERROR. Failed to parse response into <response_cls.__name__>: <exepction>
            self.logger(f"ERROR. Failed to parse response into {response_cls.__name__}: {e}",
                        log_type="manager", log_color=0)

        end = time.time()
        # Print in textual terminal:
        # [MANAGER] Ollama response time: <time> seconds
        self.logger(f"Ollama response time: {end - start:.3f} seconds")
        try:
            input_tokens = completion.prompt_eval_count
            output_tokens = completion.eval_count
            # Print in textual terminal:
            # [MANAGER] Prompt tokens: <num_1>, Completion tokens: <num_2>
            self.logger(f"Prompt tokens: <{self.class_color}>{input_tokens}</{self.class_color}>, " + \
                        f"Completion tokens: <{self.class_color}>{output_tokens}</{self.class_color}>", log_type="manager")
        except Exception:
            pass

        return parsed

    def _build_user_content(self, user_text: str, images: Optional[Iterable[str]]) -> Dict[str, Any]:
        """Compose user content with text and optional list images."""
        content: Dict[str, Any] = {"role": "user", "content": user_text}
        if images:
            encoded_images: list[str] = []
            for image_path in images:
                try:
                    base64_image = self._encode_image(image_path)
                    encoded_images.append(base64_image)
                except Exception as e:
                    # Fail soft on a single bad image but continue with others

                    # Print in textual terminal:
                    # [MANAGER] Fail soft. Image '<image_path>' could not be encoded: <exception>
                    self.logger(f"Fail soft. Image '{image_path}' could not be encoded: {e}",
                                log_type="manager", log_color=0)
            if encoded_images:
                content["images"] = encoded_images
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

        # Append current user turn (dictionary with text + images (if any))
        messages.append(user_content)
        return messages
