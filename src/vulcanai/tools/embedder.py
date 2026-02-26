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

import os
from typing import Optional

import numpy as np

# Keep Hugging Face download progress bars out of redirected Textual stdout/stderr.
os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")

from sentence_transformers import SentenceTransformer

from vulcanai.console.logger import VulcanAILogger

_HF_DOWNLOAD_INFO_PRINTED = False


def info_msg_hf_model_loading(model_name: str, logger: Optional[VulcanAILogger] = None) -> None:
    global _HF_DOWNLOAD_INFO_PRINTED

    if _HF_DOWNLOAD_INFO_PRINTED:
        return

    msg = f"Hugging Face is loading '{model_name}'. If the model is not cached yet, files are being downloaded..."
    if logger is not None:
        logger.log_console(msg)
    else:
        VulcanAILogger.log_console(msg)

    _HF_DOWNLOAD_INFO_PRINTED = True


class SBERTEmbedder:
    def __init__(self, model_name="all-MiniLM-L6-v2", logger: Optional[VulcanAILogger] = None):
        info_msg_hf_model_loading(model_name, logger)
        self.model = SentenceTransformer(model_name)

    def embed(self, text: str) -> np.ndarray:
        vec = self.model.encode(text, convert_to_numpy=True)
        return vec / np.linalg.norm(vec)

    def raw_embed(self, text: str):
        return self.model.encode(text, convert_to_numpy=False)

    def similarity(self, vecs1, vecs2):
        """Compute cosine similarity between two sets of vectors."""
        return self.model.similarity(vecs1, vecs2)
