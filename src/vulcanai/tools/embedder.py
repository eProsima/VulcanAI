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

import numpy as np
from sentence_transformers import SentenceTransformer

class SBERTEmbedder:
    def __init__(self, model_name="all-MiniLM-L6-v2"):
        self.model = SentenceTransformer(model_name)

    def embed(self, text: str) -> np.ndarray:
        vec = self.model.encode(text, convert_to_numpy=True)
        return vec / np.linalg.norm(vec)

    def raw_embed(self, text: str):
        return self.model.encode(text, convert_to_numpy=False)

    def similarity(self, vecs1, vecs2):
        """ Compute cosine similarity between two sets of vectors. """
        return self.model.similarity(vecs1, vecs2)
