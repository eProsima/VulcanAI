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

from vulcanai import AtomicTool, ValidationTool, vulcanai_tool


# Register dummy tools
@vulcanai_tool
class FileTool(AtomicTool):
    name = "file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("x", "float"), ("y", "float"), ("z", "float")]
    output_schema = {"arrived": "bool"}
    version = "0.1"

    def run(self, **kwargs):
        return {"arrived": True}


@vulcanai_tool
class NewFileTool(AtomicTool):
    name = "new_file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("label", "string")]
    output_schema = {"found": "bool"}
    version = "0.1"

    def run(self, **kwargs):
        return {"found": True}


@vulcanai_tool
class OtherFileTool(AtomicTool):
    name = "other_file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("text", "string")]
    output_schema = {"spoken": "bool"}
    version = "0.1"

    def run(self, **kwargs):
        return {"spoken": True}


@vulcanai_tool
class AnotherValidationTool(ValidationTool):
    name = "another_validation_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = []
    output_schema = {"valid": "bool"}
    version = "0.1"

    def run(self, **kwargs):
        return {"valid": True}


class NoDecoratorTool(AtomicTool):
    name = "no_decorator_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("text", "string")]
    output_schema = {"spoken": "bool"}
    version = "0.1"

    def run(self, **kwargs):
        return {"spoken": True}
