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

from vulcanai import CompositeTool, vulcanai_tool


# Register dummy composite tool
@vulcanai_tool
class ComplexFileTool(CompositeTool):
    name = "complex_file_action"
    description = "A complex action using multiple tools from file"
    tags = ["complex", "action"]
    input_schema = [("param", "string")]
    output_schema = {"result": "bool"}
    version = "0.1"
    dependencies = ["file_tool", "new_file_tool", "other_file_tool"]

    def run(self, **kwargs):
        return {"result": True}
