
from vulcanai import vulcanai_tool
from vulcanai import CompositeTool
from vulcanai import ArgValue

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
