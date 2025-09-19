
from vulcanai.tool_registry import vulcanai_tool
from vulcanai.tools import AtomicTool
from vulcanai.plan_types import ArgValue

# Register dummy tools
@vulcanai_tool
class FileTool(AtomicTool):
    name = "file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("x", "float"), ("y", "float"), ("z", "float")]
    output_schema = {"arrived": "bool"}
    version = "0.1"
    def run(self, **kwargs): return {"arrived": True}

@vulcanai_tool
class NewFileTool(AtomicTool):
    name = "new_file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("label", "string")]
    output_schema = {"found": "bool"}
    version = "0.1"
    def run(self, **kwargs): return {"found": True}

@vulcanai_tool
class OtherFileTool(AtomicTool):
    name = "other_file_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("text", "string")]
    output_schema = {"spoken": "bool"}
    version = "0.1"
    def run(self, **kwargs): return {"spoken": True}

class NoDecoratorTool(AtomicTool):
    name = "no_decorator_tool"
    description = "Irrelevant"
    tags = ["None"]
    input_schema = [("text", "string")]
    output_schema = {"spoken": "bool"}
    version = "0.1"
    def run(self, **kwargs): return {"spoken": True}
