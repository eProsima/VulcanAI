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

from prompt_toolkit import PromptSession
from rich.progress import Progress, SpinnerColumn, TextColumn
import os

from vulcanai.logger import console

class VulcanConsole:
    def __init__(self):
        self.manager = None
        self.session = PromptSession()
        self.last_plan = None
        self.last_bb = None

        self.init_manager()

    def run(self):
        self.print("VulcanAI Interactive Console")
        self.print("Type 'exit' to quit.\n")

        while True:
            try:
                user_input = self.session.prompt("[USER] >>> ")
                if user_input.strip().lower() in ("exit", "quit"):
                    break

                if user_input.startswith("/"):
                    self.handle_command(user_input)
                    continue

                with Progress(
                    SpinnerColumn(spinner_name="dots2"),
                    TextColumn("[blue]Querying LLM...[/blue]"),
                    console=console,
                ) as progress:
                    task = progress.add_task("llm", start=False)
                    result = self.manager.handle_user_request(user_input, context={})
                    progress.remove_task(task)

                self.last_plan = result.get("plan", None)
                self.last_bb = result.get("blackboard", None)

                self.print(f"Output of plan: {result.get('blackboard', {None})}")

            except KeyboardInterrupt:
                console.print("[yellow]Exiting...[/yellow]")
                break
            except EOFError:
                console.print("[yellow]Exiting...[/yellow]")
                break

    def init_manager(self):
        from vulcanai.manager_plan import PlanManager

        console.print("[console]Initializing Manager...[/console]")
        model = "gpt-5-nano"
        self.manager = PlanManager(model=model, k=7)
        self.print(f"Manager initialized with model '{model}'.")

    def handle_command(self, cmd: str):
        """Procesa comandos internos :tools, :plan, :bb, :clear"""
        if cmd == "/help":
            help_msg = (
                "Available commands:\n"
                "/help           - Show this help message\n"
                "/tools          - List available tools\n"
                "/change_k <int> - Change the 'k' value for the top_k algorithm selection\n"
                "/plan           - Show the last generated plan\n"
                "/rerun          - Rerun the last plan\n"
                "/bb             - Show the last blackboard state\n"
                "/clear          - Clear the console screen\n"
                "exit            - Exit the console\n"
                "Query any other text to process it with the LLM and execute the plan generated."
            )
            self.print(help_msg)

        elif cmd == "/tools":
            help_msg = f"\nAvailable tools (current index k={self.manager.k}):\n"
            for tool in self.manager.registry.tools.values():
                help_msg += f"- {tool.name}: {tool.description}\n"
            self.print(help_msg)

        elif cmd.startswith("/change_k"):
            parts = cmd.split()
            if len(parts) != 2 or not parts[1].isdigit():
                self.print(f"[error]Usage: /change_k <int>[/error] - Actual k is {self.manager.k}")
                return
            new_k = int(parts[1])
            self.manager.k = new_k
            self.print(f"[console]Changed k to {new_k}[/console]")

        elif cmd == "/plan":
            if self.last_plan:
                self.print("Last generated plan:")
                console.print(self.last_plan)
            else:
                self.print("No plan has been generated yet.")

        elif cmd == "/rerun":
            if self.last_plan:
                self.print("Rerunning last plan...")
                result = self.manager.executor.run(self.last_plan, self.manager.bb)
                self.last_bb = result.get("blackboard", None)
                self.print(f"Output of rerun: {result.get('blackboard', {None})}")
            else:
                self.print("No plan to rerun.")

        elif cmd == "/bb":
            if self.last_bb:
                self.print("Last blackboard state:")
                console.print(self.last_bb)
            else:
                self.print("No blackboard available.")

        elif cmd == "/clear":
            os.system("clear")

        else:
            self.print(f"[error]Unknown command {cmd}[/error]")


    def print(self, msg: str):
        console.print(f"[console]{msg}[/console]")


def main():
    console = VulcanConsole()
    console.run()


if __name__ == "__main__":
    main()
