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

import argparse
from prompt_toolkit import PromptSession
from rich.progress import Progress, SpinnerColumn, TextColumn
import os

from vulcanai.console.logger import console

class VulcanConsole:
    def __init__(self, model: str = "gpt-5-nano", k: int = 7):
        self.manager = None
        self.session = PromptSession()
        self.last_plan = None
        self.last_bb = None

        self.model = model
        self.k = k

        self.init_manager()

    def run(self):
        self.print("VulcanAI Interactive Console")
        self.print("Type 'exit' to quit.\n")

        while True:
            try:
                user_input = self.session.prompt("[USER] >>> ")
                if user_input.strip().lower() in ("exit", "quit"):
                    break

                # Internal commands start with /<command>
                if user_input.startswith("/"):
                    self.handle_command(user_input)
                    continue

                # Check for image input. Must be always at the end of the input
                images = []
                if "--image=" in user_input:
                    images = self.get_images(user_input)

                # Query LLM
                try:
                    with Progress(
                        SpinnerColumn(spinner_name="dots2"),
                        TextColumn("[blue]Querying LLM...[/blue]"),
                        console=console,
                    ) as progress:
                        task = progress.add_task("llm", start=False)
                        plan = self.manager.get_plan_from_user_request(user_input, context={"images": images})
                        progress.remove_task(task)
                except Exception as e:
                    self.print(f"[error]Error generating plan: {e}[/error]")
                    continue

                # Execute plan
                try:
                    result = self.manager.execute_plan(plan)
                except Exception as e:
                    self.print(f"[error]Error executing plan: {e}[/error]")
                    continue

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
        from vulcanai.core.manager_plan import PlanManager
        console.print("[console]Initializing Manager...[/console]")
        self.manager = PlanManager(model=self.model, k=self.k)
        self.print(f"Manager initialized with model '{self.model}'.")

    def handle_command(self, cmd: str):
        """Procesa comandos internos :tools, :plan, :bb, :clear"""
        if cmd == "/help":
            help_msg = (
                "Available commands:\n"
                "/help           - Show this help message\n"
                "/tools          - List available tools\n"
                "/change_k <int> - Change the 'k' value for the top_k algorithm selection\n"
                "/history <int>  - Change the history depth or show the current value if no <int> is provided\n"
                "/show_history   - Show the current history\n"
                "/plan           - Show the last generated plan\n"
                "/rerun          - Rerun the last plan\n"
                "/bb             - Show the last blackboard state\n"
                "/clear          - Clear the console screen\n"
                "exit            - Exit the console\n"
                "Query any other text to process it with the LLM and execute the plan generated."
                "Add --image=<path> to include images in the query. It can be used multiple times to add more images."
                " Example: '<user_prompt> --image=/path/to/image1 --image=/path/to/image2'"
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
            self.print(f"Changed k to {new_k}")

        elif cmd.startswith("/history"):
            parts = cmd.split()
            if len(parts) == 1:
                self.print(f"Current history depth is {self.manager.history_depth}")
                return
            if len(parts) != 2 or not parts[1].isdigit():
                self.print(f"[error]Usage: /history <int>[/error] - Actual history depth is {self.manager.history_depth}")
                return
            new_hist = int(parts[1])
            self.manager.update_history_depth(new_hist)

        elif cmd == "/show_history":
            if not self.manager.history:
                self.print("No history available.")
                return
            help_msg = "\nCurrent history (oldest first):\n"
            for i, (user_text, plan_summary) in enumerate(self.manager.history):
                help_msg += f"{i+1}. User: {user_text}\n   Plan summary: {plan_summary}\n"
            self.print(help_msg)

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

    def get_images(self, user_input: str):
        parts = user_input.split()
        images = []

        for part in parts:
            if part.startswith("--image="):
                images.append(part[len("--image="):])
        return images


def main():
    parser = argparse.ArgumentParser(description="VulcanAI Interactive Console")
    parser.add_argument(
        "--model", type=str, default="gpt-5-nano",
        help="LLM model to used in the agent (ej: gpt-5-nano, gemini-2.0-flash, etc.)"
    )
    parser.add_argument(
        "--register-from-file", type=str, nargs="*", default=[],
        help="Register tools from a python file (or multiple files)"
    )
    parser.add_argument(
        "--register-from-entry-point", type=str, nargs="*", default=[],
        help="Register tools from a python entry-point (or multiple entry-points)"
    )
    parser.add_argument(
        "-k", type=int, default=7,
        help="Maximum number of tools to pass to the LLM"
    )

    args = parser.parse_args()
    console = VulcanConsole(model=args.model, k=args.k)
    if args.register_from_file:
        for file in args.register_from_file:
            console.manager.register_tools_from_file(file)
    if args.register_from_entry_point:
        for entry_point in args.register_from_entry_point:
            console.manager.register_tools_from_entry_points(entry_point)
    console.run()


if __name__ == "__main__":
    main()
