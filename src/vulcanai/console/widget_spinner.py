# Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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


from rich.spinner import Spinner
from rich.text import Text
from textual.widgets import Static


class SpinnerStatus(Static):
    """
    Widget to display a spinner indicating LLM activity.
    It implements rich's Spinner and manages its display state,
    starting and stopping it as needed through SpinnerHook.
    """

    def __init__(self, logcontent, **kwargs) -> None:
        super().__init__(**kwargs)
        self.logcontent = logcontent
        self._spinner = Spinner("dots2", text="")
        self._timer = None
        self._forced_compact = False

    def on_mount(self) -> None:
        self._timer = self.set_interval(1 / 30, self._refresh, pause=True)
        self.display = False
        self.styles.height = 0

    def _refresh(self) -> None:
        self.update(self._spinner)

    def _log_is_filling_space(self) -> bool:
        """
        Determine if the log content is filling all available space.
        """
        visible = max(1, self.logcontent.size.height)
        lines = getattr(self.logcontent.document, "line_count", 0)
        return lines >= visible

    def start(self, text: str = "Querying LLM...") -> None:
        # Keep the log anchored at bottom if the user was already following output.
        if hasattr(self.logcontent, "is_near_vertical_scroll_end"):
            was_at_bottom = self.logcontent.is_near_vertical_scroll_end()
        else:
            was_at_bottom = self.logcontent.is_vertical_scroll_end
        self._spinner.text = Text(text, style="#0d87c0")
        self.display = True
        self.styles.height = 1
        if self._log_is_filling_space():
            self.logcontent.styles.height = "1fr"
            self._forced_compact = True
            self.logcontent.refresh(layout=True)
        else:
            self._forced_compact = False
        self.refresh(layout=True)
        if was_at_bottom:
            self.logcontent.scroll_end(animate=False, immediate=True, x_axis=False)
            # Re-anchor after layout has settled.
            self.call_after_refresh(self.logcontent.scroll_end, animate=False, immediate=True, x_axis=False)

        self._timer.resume()

    def stop(self) -> None:
        # Keep the log anchored at bottom if the user was already following output.
        if hasattr(self.logcontent, "is_near_vertical_scroll_end"):
            was_at_bottom = self.logcontent.is_near_vertical_scroll_end()
        else:
            was_at_bottom = self.logcontent.is_vertical_scroll_end
        self._timer.pause()
        self.display = False
        self.styles.height = 0
        if self._forced_compact:
            self.logcontent.styles.height = "auto"
            self._forced_compact = False
            self.refresh(layout=True)
            self.logcontent.refresh(layout=True)
        if was_at_bottom:
            self.logcontent.scroll_end(animate=False, immediate=True, x_axis=False)
            # Re-anchor after layout has settled.
            self.call_after_refresh(self.logcontent.scroll_end, animate=False, immediate=True, x_axis=False)

        self.update("")
