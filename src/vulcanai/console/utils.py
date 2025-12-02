import sys
# sipnner
from textual.timer import Timer

class StreamToTextual:
    """
    Class used to redirect the stdout/stderr streams in the textual terminal
    """

    def __init__(self, app, stream_name: str = "stdout"):
        self.app = app
        self.real_stream = getattr(sys, stream_name)

    def write(self, data: str):
        if not data:
            return

        # optional: still write to real stdout/stderr
        self.real_stream.write(data)
        self.real_stream.flush()

        if data.strip():
            # Ensure update happens on the app thread
            self.app.call_from_thread(self.app.append_log_text, data)
            self.app.render_log()

    def flush(self):
        self.real_stream.flush()

class SpinnerHook:
    """
    Single entrant spinner controller for console.
    - Starts the spinner on the LLM request.
    - Stops the spinner when LLM request is over.
    """

    def __init__(self, console):

        self.console = console

        # Spinner states
        self.spinner_timer: Timer | None = None
        self.spinner_frames = ["⠋","⠙","⠹","⠸","⠼","⠴","⠦","⠧","⠇","⠏"]
        self.spinner_frame_index = 0
        self.spinner_line_index: int | None = None

    def on_request_start(self, text: str = "Querying LLM...") -> None:
        """
        Create the spinner line at the end of the log and start updating it.
        """

        self.color = "#0d87c0"
        self.update_color = "#15B606"
        self.text = text

        # Check if it is already running
        if self.spinner_timer is not None:
            return

        # Initialized the class variables
        """self.spinner_line_index = len(self.console.log_lines)"""
        self.spinner_line_index = len(self.console.log_lines_dq)
        self.console._log(f"[{self.color}]{text}[/{self.color}]")
        self.spinner_frame_index = 0

        # Update every 0.1s
        self.spinner_timer = self.console.set_interval(0.1, self.update_spinner)
        # Update the terminal
        self.console.render_log()

    def update_spinner(self) -> None:
        """
        Timer callback. Rotate the spinner frame on the stored last log line.
        """

        # Check if the spinner is not running
        if self.spinner_line_index is None:
            return

        frame = self.spinner_frames[self.spinner_frame_index]
        self.spinner_frame_index = (self.spinner_frame_index + 1) % len(self.spinner_frames)

        # Update that specific line only
        """self.console.log_lines[self.spinner_line_index] = \
            f"[{self.update_color}]{frame}[/{self.update_color}] " + \
            f"[{self.color}]{self.text}[/{self.color}]"
        """
        self.console.log_lines_dq[self.spinner_line_index] = \
            f"[{self.update_color}]{frame}[/{self.update_color}] " + \
            f"[{self.color}]{self.text}[/{self.color}]"
        
        # Update the terminal
        self.console.render_log()

    def on_request_end(self) -> None:
        """
        Stop the spinner.
        Optional, replace the line with final_text.
        """

        # Check if the spinner is running
        if self.spinner_timer is not None:
            self.spinner_timer.stop()
            self.spinner_timer = None

        # Update the spinner message line
        if self.spinner_line_index is not None:
            """self.console.log_lines[self.spinner_line_index] += f"[{self.update_color}] Query finished![/{self.update_color}]" """
            self.console.log_lines_dq[self.spinner_line_index] += f"[{self.update_color}] Query finished![/{self.update_color}]"
            self.spinner_line_index = None
            self.console.render_log()


def attach_ros_logger_to_console(console, node):
    """
    Function that remove ROS node overlaping prints in the terminal
    """

    logger = node.get_logger()

    def info_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[INFO] {msg}[/gray]")

    def warn_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[WARN] {msg}[/gray]")

    def error_hook(msg, *args, **kwargs):
        console.call_from_thread(console._log, f"[gray]\[ROS] \[ERROR] {msg}[/gray]")

    logger.info = info_hook
    logger.warning = warn_hook
    logger.error = error_hook


def common_prefix(strings: str) -> str:
    if not strings:
        return ""

    common_prefix = strings[0]
    commands = strings[0]

    for i in range(1, len(strings)):
        commands += f"    {strings[i]}"

        tmp = ""
        n = min(len(common_prefix), len(strings[i]))
        j = 0

        while j < n:
            if common_prefix[j] != strings[i][j]:
                break
            tmp += common_prefix[j]

            j += 1

        if j < n:
            common_prefix = tmp

    return common_prefix, commands