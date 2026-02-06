import threading
import time



import rclpy
from rclpy.node import Node
from rclpy.task import Future

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

from eprosima_bot_interfaces.msg import EProsimaBotConfig, EProsimaBotConfigColor


import sys
import os
import termios
import tty



domain = "42"
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

class SharedNode(Node):
    def __init__(self, name: str = "vulcanai_shared_node"):
        super().__init__(name)
        # Dictionary to store created clients
        self._vulcan_clients = {}
        # Dictionary to store created publishers
        self._vulcan_publishers = {}

        # Ensure entities creation is thread-safe.
        self.node_lock = threading.Lock()

    def get_client(self, srv_type, srv_name):
        """
        Get a cached client for the specified service type and name or
        create a new one if it doesn't exist.
        """
        key = (srv_type, srv_name)
        with self.node_lock:
            if key not in self._vulcan_clients:
                client = self.create_client(srv_type, srv_name)
                self._vulcan_clients[key] = client
                self.get_logger().info(f"Created new client for {srv_name}")
            return self._vulcan_clients[key]

    def get_publisher(self, msg_type, topic_name, qos):
        """
        Get a cached publisher for the specified message type and topic name or
        create a new one if it doesn't exist.
        """
        key = (msg_type, topic_name)
        with self.node_lock:
            if key not in self._vulcan_publishers:
                publisher = self.create_publisher(msg_type, topic_name, qos)
                self._vulcan_publishers[key] = publisher
                self.get_logger().info(f"Created new publisher for {topic_name}")
            return self._vulcan_publishers[key]

    def wait_for_message(self, msg_type, topic: str, timeout_sec: float = None):
        """
        Block until a message is received or timeout expires.
        Subscriptions are created on demand and destroyed after use to avoid
        handling spins and callbacks in a separate thread.
        """
        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(msg_type, topic, callback, 10)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        return None

def _get_key():
    """Read one character from stdin (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def run():
    os.environ["ROS_DOMAIN_ID"] = "12"

    rclpy.init()
    node = SharedNode(name="vulcanai_shared_node")

    move_val = float(1)
    left_wheel = float(0.1)
    right_wheel = float(0.1)
    tray_val = float(0.1)

    pub = node.get_publisher(Twist, "/cmd_vel", qos=qos)
    config_pub = node.get_publisher(EProsimaBotConfig, "/config", qos=qos)
    msg = Twist()

    node.get_logger().info(
        "Keyboard control started:\n"
        "  W: forward\n"
        "  S: backward\n"
        "  A: rotate left\n"
        "  D: rotate right\n"
        "  SPACE: stop\n"
        "  R: Reset colors (disable LEDs)\n"
        "  B: Enable sound/beep\n"
        "  Q: quit"
    )

    try:
        while True:
            key = _get_key()

            msg.linear.x = 0.0
            msg.linear.y = 0.0

            if key.lower() == "w":
                msg.linear.x = +move_val
                msg.linear.y = +move_val
            elif key.lower() == "s":
                msg.linear.x = -move_val
                msg.linear.y = -move_val
            elif key.lower() == "a":
                msg.linear.x = +move_val
                msg.linear.y = -move_val
            elif key.lower() == "d":
                msg.linear.x = -move_val
                msg.linear.y = +move_val
            elif key == " ":
                pass  # stop
            elif key.lower() == "r":
                # Reset colors - disable LEDs
                config_msg = EProsimaBotConfig()
                config_msg.sound_enabled = False
                config_msg.set_leds = False
                config_msg.video_enabled = False
                # Initialize all 9 colors to black (0, 0, 0)
                config_msg.color = [EProsimaBotConfigColor(r=0, g=0, b=0) for _ in range(9)]
                config_pub.publish(config_msg)
                node.get_logger().info("Reset config: LEDs disabled, all colors set to black")
                continue
            elif key.lower() == "b":
                # Enable sound/beep
                config_msg = EProsimaBotConfig()
                config_msg.sound_enabled = True
                config_msg.set_leds = True
                config_msg.video_enabled = True
                # Initialize all 9 colors to white (255, 255, 255)
                config_msg.color = [EProsimaBotConfigColor(r=255, g=255, b=255) for _ in range(9)]
                config_pub.publish(config_msg)
                node.get_logger().info("Beep config: Sound enabled, LEDs enabled with white color")
                continue
            elif key.lower() == "q":
                node.get_logger().info("Exiting keyboard control.")
                break
            else:
                continue

            pub.publish(msg)
            node.get_logger().debug(
                f"cmd_vel → linear={left_wheel}, angular={right_wheel}"
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop the turtle when exiting
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)

    return {"success": True}



run()