"""
This file contains example tools to interact with safebot and
demonstrate how to create custom tools compatible with VulcanAI.
"""

import time
import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from vulcanai import AtomicTool, CompositeTool, vulcanai_tool

from eprosima_bot_interfaces.msg import EProsimaBotConfig, EProsimaBotConfigColor

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
                publisher = self.create_publisher(msg_type, topic_name, qos=qos)
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


@vulcanai_tool
class MoveSafeBotTool(AtomicTool):
    name = "move_safe_bot"
    description = (
        "Move the robot 'SafeBot' with 'left_wheel' and 'right_wheel' input variables "
        "with values [1, -1] by publishing the message 'duration' times (seconds)."
        "To move forward/up left_wheel=1 and right_wheel=1."
        "To move backward/down left_wheel=-1 and right_wheel=-1."
        "To move left left_wheel=1 and right_wheel=-1."
        "To move right left_wheel=-1 and right_wheel=1."
        "Use left_wheel=0 and right_wheel=0 to stop."
        "'success' indicates if the command was sent correctly."
    )
    tags = ["safebot" "move", "velocity", "forward", "backward", "left", "right", "cmd_vel", "stop", "tray"]
    input_schema = [
        ("left_wheel", "float"),
        ("right_wheel", "float"),
        ("tray", "float"),
        ("duration", "int"),
    ]
    output_schema = [{"success": "bool"}]


    """
    cmd_vel_topic_name_{"rt/cmd_vel"};
    cmd_vel_type_name_{"geometry_msgs::msg::dds_::Twist_"};

    ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;

        - Control (10 Hz):
            self.right_val = self.power * (-1 * self.joystick.axes[1])
            self.left_val = self.power * (-1 * self.joystick.axes[1])
            linear_vel = (self.right_val + self.left_val)/2
            linear_vel = abs(linear_vel)
            self.right_val -= (self.joystick.axes[2]) * self.power * (0.95 - linear_vel)/0.95
            self.left_val += (self.joystick.axes[2]) * self.power * (0.95 - linear_vel)/0.95
            msg.linear.y = self.right_val
            msg.linear.x = self.left_val
            msg.linear.z = self.tray
    """

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        name = "SafeBot"
        # left_wheel =  float(kwargs.get("linear", 0.0))
        # right_wheel = float(kwargs.get("angular", 0.0))
        # tray_val = float(kwargs.get("tray", 10.0))
        # duration = int(kwargs.get("duration", 10.0))

        # # Create publisher in topic of the SafeBot
        # pub = node.get_publisher(Twist, f"/rt/cmd_vel")
        # # Type: geometry_msgs::msg::dds_::Twist_
        # msg = Twist()

        # # Move SafeBot
        # msg.linear.x = left_wheel
        # msg.linear.y = right_wheel

        # # Ensure the tray does not drag on the floor
        # tray_val = max(tray_val, 10)
        # tray_val = min(tray_val, 80)
        # msg.linear.z = tray_val


        # for idx in range(duration):
        #     node.get_logger().info(f"Publishing message {idx + 1} to topic /{name}/cmd_vel: linear={msg.linear.x}, angular={msg.linear.y}")
        #     pub.publish(msg)
        #     time.sleep(1)

        # return {"success": True}

        # TODO
        # ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        # HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        # DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;


        left_wheel =  float(kwargs.get("left_wheel", 0.0))
        right_wheel = float(kwargs.get("right_wheel", 0.0))
        tray_val = float(kwargs.get("tray", 10.0))
        duration = int(kwargs.get("duration", 10.0))

        # Ensure the tray does not drag on the floor
        tray_val = max(tray_val, 10.0)
        tray_val = min(tray_val, 50.0)

        #pub = node.get_publisher(Twist, f"/rt/cmd_vel")
        pub = node.get_publisher(Twist, "/cmd_vel", qos=qos)

        msg = Twist()
        msg.linear.x = left_wheel * 0.5
        msg.linear.y = right_wheel * 0.5
        msg.linear.z = tray_val

        sleep_time = 0.05

        # left or right
        if left_wheel < 0 and right_wheel > 0 or \
            left_wheel > 0 and right_wheel < 0 :
            sleep_time = 0.138

        start_time = time.time()
        msg_count = 0
        while time.time() - start_time < duration:
            pub.publish(msg)
            msg_count += 1
            time.sleep(sleep_time)


        node.get_logger().info(f"Published the message {msg_count} times to topic /cmd_vel: " \
                                   f"left_wheel={left_wheel}, " \
                                   f"right_wheel={right_wheel}, " \
                                   f"tray={tray_val}")

        # for idx in range(10):
        #     node.get_logger().info(f"Publishing message {idx + 1} to topic /cmd_vel: " \
        #                            f"left_wheel={left_wheel}, " \
        #                            f"right_wheel={right_wheel}, " \
        #                            f"tray={tray_val}")
            #node._log(f"Publishing message {idx + 1} to topic /{name}/cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
            # pub.publish(msg)
            # time.sleep(1)
        return {"success": True}

@vulcanai_tool
class ConfigSafeBotTool(AtomicTool):
    name = "config_safe_bot"
    description = (
        "Configure the robot 'SafeBot' with a color, sound or video flags"
        "If the color flag is activated it will change the leds of the robot"
        "If the sound is activated the robot will make a sound"
        "If the video flag is activated the robot will turn on the camera"
    )
    tags = ["safebot" "config", "color", "sound", "led", "bep"]
    input_schema = [
        ("color", "bool"),
        ("sound", "bool"),
        ("video", "bool"),
        ("duration", "int"),
    ]
    output_schema = [{"success": "bool"}]


    """
    config_topic_name_{"rt/config"};
    config_type_name_{"eprosimabotconfig_msgs::msg::dds_::EProsimaBotConfig_"};
    ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;

        - Config:
            EProsimaBotConfig.msg
                bool sound_enabled
                bool set_leds
                EProsimaBotConfigColor[9] color
                bool video_enabled

            EProsimaBotConfigColor.msg
                uint8 r
                uint8 g
                uint8 b

            self.config_msg.sound_enabled = True
            self.config_msg.video_enabled = True
            self.config_msg.set_leds = True
            color = EProsimaBotConfigColor()
            color.r = int(r * 255)
            color.g = int(g * 255)
            color.b = int(b * 255)
            self.config_msg.color[0] = color
            self.config_msg.color[1] = color
            self.config_msg.color[2] = color
            self.config_msg.color[3] = color
            self.config_msg.color[4] = color
            self.config_msg.color[5] = color
            self.config_msg.color[6] = color
            self.config_msg.color[7] = color
            self.config_msg.color[8] = color
    """

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        # E.G.:
        # Config robot with sound and color active, the video disabled

        name = "SafeBot"
        color = kwargs.get("color", True)
        sound = kwargs.get("sound", False)
        video = False #kwargs.get("video", False)  # TODO
        duration = int(kwargs.get("duration", 10.0))

        # Create publisher in topic of the SafeBot
        pub = node.get_publisher(EProsimaBotConfig, f"/config", qos)
        # Type: geometry_msgs::msg::dds_::Twist_
        msg = EProsimaBotConfig()

        # Move SafeBot
        msg.sound_enabled = sound
        msg.video_enabled = video

        # 9 72 126
        if color:
            rgb_color = EProsimaBotConfigColor()
            rgb_color.r = 0
            rgb_color.g = 0
            rgb_color.b = 0

            msg.color = [rgb_color for _ in range(9)]



        for idx in range(100):
            node.get_logger().info(f"Publishing message {idx + 1} to topic /rt/config: " \
                        f"sound_enabled={sound}, " \
                        f"video_enabled={video}, " \
                        f"color=[r({rgb_color.r}), g({rgb_color.g}), b({rgb_color.b})]")
            pub.publish(msg)
            #time.sleep(1)

        return {"success": True}