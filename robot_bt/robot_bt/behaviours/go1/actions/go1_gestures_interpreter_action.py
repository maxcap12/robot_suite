"""Hand-gesture interpreter for the Unitree Go1.

Translates the gestures published by the hand_gestures plugin into the
interface exposed by the unitree_ros driver (https://github.com/snt-arg/unitree_ros):

- geometry_msgs/Twist on ``/cmd_vel``     -> walking velocity
- std_msgs/Empty     on ``/stand_up``    -> stand up
- std_msgs/Empty     on ``/stand_down``  -> lie down

Gesture mapping (both hands must be visible, same as the Spot interpreter):

| Left hand    | Right hand   | Action        |
| ------------ | ------------ | ------------- |
| Open_Palm    | Pointing_Up  | Move forward  |
| Open_Palm    | Victory      | Move backward |
| Closed_Fist  | Pointing_Up  | Move left     |
| Pointing_Up  | Closed_Fist  | Move right    |
| Closed_Fist  | Thumb_Up     | Rotate left   |
| Thumb_Up     | Closed_Fist  | Rotate right  |
| Open_Palm    | Thumb_Up     | Stand up      |
| Open_Palm    | Thumb_Down   | Stand down    |

Any other combination (or fewer than two hands) stops the robot.

Safety: velocity commands are only published while this behaviour is being
ticked (i.e. while hand control mode is selected). When the BT stops ticking it
(mode switched, detector failing, ...) a single zero Twist is published so the
robot does not keep walking with the last command. Landmarks older than
``landmarks_timeout`` seconds are ignored (robot stops).
"""

import time
from typing import Optional

import py_trees
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from hand_gestures_msgs.msg import Landmarks

from robot_bt.behaviours.shared.actions import Action


class Go1GesturesInterpreterAction(Action):
    # Topics (must match the unitree_ros driver configuration)
    landmarks_topic: str = "/hand/landmarks"
    cmd_vel_topic: str = "/cmd_vel"
    stand_up_topic: str = "/stand_up"
    stand_down_topic: str = "/stand_down"

    # Motion settings
    linear_speed: float = 0.3  # m/s
    lateral_speed: float = 0.2  # m/s
    angular_speed: float = 0.6  # rad/s

    # Timing settings
    landmarks_timeout: float = 0.5  # s, ignore landmarks older than this
    inactive_timeout: float = 0.3  # s, without ticks -> considered inactive
    publish_period: float = 1 / 20  # s, max rate for cmd_vel publishing
    posture_cooldown: float = 2.0  # s, min time between stand up/down commands

    def __init__(self, name: str, node: Optional[Node] = None) -> None:
        super().__init__(name=name, bt_node=node)
        if self.node is None:
            raise ValueError("no ROS 2 node available.")

        self.node.create_subscription(
            Landmarks, self.landmarks_topic, self.landmarks_callback, 10
        )
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.stand_up_pub = self.node.create_publisher(Empty, self.stand_up_topic, 1)
        self.stand_down_pub = self.node.create_publisher(
            Empty, self.stand_down_topic, 1
        )
        self.watchdog = self.node.create_timer(0.1, self.watchdog_callback)

        self.last_landmarks: Optional[Landmarks] = None
        self.last_landmarks_time = 0.0
        self.last_tick_time = 0.0
        self.last_publish_time = 0.0
        self.last_posture_time = 0.0
        self.is_active = False
        self.is_down = False

    # ------------------------------------------------------------------ #
    # ROS callbacks
    # ------------------------------------------------------------------ #
    def landmarks_callback(self, msg: Landmarks) -> None:
        self.last_landmarks = msg
        self.last_landmarks_time = time.monotonic()

    def watchdog_callback(self) -> None:
        """Stops the robot once when hand control stops being ticked."""
        if (
            self.is_active
            and time.monotonic() - self.last_tick_time > self.inactive_timeout
        ):
            self.node.get_logger().info("Hand control inactive, stopping Go1")
            self.cmd_vel_pub.publish(Twist())
            self.is_active = False

    # ------------------------------------------------------------------ #
    # Gesture handling
    # ------------------------------------------------------------------ #
    def gestures_to_twist(self, left: str, right: str) -> Twist:
        cmd = Twist()

        if left == "Open_Palm" and right == "Pointing_Up":
            cmd.linear.x = self.linear_speed
        elif left == "Open_Palm" and right == "Victory":
            cmd.linear.x = -self.linear_speed
        elif left == "Closed_Fist" and right == "Pointing_Up":
            cmd.linear.y = self.lateral_speed
        elif left == "Pointing_Up" and right == "Closed_Fist":
            cmd.linear.y = -self.lateral_speed
        elif left == "Closed_Fist" and right == "Thumb_Up":
            cmd.angular.z = self.angular_speed
        elif left == "Thumb_Up" and right == "Closed_Fist":
            cmd.angular.z = -self.angular_speed

        return cmd

    def handle_posture(self, left: str, right: str) -> None:
        now = time.monotonic()
        if now - self.last_posture_time < self.posture_cooldown:
            return

        if left == "Open_Palm" and right == "Thumb_Up":
            self.node.get_logger().info("Go1: standing up")
            self.stand_up_pub.publish(Empty())
            self.is_down = False
            self.last_posture_time = now
        elif left == "Open_Palm" and right == "Thumb_Down":
            self.node.get_logger().info("Go1: standing down")
            self.cmd_vel_pub.publish(Twist())
            self.stand_down_pub.publish(Empty())
            self.is_down = True
            self.last_posture_time = now

    def update(self) -> py_trees.common.Status:
        now = time.monotonic()
        self.last_tick_time = now
        self.is_active = True

        cmd = Twist()
        msg = self.last_landmarks
        fresh = msg is not None and now - self.last_landmarks_time < self.landmarks_timeout

        if fresh:
            left = msg.left_hand.gesture if msg.left_hand.handedness != "" else "None"
            right = (
                msg.right_hand.gesture if msg.right_hand.handedness != "" else "None"
            )
            if left != "None" and right != "None":
                self.handle_posture(left, right)
                cmd = self.gestures_to_twist(left, right)

        if self.is_down:
            cmd = Twist()

        if now - self.last_publish_time >= self.publish_period:
            self.cmd_vel_pub.publish(cmd)
            self.last_publish_time = now

        return py_trees.common.Status.SUCCESS
