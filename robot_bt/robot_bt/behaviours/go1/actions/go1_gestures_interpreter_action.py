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

How the driver behaves (and why this class is written this way):

- The driver re-sends its *last* command to the robot at 500 Hz, and every
  ``/cmd_vel`` message switches the robot to walking mode. A ``/cmd_vel``
  received right after a stand up/down command therefore interrupts the
  posture transition (the robot jumps up into walking mode, fast and unstable).
- 400 ms after the last ``/cmd_vel`` the driver itself sends a zero walking
  command, which has the same effect.

So:

- ``/cmd_vel`` is only published while a motion gesture is held (at most
  every ``publish_period``), plus one zero Twist when the motion stops.
- Nothing is published while the robot is down, nor during
  ``posture_transition_time`` after a stand up.
- A stand up/down command is sent only once at least ``driver_reset_delay``
  has passed since the last ``/cmd_vel`` (the robot is stopped first).
- A stand up/down command is sent once per gesture (it must be released and
  made again to send another one), and at most every ``posture_cooldown``
  (a gesture held during the cooldown is sent when it ends).
- The posture is also tracked from ``/stand_up`` and ``/stand_down``
  published by other nodes (e.g. the control station `t` / `l` keys).

Safety: velocity commands are only published while this behaviour is being
ticked (i.e. while hand control mode is selected). When the BT stops ticking it
(mode switched, detector failing, ...) the robot is stopped if it was moving.
Landmarks older than ``landmarks_timeout`` seconds are ignored (robot stops).
"""

import time
from typing import Optional

import py_trees
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from hand_gestures_msgs.msg import Landmarks

from robot_bt.behaviours.shared.actions import Action

STAND_UP = "stand_up"
STAND_DOWN = "stand_down"


def is_zero(cmd: Twist) -> bool:
    return (
        cmd.linear.x == 0.0
        and cmd.linear.y == 0.0
        and cmd.linear.z == 0.0
        and cmd.angular.x == 0.0
        and cmd.angular.y == 0.0
        and cmd.angular.z == 0.0
    )


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
    posture_transition_time: float = 2.0  # s, no cmd_vel after a stand up
    driver_reset_delay: float = 0.45  # s, driver sends a zero walk cmd 0.4 s after the last cmd_vel

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
        # Track the posture from any source (gestures, control station `t`/`l`
        # keys, ...), otherwise standing up with the keyboard after a stand-down
        # gesture would leave hand control blocked.
        self.node.create_subscription(
            Empty, self.stand_up_topic, self.stand_up_callback, 1
        )
        self.node.create_subscription(
            Empty, self.stand_down_topic, self.stand_down_callback, 1
        )
        self.watchdog = self.node.create_timer(0.05, self.watchdog_callback)

        self.last_landmarks: Optional[Landmarks] = None
        self.last_landmarks_time = 0.0
        self.last_tick_time = 0.0
        self.last_publish_time = 0.0
        self.last_cmd_was_zero = True
        self.last_posture_time = -self.posture_cooldown
        self.sent_posture_gesture: Optional[str] = None  # held, already sent
        self.pending_posture: Optional[str] = None
        self.transition_end_time = 0.0
        self.is_active = False
        self.is_down = False  # the driver stands the robot up when it starts

    # ------------------------------------------------------------------ #
    # ROS callbacks
    # ------------------------------------------------------------------ #
    def landmarks_callback(self, msg: Landmarks) -> None:
        self.last_landmarks = msg
        self.last_landmarks_time = time.monotonic()

    def stand_up_callback(self, _: Empty) -> None:
        self._set_standing_up(time.monotonic())

    def stand_down_callback(self, _: Empty) -> None:
        self.is_down = True

    def watchdog_callback(self) -> None:
        now = time.monotonic()
        # A posture command requested just before leaving hand mode is still sent
        self.process_pending_posture(now)

        if self.is_active and now - self.last_tick_time > self.inactive_timeout:
            self.node.get_logger().info("Hand control inactive, stopping Go1")
            self.stop_if_moving(now)
            self.is_active = False

    # ------------------------------------------------------------------ #
    # Command helpers
    # ------------------------------------------------------------------ #
    def publish_cmd_vel(self, cmd: Twist, now: float) -> None:
        self.cmd_vel_pub.publish(cmd)
        self.last_publish_time = now
        self.last_cmd_was_zero = is_zero(cmd)

    def stop_if_moving(self, now: float) -> None:
        if not self.last_cmd_was_zero:
            self.publish_cmd_vel(Twist(), now)

    def _set_standing_up(self, now: float) -> None:
        self.is_down = False
        self.transition_end_time = now + self.posture_transition_time

    def request_posture(self, posture: str, now: float) -> None:
        self.node.get_logger().info(f"Go1: {posture.replace('_', ' ')} requested")
        self.pending_posture = posture
        self.last_posture_time = now
        self.stop_if_moving(now)
        self.process_pending_posture(now)

    def process_pending_posture(self, now: float) -> None:
        if self.pending_posture is None:
            return
        # Wait until the driver can't send its own zero walking command anymore
        if now - self.last_publish_time < self.driver_reset_delay:
            return

        if self.pending_posture == STAND_UP:
            self.node.get_logger().info("Go1: standing up")
            self.stand_up_pub.publish(Empty())
            self._set_standing_up(now)
        else:
            self.node.get_logger().info("Go1: standing down")
            self.stand_down_pub.publish(Empty())
            self.is_down = True
        self.pending_posture = None

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

    @staticmethod
    def gestures_to_posture(left: str, right: str) -> Optional[str]:
        if left == "Open_Palm" and right == "Thumb_Up":
            return STAND_UP
        if left == "Open_Palm" and right == "Thumb_Down":
            return STAND_DOWN
        return None

    def update(self) -> py_trees.common.Status:
        now = time.monotonic()
        self.last_tick_time = now
        self.is_active = True

        left, right = "None", "None"
        msg = self.last_landmarks
        if msg is not None and now - self.last_landmarks_time < self.landmarks_timeout:
            if msg.left_hand.handedness != "":
                left = msg.left_hand.gesture
            if msg.right_hand.handedness != "":
                right = msg.right_hand.gesture

        # Stand up / down: once per gesture (it must be released before it can
        # be sent again). A gesture made during the cooldown is sent when it ends.
        posture = self.gestures_to_posture(left, right)
        if posture != self.sent_posture_gesture:
            self.sent_posture_gesture = None
        if (
            posture is not None
            and self.sent_posture_gesture is None
            and now - self.last_posture_time >= self.posture_cooldown
        ):
            self.request_posture(posture, now)
            self.sent_posture_gesture = posture
        self.process_pending_posture(now)

        # Don't send any velocity while a posture command is pending, while the
        # robot is down or while it is standing up: it would interrupt the motion
        if (
            self.pending_posture is not None
            or self.is_down
            or now < self.transition_end_time
        ):
            return py_trees.common.Status.SUCCESS

        cmd = Twist()
        if left != "None" and right != "None":
            cmd = self.gestures_to_twist(left, right)

        if is_zero(cmd):
            self.stop_if_moving(now)  # one zero Twist when the motion stops
        elif now - self.last_publish_time >= self.publish_period:
            self.publish_cmd_vel(cmd, now)

        return py_trees.common.Status.SUCCESS
