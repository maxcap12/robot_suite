from __future__ import annotations
from typing import Any, Optional
import os

from hand_gestures.models.mp_model import MediaPipeGesturesRecognizer

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"  # Suppress TensorFlow logging
import rclpy
import cv_bridge
import hand_gestures.helpers as helpers

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from plugin_base.plugin_base import PluginNode, NodeState
from hand_gestures_msgs.msg import Landmarks
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory


class LandmarkDetectorNode(PluginNode):
    """Node to detect hand landmarks from an input image

    Subscriptions:
        - /camera/image_raw (sensor_msgs/Image): Input image

    Publications:
        - /hand/landmarks (hand_gestures_msgs/Landmarks): Detected hand landmarks

    Parameters:
        - img_input_topic (str): Input image topic [default: /camera/image_raw]
        - landmarks_topic (str): Output landmarks topic [default: /hand/landmarks]
        - num_hands (int): Maximum number of hands to detect [default: 2]
        - min_detection_confidence (float): Minimum confidence to detect a hand [default: 0.5]
        - min_tracking_confidence (float): Minimum confidence to track a hand [default: 0.5]
    """

    img_input_topic: str = "/camera/image_raw"
    landmarks_topic: str = "/hand/landmarks"
    input_img_sub: Subscription
    landmarks_pub: Publisher
    received_image_msg = None

    # Hand landmarker parameters
    num_hands: int = 2
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.load_parameters()

        self.input_img_sub = self.create_subscription(
            Image, self.img_input_topic, self.img_input_callback, 10
        )

        self.landmarks_pub = self.create_publisher(Landmarks, self.landmarks_topic, 10)

        self.pkg_share_dir = get_package_share_directory("hand_gestures")
        self.bridge = cv_bridge.CvBridge()

        self.model = MediaPipeGesturesRecognizer(
            model_path=os.path.join(
                self.pkg_share_dir, "config", "gesture_recognizer.task"
            ),
            num_hands=self.num_hands,
            min_det_conf=self.min_detection_confidence,
            min_track_conf=self.min_tracking_confidence,
        )

        self.prev_image_msg = None

        

    def load_parameters(self) -> None:
        self.declare_parameter("img_input_topic", self.img_input_topic)
        self.declare_parameter("landmarks_topic", self.landmarks_topic)
        self.declare_parameter("num_hands", self.num_hands)
        self.declare_parameter(
            "min_detection_confidence", self.min_detection_confidence
        )
        self.declare_parameter("min_tracking_confidence", self.min_tracking_confidence)

        self.img_input_topic = (
            self.get_parameter("img_input_topic").get_parameter_value().string_value
        )
        self.landmarks_topic = (
            self.get_parameter("landmarks_topic").get_parameter_value().string_value
        )
        self.num_hands = (
            self.get_parameter("num_hands").get_parameter_value().integer_value
        )
        self.min_detection_confidence = (
            self.get_parameter("min_detection_confidence")
            .get_parameter_value()
            .double_value
        )
        self.min_tracking_confidence = (
            self.get_parameter("min_tracking_confidence")
            .get_parameter_value()
            .double_value
        )

    def img_input_callback(self, msg: Image) -> None:
        self.get_logger().debug("Received image")
        self.received_image_msg = msg
        

    def _process_image(self) -> None:
        if self.received_image_msg is None:
            self.get_logger().debug("No frame to process")
            return

        if self.received_image_msg == self.prev_image_msg:
            self.get_logger().debug("Received same frame")
            return

        img = self.bridge.imgmsg_to_cv2(self.received_image_msg, "rgb8")

        msg = helpers.serialize_gesture_recognizer_result(self.model.predict(img))

        if msg is None:
            return
        msg.header = self.received_image_msg.header
        self.get_logger().debug(f"Publishing landmarks, header: {msg.header}")
        self.landmarks_pub.publish(msg)

        self.prev_image_msg = self.received_image_msg

    def tick(self, blackboard: Optional[dict["str", Any]] = None) -> NodeState:
        try:
            self._process_image()
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
            return NodeState.FAILURE
        return NodeState.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDetectorNode("landmark_detector_node")
    rclpy.spin(node)
    rclpy.shutdown()
