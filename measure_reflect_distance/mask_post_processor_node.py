#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image


class MaskPostProcessingNode(Node):
    def __init__(self) -> None:
        super().__init__("mask_post_processing_node")

        # Parameters let us reuse this node with different topics without code changes.
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("mask_topic", "/mask_image")
        self.declare_parameter("output_topic", "/mask_image_processed")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)  # seconds

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        queue_size = (
            self.get_parameter("sync_queue_size")
            .get_parameter_value()
            .integer_value
        )
        slop = (
            self.get_parameter("sync_slop").get_parameter_value().double_value
        )

        if queue_size <= 0:
            self.get_logger().warn(
                "sync_queue_size must be positive. Falling back to default value 10."
            )
            queue_size = 10

        if slop <= 0.0:
            self.get_logger().warn(
                "sync_slop must be positive. Falling back to default value 0.05."
            )
            slop = 0.05

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, output_topic, 10)

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.mask_sub = Subscriber(self, Image, mask_topic)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.mask_sub],
            queue_size=queue_size,
            slop=slop,
        )
        self.synchronizer.registerCallback(self.synced_callback)

        self.get_logger().info(
            f"MaskPostProcessingNode started. rgb_topic={rgb_topic}, "
            f"mask_topic={mask_topic}, output_topic={output_topic}"
        )

    def synced_callback(self, rgb_msg: Image, mask_msg: Image) -> None:
        try:
            mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
        except CvBridgeError as err:
            self.get_logger().error(f"Failed to convert mask image: {err}")
            return

        processed_mask = self.process_mask(mask_image, rgb_msg.height, rgb_msg.width)

        try:
            output_msg = self.bridge.cv2_to_imgmsg(processed_mask, encoding="mono8")
        except CvBridgeError as err:
            self.get_logger().error(f"Failed to convert processed mask image: {err}")
            return

        output_msg.header = rgb_msg.header
        self.publisher_.publish(output_msg)

    def process_mask(
        self, mask: np.ndarray, target_height: int, target_width: int
    ) -> np.ndarray:
        """Apply simple cleanup so that downstream consumers receive consistent masks."""
        resized = mask
        if mask.shape[0] != target_height or mask.shape[1] != target_width:
            resized = cv2.resize(mask, (target_width, target_height), interpolation=cv2.INTER_NEAREST)

        # The mask from the network should already be binary, but we re-binarize it
        # to guard against interpolation artifacts or scaling to 255.
        _, binary = cv2.threshold(resized, 127, 255, cv2.THRESH_BINARY)
        return binary


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MaskPostProcessingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
