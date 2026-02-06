#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
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
        self.declare_parameter("binarize_threshold", 127)
        self.declare_parameter("binarize_max_value", 255)
        self.declare_parameter("morph_operation", "close")
        self.declare_parameter("morph_kernel_shape", "rect")
        self.declare_parameter("morph_kernel_width", 3)
        self.declare_parameter("morph_kernel_height", 3)
        self.declare_parameter("morph_iterations", 1)

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        queue_size = self._get_int_param(
            "sync_queue_size", default=10, min_value=1
        )
        slop = self._get_float_param("sync_slop", default=0.05, min_value=0.0)
        self.binarize_threshold = self._get_int_param(
            "binarize_threshold", default=127, min_value=0, max_value=255
        )
        self.binarize_max_value = self._get_int_param(
            "binarize_max_value", default=255, min_value=1, max_value=255
        )
        self.morph_operation = self._get_str_param(
            "morph_operation",
            default="close",
            allowed_values=("none", "close", "open", "erode", "dilate"),
        )
        self.morph_kernel_shape = self._get_str_param(
            "morph_kernel_shape",
            default="rect",
            allowed_values=("rect", "ellipse", "cross"),
        )
        self.morph_kernel_width = self._get_int_param(
            "morph_kernel_width", default=3, min_value=1
        )
        self.morph_kernel_height = self._get_int_param(
            "morph_kernel_height", default=3, min_value=1
        )
        self.morph_iterations = self._get_int_param(
            "morph_iterations", default=1, min_value=0
        )

        self.bridge = CvBridge()
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher_ = self.create_publisher(Image, output_topic, reliable_qos)

        self.rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=qos_profile_sensor_data)
        self.mask_sub = Subscriber(self, Image, mask_topic, qos_profile=qos_profile_sensor_data)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.mask_sub],
            queue_size=queue_size,
            slop=slop,
        )
        self.synchronizer.registerCallback(self.synced_callback)

        self.get_logger().info(
            "MaskPostProcessingNode started. "
            f"rgb_topic={rgb_topic}, mask_topic={mask_topic}, "
            f"output_topic={output_topic}, "
            f"sync_queue_size={queue_size}, sync_slop={slop:.3f}, "
            f"binarize_threshold={self.binarize_threshold}, "
            f"binarize_max_value={self.binarize_max_value}, "
            f"morph_operation={self.morph_operation}, "
            f"morph_kernel_shape={self.morph_kernel_shape}, "
            f"morph_kernel=({self.morph_kernel_width}x{self.morph_kernel_height}), "
            f"morph_iterations={self.morph_iterations}"
        )

    def _get_int_param(
        self,
        name: str,
        default: int,
        min_value: int | None = None,
        max_value: int | None = None,
    ) -> int:
        value = self.get_parameter(name).get_parameter_value().integer_value
        if min_value is not None and value < min_value:
            self.get_logger().warn(
                f"{name} must be >= {min_value}. Falling back to {default}."
            )
            return default
        if max_value is not None and value > max_value:
            self.get_logger().warn(
                f"{name} must be <= {max_value}. Falling back to {default}."
            )
            return default
        return value

    def _get_float_param(
        self, name: str, default: float, min_value: float | None = None
    ) -> float:
        value = self.get_parameter(name).get_parameter_value().double_value
        if min_value is not None and value <= min_value:
            self.get_logger().warn(
                f"{name} must be > {min_value}. Falling back to {default}."
            )
            return default
        return value

    def _get_str_param(
        self, name: str, default: str, allowed_values: tuple[str, ...]
    ) -> str:
        value = self.get_parameter(name).get_parameter_value().string_value
        if value not in allowed_values:
            allowed = ", ".join(allowed_values)
            self.get_logger().warn(
                f"{name} must be one of [{allowed}]. Falling back to {default}."
            )
            return default
        return value

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
        _, binary = cv2.threshold(
            resized,
            self.binarize_threshold,
            self.binarize_max_value,
            cv2.THRESH_BINARY,
        )

        if self.morph_iterations <= 0 or self.morph_operation == "none":
            return binary

        kernel_shape_map = {
            "rect": cv2.MORPH_RECT,
            "ellipse": cv2.MORPH_ELLIPSE,
            "cross": cv2.MORPH_CROSS,
        }
        kernel_shape = kernel_shape_map.get(self.morph_kernel_shape, cv2.MORPH_RECT)
        kernel = cv2.getStructuringElement(
            kernel_shape, (self.morph_kernel_width, self.morph_kernel_height)
        )

        if self.morph_operation == "open":
            return cv2.morphologyEx(
                binary, cv2.MORPH_OPEN, kernel, iterations=self.morph_iterations
            )
        if self.morph_operation == "erode":
            return cv2.erode(binary, kernel, iterations=self.morph_iterations)
        if self.morph_operation == "dilate":
            return cv2.dilate(binary, kernel, iterations=self.morph_iterations)

        # Default: closing fills small holes / smooth jagged edges.
        return cv2.morphologyEx(
            binary, cv2.MORPH_CLOSE, kernel, iterations=self.morph_iterations
        )


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
