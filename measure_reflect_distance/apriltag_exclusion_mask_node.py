#!/usr/bin/env python3

from __future__ import annotations

from typing import Any

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_EXCLUDE_TOP_RATIO = 1.0 / 6.0


class ApriltagExclusionMaskNode(Node):
    """Mask out the top region of images before AprilTag detection."""

    def __init__(self) -> None:
        super().__init__("apriltag_exclusion_mask_node")

        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "output_topic", "/camera/camera/color/image_apriltag_masked"
        )
        self.declare_parameter("exclude_top_ratio", DEFAULT_EXCLUDE_TOP_RATIO)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.exclude_top_ratio = self._get_valid_ratio(
            "exclude_top_ratio", DEFAULT_EXCLUDE_TOP_RATIO
        )

        self._bridge = CvBridge()

        self._sub = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(Image, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            "ApriltagExclusionMaskNode started. "
            f"input_topic={input_topic}, output_topic={output_topic}, "
            f"exclude_top_ratio={self.exclude_top_ratio:.6f}"
        )

    def _get_valid_ratio(self, name: str, default: float) -> float:
        raw_value: Any = self.get_parameter(name).value
        try:
            ratio = float(raw_value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"{name} must be a float in [0.0, 1.0). Falling back to {default:.6f}."
            )
            return default

        if not 0.0 <= ratio < 1.0:
            self.get_logger().warn(
                f"{name} must be in [0.0, 1.0). Falling back to {default:.6f}."
            )
            return default
        return ratio

    def _image_callback(self, msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            masked = image.copy()

            if masked.ndim < 2 or masked.shape[0] <= 0:
                self.get_logger().warn("Received malformed image. Forwarding as-is.")
                self._pub.publish(msg)
                return

            excluded_rows = int(masked.shape[0] * self.exclude_top_ratio)
            if excluded_rows > 0:
                masked[:excluded_rows, ...] = 0

            output = self._bridge.cv2_to_imgmsg(
                masked, encoding=msg.encoding if msg.encoding else "passthrough"
            )
            output.header = msg.header
            self._pub.publish(output)
        except CvBridgeError as err:
            self.get_logger().error(
                f"Failed to convert image for Apriltag masking: {err}. Forwarding input image."
            )
            self._pub.publish(msg)
        except Exception as err:
            self.get_logger().error(
                f"Unexpected masking error: {err}. Forwarding input image."
            )
            self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ApriltagExclusionMaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
