#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image


class RGBRelayNode(Node):
    """
    Lightweight relay that republishes RGB images to a dedicated topic with optional throttling.
    This decouples heavy downstream processing (e.g., U-Net inference) from the main RGB stream.
    """

    def __init__(self) -> None:
        super().__init__("rgb_relay_node")

        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/camera/unet/image_raw")
        self.declare_parameter("max_fps", 10.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.max_fps = float(self.get_parameter("max_fps").get_parameter_value().double_value)

        if self.max_fps <= 0.0:
            self.min_interval = Duration(seconds=0.0)
        else:
            self.min_interval = Duration(seconds=1.0 / self.max_fps)
        self._last_pub_time: Time | None = None

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.publisher_ = self.create_publisher(Image, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f"RGBRelayNode relaying {input_topic} -> {output_topic} (max_fps={self.max_fps:.2f})"
        )

    def _image_callback(self, msg: Image) -> None:
        now = self.get_clock().now()
        if self._last_pub_time is not None and self.min_interval.nanoseconds > 0:
            if (now - self._last_pub_time) < self.min_interval:
                return
        self._last_pub_time = now
        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RGBRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
