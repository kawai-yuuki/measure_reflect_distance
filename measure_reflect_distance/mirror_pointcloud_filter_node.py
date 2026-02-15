#!/usr/bin/env python3

from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class MirrorPointCloudFilterNode(Node):
    """Strip non-XYZ fields from a mirror point cloud for Nav2 costmap inputs."""

    def __init__(self) -> None:
        super().__init__("mirror_pointcloud_filter_node")

        self.declare_parameter("input_topic", "mirror_surface_projected_points")
        self.declare_parameter("output_topic", "mirror_surface_projected_points_xyz")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("use_current_stamp", True)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._frame_id_override = (
            self.get_parameter("frame_id_override").get_parameter_value().string_value
        )
        self._use_current_stamp = bool(self.get_parameter("use_current_stamp").value)

        self._pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(
            PointCloud2,
            input_topic,
            self._callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "MirrorPointCloudFilterNode ready "
            f"(input={input_topic}, output={output_topic}, use_current_stamp={self._use_current_stamp})"
        )

    def _callback(self, msg: PointCloud2) -> None:
        points_iter: Iterable[Tuple[float, float, float]] = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = [(float(x), float(y), float(z)) for x, y, z in points_iter]
        header = Header()
        header.stamp = self.get_clock().now().to_msg() if self._use_current_stamp else msg.header.stamp
        header.frame_id = self._frame_id_override or msg.header.frame_id

        cloud = pc2.create_cloud_xyz32(header, points)
        self._pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MirrorPointCloudFilterNode()
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
