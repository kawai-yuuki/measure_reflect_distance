#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, Set

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from measure_reflect_distance.util.mirror_geometry import quat_from_R


def _safe_normalize(vec: np.ndarray) -> np.ndarray:
    """ゼロ割りを避けつつ正規化する。"""
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        return vec
    return vec / norm


def _build_basis(normal: np.ndarray, tangent_hint: np.ndarray) -> np.ndarray:
    """法線と接線ヒントから右手直交基底 [tangent, binormal, normal] を構成する。"""
    n = _safe_normalize(normal)

    # 接線ヒントを平面内へ射影
    t = tangent_hint - float(tangent_hint @ n) * n
    if np.linalg.norm(t) < 1e-6:
        ref = np.array([1.0, 0.0, 0.0])
        if abs(float(ref @ n)) > 0.9:
            ref = np.array([0.0, 1.0, 0.0])
        t = ref - float(ref @ n) * n
    t = _safe_normalize(t)

    # binormal = n × t
    b = np.cross(n, t)
    if np.linalg.norm(b) < 1e-6:
        ref = np.array([0.0, 0.0, 1.0])
        b = np.cross(n, ref)
        if np.linalg.norm(b) < 1e-6:
            b = np.array([1.0, 0.0, 0.0])
    b = _safe_normalize(b)

    # 接線を再計算（数値誤差を抑える）
    t = np.cross(b, n)
    t = _safe_normalize(t)

    return np.column_stack((t, b, n))


@dataclass
class PlaneObservation:
    marker_id: int
    normal: np.ndarray
    distance: float
    point: np.ndarray
    tangent: np.ndarray
    stamp_sec: float


class MirrorPlaneMapper(Node):
    """
    鏡面推定の生データをそのまま Marker として可視化するノード。
    クラスタリングを行わず、各観測ごとに個別のマーカーを生成し、削除もしない。
    """

    def __init__(self) -> None:
        super().__init__("mirror_plane_mapper")

        self.declare_parameter("plane_topic", "mirror_plane")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "mirror_plane_map_markers")
        self.declare_parameter("marker_scale", [0.6, 0.6, 0.02])
        self.declare_parameter("publish_period_sec", 0.5)

        plane_topic = self.get_parameter("plane_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.marker_scale = np.array(
            self.get_parameter("marker_scale").get_parameter_value().double_array_value or [0.6, 0.6, 0.02],
            dtype=float,
        )
        publish_period = max(self.get_parameter("publish_period_sec").get_parameter_value().double_value, 0.1)

        self._observations: Dict[int, PlaneObservation] = {}
        self._active_marker_ids: Set[int] = set()
        self._next_marker_id = 0

        self.create_subscription(Float64MultiArray, plane_topic, self._plane_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.timer = self.create_timer(publish_period, self._publish_markers)

        self.get_logger().info(
            f"[mirror_plane_mapper] visualizing raw observations from '{plane_topic}' on '{self.marker_topic}' "
            f"in frame '{self.frame_id}'"
        )

    def _plane_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) not in (4, 7, 10):
            self.get_logger().warn(f"Ignoring plane with invalid length: {len(msg.data)}")
            return

        normal = np.array(msg.data[:3], dtype=float)
        distance = float(msg.data[3])

        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            self.get_logger().warn("Received plane with near-zero normal; ignoring")
            return
        normal = normal / norm

        if len(msg.data) >= 7:
            point = np.array(msg.data[4:7], dtype=float)
        else:
            point = -distance * normal

        if len(msg.data) >= 10:
            tangent = np.array(msg.data[7:10], dtype=float)
        else:
            ref = np.array([1.0, 0.0, 0.0])
            if abs(float(ref @ normal)) > 0.9:
                ref = np.array([0.0, 1.0, 0.0])
            tangent = ref - float(ref @ normal) * normal
        tangent = _safe_normalize(tangent)

        marker_id = self._next_marker_id
        self._next_marker_id += 1

        self._observations[marker_id] = PlaneObservation(
            marker_id=marker_id,
            normal=normal,
            distance=distance,
            point=point,
            tangent=tangent,
            stamp_sec=self.get_clock().now().nanoseconds / 1e9,
        )

    def _publish_markers(self) -> None:
        if not self.marker_pub:
            return

        now_msg = self.get_clock().now().to_msg()

        markers = MarkerArray()
        current_ids: Set[int] = set()

        for obs in self._observations.values():
            basis = _build_basis(obs.normal, obs.tangent)
            q = quat_from_R(basis)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now_msg
            marker.ns = "mirror_planes"
            marker.id = obs.marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(obs.point[0])
            marker.pose.position.y = float(obs.point[1])
            marker.pose.position.z = float(obs.point[2])
            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])
            marker.scale.x = float(self.marker_scale[0])
            marker.scale.y = float(self.marker_scale[1])
            marker.scale.z = float(max(self.marker_scale[2], 1e-3))
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.7

            markers.markers.append(marker)
            current_ids.add(obs.marker_id)

        # 古いマーカーを削除
        for marker_id in list(self._active_marker_ids - current_ids):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now_msg
            marker.ns = "mirror_planes"
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
            self._active_marker_ids.remove(marker_id)

        if markers.markers:
            self.marker_pub.publish(markers)
            self._active_marker_ids.update(current_ids)


def main(args=None):
    rclpy.init(args=args)
    node = MirrorPlaneMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
