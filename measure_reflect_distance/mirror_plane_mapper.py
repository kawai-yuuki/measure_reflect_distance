#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional, Set

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


def _sym_point_plane_distance(point_a: np.ndarray, normal_a: np.ndarray,
                              point_b: np.ndarray, normal_b: np.ndarray) -> float:
    """DP-planes で使用される対称点-平面距離。"""
    diff = point_a - point_b
    term_ab = abs(float(diff @ normal_b))
    term_ba = abs(float(-diff @ normal_a))
    return 0.5 * (term_ab + term_ba)


@dataclass
class PlaneCluster:
    """DP-planes クラスタ中心の統計量。"""

    cluster_id: int
    point_sum: np.ndarray
    normal_sum: np.ndarray
    tangent_sum: np.ndarray
    count: int
    last_update: float

    def representative_point(self) -> np.ndarray:
        return self.point_sum / max(1, self.count)

    def representative_normal(self) -> np.ndarray:
        return _safe_normalize(self.normal_sum)

    def representative_tangent(self) -> np.ndarray:
        tangent = self.tangent_sum / max(1, self.count)
        normal = self.representative_normal()
        tangent = tangent - float(tangent @ normal) * normal
        tangent = _safe_normalize(tangent)
        if np.linalg.norm(tangent) < 1e-6:
            # 直交する任意軸を再構成
            ref = np.array([1.0, 0.0, 0.0])
            if abs(float(ref @ normal)) > 0.9:
                ref = np.array([0.0, 1.0, 0.0])
            tangent = ref - float(ref @ normal) * normal
            tangent = _safe_normalize(tangent)
        return tangent

    def plane_distance(self) -> float:
        n = self.representative_normal()
        p = self.representative_point()
        return -float(n @ p)

    def merge(self, point: np.ndarray, normal: np.ndarray, tangent: np.ndarray, stamp_sec: float) -> None:
        # 法線をクラスタの向きに合わせる（符号違いは同一平面）
        cluster_normal = self.representative_normal()
        if float(cluster_normal @ normal) < 0.0:
            normal = -normal
            tangent = -tangent

        tangent = tangent - float(tangent @ normal) * normal
        tangent = _safe_normalize(tangent)

        self.point_sum += point
        self.normal_sum += normal
        self.tangent_sum += tangent
        self.count += 1
        self.last_update = stamp_sec


class MirrorPlaneMapper(Node):
    """
    鏡面推定の観測を DP-planes に基づくクラスタリングで統合し、鏡面ごとに代表マーカーを出力するノード。
    """

    def __init__(self) -> None:
        super().__init__("mirror_plane_mapper")

        # --- パラメータ ---
        self.declare_parameter("plane_topic", "mirror_plane")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "mirror_plane_map_markers")
        self.declare_parameter("marker_scale", [0.1, 0.1, 0.01])
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("dp_lambda", 0.10)  # [m] DP-planes のクラスタ生成閾値
        self.declare_parameter("cluster_stale_time_sec", 0.0)
        self.declare_parameter("cluster_min_support", 3)

        plane_topic = self.get_parameter("plane_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.marker_scale = np.array(
            self.get_parameter("marker_scale").get_parameter_value().double_array_value or [0.6, 0.6, 0.02],
            dtype=float,
        )
        publish_period = max(self.get_parameter("publish_period_sec").get_parameter_value().double_value, 0.1)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.dp_lambda = max(0.0, float(self.get_parameter("dp_lambda").value))
        self.cluster_stale_time = max(0.0, float(self.get_parameter("cluster_stale_time_sec").value))
        self.cluster_min_support = max(1, int(self.get_parameter("cluster_min_support").value))

        # --- 状態 ---
        self._clusters: Dict[int, PlaneCluster] = {}
        self._active_marker_ids: Set[int] = set()
        self._next_cluster_id = 0

        # --- ROS I/O ---
        self.create_subscription(Float64MultiArray, plane_topic, self._plane_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        if self.publish_markers:
            self.timer = self.create_timer(publish_period, self._publish_markers)
            self.get_logger().info(
                "[mirror_plane_mapper] DP-planes clustering enabled: "
                f"lambda={self.dp_lambda:.3f} m, min_support={self.cluster_min_support}"
            )
        else:
            self.timer = None
            self._publish_marker_delete_all()
            self.get_logger().info(
                "[mirror_plane_mapper] 'publish_markers' is false; marker output disabled and previous markers cleared"
            )

    # --- 観測ハンドリング ---

    def _plane_callback(self, msg: Float64MultiArray) -> None:
        if not self.publish_markers:
            return
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

        stamp_sec = self.get_clock().now().nanoseconds / 1e9
        self._assign_observation(point, normal, tangent, stamp_sec)

    def _assign_observation(self, point: np.ndarray, normal: np.ndarray,
                            tangent: np.ndarray, stamp_sec: float) -> None:
        """DP-planes に基づき観測を既存クラスタへ割り当て。閾値超過なら新規クラスタを生成。"""
        best_cluster: Optional[PlaneCluster] = None
        best_distance = math.inf

        for cluster in self._clusters.values():
            cluster_point = cluster.representative_point()
            cluster_normal = cluster.representative_normal()
            distance = _sym_point_plane_distance(point, normal, cluster_point, cluster_normal)
            if distance < best_distance:
                best_distance = distance
                best_cluster = cluster

        if best_cluster is None or best_distance > self.dp_lambda:
            self._create_cluster(point, normal, tangent, stamp_sec)
            return

        best_cluster.merge(point, normal, tangent, stamp_sec)

    def _create_cluster(self, point: np.ndarray, normal: np.ndarray,
                        tangent: np.ndarray, stamp_sec: float) -> None:
        cluster_id = self._next_cluster_id
        self._next_cluster_id += 1

        tangent = tangent - float(tangent @ normal) * normal
        tangent = _safe_normalize(tangent)

        self._clusters[cluster_id] = PlaneCluster(
            cluster_id=cluster_id,
            point_sum=point.copy(),
            normal_sum=normal.copy(),
            tangent_sum=tangent.copy(),
            count=1,
            last_update=stamp_sec,
        )
        self.get_logger().info(f"[mirror_plane_mapper] new cluster #{cluster_id} created (count=1)")

    # --- Marker 出力 ---

    def _publish_markers(self) -> None:
        if not self.publish_markers or not self.marker_pub:
            return

        now_ros_time = self.get_clock().now()
        now_msg = now_ros_time.to_msg()
        now_sec = now_ros_time.nanoseconds / 1e9

        self._expire_stale_clusters(now_sec)

        markers = MarkerArray()
        current_ids: Set[int] = set()

        for cluster in self._clusters.values():
            if cluster.count < self.cluster_min_support:
                continue

            normal = cluster.representative_normal()
            tangent = cluster.representative_tangent()
            point = cluster.representative_point()
            basis = _build_basis(normal, tangent)
            q = quat_from_R(basis)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now_msg
            marker.ns = "mirror_planes"
            marker.id = cluster.cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
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
            current_ids.add(cluster.cluster_id)

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
        else:
            self._publish_marker_delete_all()

    def _expire_stale_clusters(self, now_sec: float) -> None:
        if self.cluster_stale_time <= 0.0:
            return
        stale_ids = [
            cluster_id
            for cluster_id, cluster in self._clusters.items()
            if (now_sec - cluster.last_update) > self.cluster_stale_time
        ]
        for cluster_id in stale_ids:
            del self._clusters[cluster_id]
            if cluster_id in self._active_marker_ids:
                self._active_marker_ids.remove(cluster_id)
            self.get_logger().info(f"[mirror_plane_mapper] cluster #{cluster_id} expired")

    def _publish_marker_delete_all(self) -> None:
        if not self.marker_pub:
            return
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mirror_planes"
        marker.id = 0
        marker.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)
        self._active_marker_ids.clear()


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
