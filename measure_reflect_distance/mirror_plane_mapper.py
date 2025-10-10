#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from measure_reflect_distance.util.mirror_geometry import quat_from_two_vectors


def _normalize(vec: np.ndarray) -> np.ndarray:
    """安全に正規化するユーティリティ（ゼロ除算を避ける）。"""
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        return vec
    return vec / norm


@dataclass
class PlaneCluster:
    """同一鏡面を表すクラスタの累積情報。"""
    cluster_id: int
    normal_sum: np.ndarray
    distance_sum: float
    count: int
    last_update_sec: float

    def averaged_normal(self) -> np.ndarray:
        return _normalize(self.normal_sum)

    def averaged_distance(self) -> float:
        return self.distance_sum / max(self.count, 1)

    def update(self, normal: np.ndarray, distance: float, stamp_sec: float) -> None:
        current_normal = self.averaged_normal()
        if float(current_normal @ normal) < 0.0:
            normal = -normal
            distance = -distance
        self.normal_sum = self.normal_sum + normal
        self.distance_sum += distance
        self.count += 1
        self.last_update_sec = stamp_sec


class MirrorPlaneMapper(Node):
    def __init__(self) -> None:
        super().__init__("mirror_plane_mapper")

        # --- パラメータ宣言（デフォルト値含む） ---
        self.declare_parameter("plane_topic", "mirror_plane")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "mirror_plane_map_markers")
        self.declare_parameter("angle_threshold_deg", 20.0)
        self.declare_parameter("distance_threshold", 0.5)
        self.declare_parameter("marker_scale", [0.6, 0.6, 0.02])
        self.declare_parameter("forget_after_sec", 0.0)  # 0 → 忘れない
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("display_timeout_sec", 0.0)  # 0 → 常に表示

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.angle_threshold = math.radians(
            self.get_parameter("angle_threshold_deg").get_parameter_value().double_value
        )
        self.distance_threshold = self.get_parameter("distance_threshold").get_parameter_value().double_value
        self.marker_scale = np.array(
            self.get_parameter("marker_scale").get_parameter_value().double_array_value or [0.6, 0.6, 0.02],
            dtype=float,
        )
        self.forget_after = self.get_parameter("forget_after_sec").get_parameter_value().double_value
        publish_period = self.get_parameter("publish_period_sec").get_parameter_value().double_value
        publish_period = max(publish_period, 0.1)
        self.display_timeout = float(self.get_parameter("display_timeout_sec").value)

        self._clusters: Dict[int, PlaneCluster] = {}
        self._next_cluster_id = 0
        self._deleted_ids = set()
        self._last_observation_sec: Optional[float] = None
        self._markers_active = False

        plane_topic = self.get_parameter("plane_topic").get_parameter_value().string_value
        # 鏡面推定ノードが発行する Float64[4]（法線+距離）を購読する
        self.create_subscription(Float64MultiArray, plane_topic, self._plane_callback, 10)
        # カメラ座標（frame_id パラメータ）で鏡面を MarkerArray として出力
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.timer = self.create_timer(publish_period, self._publish_markers)

        self.get_logger().info(
            f"[mirror_plane_mapper] listening on '{plane_topic}', publishing markers on '{self.marker_topic}' "
            f"in frame '{self.frame_id}'"
        )

    def _plane_callback(self, msg: Float64MultiArray) -> None:
        """鏡面（法線+距離）の単発観測を受け取りクラスタに振り分ける。"""
        if len(msg.data) != 4:
            self.get_logger().warn(f"Ignoring plane with invalid length: {len(msg.data)}")
            return

        normal = np.array(msg.data[:3], dtype=float)
        distance = float(msg.data[3])
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            self.get_logger().warn("Received plane with near-zero normal; ignoring")
            return
        normal = normal / norm
        stamp_sec = self.get_clock().now().nanoseconds / 1e9
        self._last_observation_sec = stamp_sec

        match = self._find_matching_cluster(normal, distance)
        if match is None:
            cluster_id = self._next_cluster_id
            self._next_cluster_id += 1

            self._clusters[cluster_id] = PlaneCluster(
                cluster_id=cluster_id,
                normal_sum=normal.copy(),
                distance_sum=distance,
                count=1,
                last_update_sec=stamp_sec,
            )
            self.get_logger().info(
                f"Created new mirror plane cluster id={cluster_id}, normal={normal}, distance={distance:.3f}"
            )
        else:
            cluster, aligned_normal, aligned_distance = match
            cluster.update(aligned_normal, aligned_distance, stamp_sec)

    def _find_matching_cluster(
        self, normal: np.ndarray, distance: float
    ) -> Optional[tuple]:
        """既存クラスタと角度・距離の両面で一致するかを判定する。"""
        for cluster in self._clusters.values():
            avg_normal = cluster.averaged_normal()
            cos_angle = float(avg_normal @ normal)
            aligned_normal = normal
            aligned_distance = distance
            if cos_angle < 0.0:
                cos_angle = -cos_angle
                aligned_normal = -normal
                aligned_distance = -distance

            cos_angle = max(min(cos_angle, 1.0), -1.0)
            angle = math.acos(cos_angle)
            if angle > self.angle_threshold:
                continue

            if abs(cluster.averaged_distance() - aligned_distance) > self.distance_threshold:
                continue

            return cluster, aligned_normal, aligned_distance
        return None

    def _publish_markers(self) -> None:
        """クラスタの平均結果を MarkerArray として配信する。"""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clock_now_sec = self.get_clock().now().nanoseconds / 1e9
        current_sec = clock_now_sec if self.display_timeout > 0.0 else None
        inactive = False
        if self.display_timeout > 0.0:
            if self._last_observation_sec is None:
                inactive = True
            else:
                inactive = (current_sec - self._last_observation_sec) > self.display_timeout

        expired_ids = []
        if self.forget_after > 0.0:
            for cid, cluster in self._clusters.items():
                if (clock_now_sec - cluster.last_update_sec) > self.forget_after:
                    expired_ids.append(cid)

        for cid in expired_ids:
            del self._clusters[cid]
            self._deleted_ids.add(cid)

        if inactive:
            if self._markers_active:
                for cluster in self._clusters.values():
                    marker = Marker()
                    marker.header.frame_id = self.frame_id
                    marker.header.stamp = now
                    marker.ns = "mirror_planes"
                    marker.id = cluster.cluster_id
                    marker.action = Marker.DELETE
                    markers.markers.append(marker)
                self._markers_active = False
            for deleted_id in list(self._deleted_ids):
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = now
                marker.ns = "mirror_planes"
                marker.id = deleted_id
                marker.action = Marker.DELETE
                markers.markers.append(marker)
                self._deleted_ids.remove(deleted_id)
            if markers.markers:
                self.marker_pub.publish(markers)
            return

        self._markers_active = True

        for cluster in self._clusters.values():
            normal = cluster.averaged_normal()
            distance = cluster.averaged_distance()
            position = -distance * normal
            orientation = quat_from_two_vectors(np.array([0.0, 0.0, 1.0]), normal)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "mirror_planes"
            marker.id = cluster.cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            marker.pose.orientation.x = float(orientation[0])
            marker.pose.orientation.y = float(orientation[1])
            marker.pose.orientation.z = float(orientation[2])
            marker.pose.orientation.w = float(orientation[3])
            marker.scale.x = float(self.marker_scale[0])
            marker.scale.y = float(self.marker_scale[1])
            marker.scale.z = float(max(self.marker_scale[2], 1e-3))
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.8
            markers.markers.append(marker)

        for deleted_id in list(self._deleted_ids):
            # 忘却済みクラスタは RViz 上からも削除する
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "mirror_planes"
            marker.id = deleted_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
            self._deleted_ids.remove(deleted_id)

        if markers.markers:
            self.marker_pub.publish(markers)


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
