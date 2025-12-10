#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


@dataclass
class ClusterGrid:
    origin: np.ndarray
    tangent: np.ndarray
    binormal: np.ndarray
    normal: np.ndarray
    resolution: float
    counts: Dict[Tuple[int, int], float] = field(default_factory=dict)
    min_i: int = 0
    max_i: int = -1
    min_j: int = 0
    max_j: int = -1
    total_points: float = 0.0
    last_update: float = 0.0
    polygon_cache: Optional[List[np.ndarray]] = None

    def integrate_points(self, points: np.ndarray) -> None:
        if points.size == 0:
            return
        rel = points - self.origin[None, :]
        u = rel @ self.tangent
        v = rel @ self.binormal
        res_inv = 1.0 / self.resolution
        i_indices = np.floor(u * res_inv).astype(int)
        j_indices = np.floor(v * res_inv).astype(int)
        for i, j in zip(i_indices, j_indices):
            key = (i, j)
            self.counts[key] = self.counts.get(key, 0.0) + 1.0
            if self.total_points == 0.0:
                self.min_i = self.max_i = i
                self.min_j = self.max_j = j
            else:
                self.min_i = min(self.min_i, i)
                self.max_i = max(self.max_i, i)
                self.min_j = min(self.min_j, j)
                self.max_j = max(self.max_j, j)
            self.total_points += 1.0
        self.polygon_cache = None

    def as_density_map(self) -> Optional[np.ndarray]:
        if not self.counts:
            return None
        width = self.max_i - self.min_i + 1
        height = self.max_j - self.min_j + 1
        grid = np.zeros((height, width), dtype=np.float32)
        for (i, j), value in self.counts.items():
            grid[j - self.min_j, i - self.min_i] = float(value)
        if grid.max() > 0:
            grid /= grid.max()
        return grid

    def grid_to_world(self, i: int, j: int) -> np.ndarray:
        return (
            self.origin
            + self.tangent * ((i + 0.5) * self.resolution)
            + self.binormal * ((j + 0.5) * self.resolution)
        )


class MirrorSurfaceProjectionAggregatorNode(Node):
    """Accumulate projected points per cluster and rebuild polygons for visualization."""

    def __init__(self) -> None:
        super().__init__("mirror_surface_projection_aggregator_node")

        self.declare_parameter("projected_points_topic", "mirror_surface_projected_points")
        self.declare_parameter("cluster_plane_topic", "mirror_plane_clustered")
        self.declare_parameter("marker_topic", "mirror_surface_projected_markers")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("grid_resolution", 0.01)
        self.declare_parameter("grid_update_period_sec", 1.0)
        self.declare_parameter("grid_density_threshold", 0.2)
        self.declare_parameter("min_points", 10)
        self.declare_parameter("marker_line_width", 0.01)

        points_topic = self.get_parameter("projected_points_topic").get_parameter_value().string_value
        self._cluster_plane_topic = (
            self.get_parameter("cluster_plane_topic").get_parameter_value().string_value
        )
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self._grid_resolution = max(
            1e-4, self.get_parameter("grid_resolution").get_parameter_value().double_value
        )
        self._grid_update_period = max(
            0.1, self.get_parameter("grid_update_period_sec").get_parameter_value().double_value
        )
        self._grid_density_threshold = min(
            1.0,
            max(0.0, self.get_parameter("grid_density_threshold").get_parameter_value().double_value),
        )
        self._min_points = max(1, int(self.get_parameter("min_points").get_parameter_value().integer_value))
        self.marker_line_width = max(
            0.001, self.get_parameter("marker_line_width").get_parameter_value().double_value
        )

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(PointCloud2, points_topic, self._points_callback, 10)
        self.create_subscription(
            Float64MultiArray,
            self._cluster_plane_topic,
            self._cluster_plane_callback,
            10,
        )
        self._grid_timer = self.create_timer(self._grid_update_period, self._process_grids)

        self._cluster_grids: Dict[int, ClusterGrid] = {}
        self._last_publish_had_data = False

        self.get_logger().info(
            "MirrorSurfaceProjectionAggregatorNode ready "
            f"(points_topic={points_topic}, marker_topic={marker_topic}, cluster_plane_topic={self._cluster_plane_topic})"
        )

    def _cluster_plane_callback(self, msg: Float64MultiArray) -> None:
        data = msg.data
        if not data:
            return
        cluster_id = int(round(data[0]))
        normal = np.array(data[1:4], dtype=np.float64)
        distance = float(data[4]) if len(data) >= 5 else 0.0
        point = np.array(data[5:8], dtype=np.float64) if len(data) >= 8 else normal * (-distance)
        if len(data) >= 11:
            tangent = np.array(data[8:11], dtype=np.float64)
        else:
            tangent = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        norm_n = np.linalg.norm(normal)
        if norm_n > 1e-6:
            normal = normal / norm_n
            distance = distance / norm_n
        tangent, binormal = self._build_basis(normal, tangent)
        self._update_grid(cluster_id, point, tangent, binormal, normal)

    def _build_basis(self, normal: np.ndarray, tangent_hint: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        normal = np.asarray(normal, dtype=np.float64)
        tangent_hint = np.asarray(tangent_hint, dtype=np.float64)
        tangent = tangent_hint - float(tangent_hint @ normal) * normal
        norm_t = np.linalg.norm(tangent)
        if norm_t < 1e-6:
            tangent = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            tangent = tangent - float(tangent @ normal) * normal
            norm_t = np.linalg.norm(tangent)
        if norm_t > 1e-6:
            tangent = tangent / norm_t
        binormal = np.cross(normal, tangent)
        norm_b = np.linalg.norm(binormal)
        if norm_b < 1e-6:
            ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            binormal = np.cross(normal, ref)
            norm_b = np.linalg.norm(binormal)
        if norm_b > 1e-6:
            binormal = binormal / norm_b
        return tangent, binormal

    def _update_grid(
        self, cluster_id: int, point: np.ndarray, tangent: np.ndarray, binormal: np.ndarray, normal: np.ndarray
    ) -> None:
        grid = self._cluster_grids.get(cluster_id)
        if grid is None:
            grid = ClusterGrid(
                origin=point.copy(),
                tangent=tangent.copy(),
                binormal=binormal.copy(),
                normal=normal.copy(),
                resolution=self._grid_resolution,
            )
        else:
            grid.origin = point.copy()
            grid.tangent = tangent.copy()
            grid.binormal = binormal.copy()
            grid.normal = normal.copy()
            grid.resolution = self._grid_resolution
        self._cluster_grids[cluster_id] = grid

    def _points_callback(self, msg: PointCloud2) -> None:
        field_names = {f.name for f in msg.fields}
        required_fields = {"x", "y", "z", "cluster_id"}
        if not required_fields.issubset(field_names):
            self.get_logger().warn("Projected points message missing required fields; skipping.")
            return

        clusters: Dict[int, List[Tuple[float, float, float]]] = {}
        for x, y, z, cid in pc2.read_points(
            msg, field_names=("x", "y", "z", "cluster_id"), skip_nans=True
        ):
            cluster_id = int(round(cid))
            if cluster_id < 0:
                continue
            clusters.setdefault(cluster_id, []).append((x, y, z))

        if not clusters:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        for cluster_id, pts in clusters.items():
            grid = self._cluster_grids.get(cluster_id)
            if grid is None:
                self.get_logger().debug(
                    f"No grid initialized for cluster_id={cluster_id}; skipping accumulation."
                )
                continue
            pts_arr = np.asarray(pts, dtype=np.float64)
            grid.integrate_points(pts_arr)
            grid.last_update = now_sec

    def _process_grids(self) -> None:
        marker_array = MarkerArray()
        marker_id = 0
        for cluster_id, grid in list(self._cluster_grids.items()):
            if grid.total_points < self._min_points:
                continue
            density = grid.as_density_map()
            if density is None or density.size == 0:
                continue
            threshold = max(self._grid_density_threshold, 1e-3)
            _, binary = cv2.threshold(density, threshold, 1.0, cv2.THRESH_BINARY)
            binary_uint8 = (binary * 255).astype(np.uint8)
            contours, _ = cv2.findContours(binary_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if contour.shape[0] < 3:
                    continue
                world_points: List[np.ndarray] = []
                for pt in contour:
                    ci = int(pt[0][0]) + grid.min_i
                    cj = int(pt[0][1]) + grid.min_j
                    world_points.append(grid.grid_to_world(ci, cj))
                if len(world_points) < 3:
                    continue
                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "mirror_surface_projection_aggregated"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = self.marker_line_width
                marker.color.r = 1.0
                marker.color.g = 0.6
                marker.color.b = 0.0
                marker.color.a = 0.9
                for p in world_points:
                    marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                # close loop
                first = world_points[0]
                marker.points.append(Point(x=float(first[0]), y=float(first[1]), z=float(first[2])))
                marker_array.markers.append(marker)
                marker_id += 1

        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self._last_publish_had_data = True
        elif self._last_publish_had_data:
            delete_all = Marker()
            delete_all.header.frame_id = self.target_frame
            delete_all.header.stamp = self.get_clock().now().to_msg()
            delete_all.action = Marker.DELETEALL
            marker_array.markers.append(delete_all)
            self.marker_pub.publish(marker_array)
            self._last_publish_had_data = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MirrorSurfaceProjectionAggregatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
