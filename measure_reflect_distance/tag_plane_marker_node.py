#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from measure_reflect_distance.util.mirror_geometry import mat4_from_tf


@dataclass
class PlaneState:
    """Holds the smoothed 3D quadrilateral for a single tag ID."""

    corners: np.ndarray  # shape = (4, 3)
    last_update_ns: int

    def age(self, now_ns: int) -> float:
        return (now_ns - self.last_update_ns) * 1e-9


def to_point(vec3: np.ndarray) -> Point:
    pt = Point()
    pt.x = float(vec3[0])
    pt.y = float(vec3[1])
    pt.z = float(vec3[2])
    return pt


def make_color(r: float, g: float, b: float, a: float = 0.6) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


class TagPlaneMarkerNode(Node):
    """
    Aggregate AprilTag quadrilaterals over time and publish them as RViz markers.

    The node requests TF for each detected tag, builds the 3D corners using the known tag size,
    smooths the result with an exponential moving average, and publishes one filled marker per ID.
    """

    COLOR_TABLE = [
        make_color(0.90, 0.30, 0.25),
        make_color(0.20, 0.60, 0.85),
        make_color(0.95, 0.70, 0.20),
        make_color(0.35, 0.80, 0.40),
        make_color(0.60, 0.45, 0.80),
        make_color(0.20, 0.65, 0.55),
    ]

    def __init__(self) -> None:
        super().__init__('tag_plane_marker_node')

        # ---- Parameters ----
        self.declare_parameter('marker_topic', '/tag_plane_markers')
        self.declare_parameter('target_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('output_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame_template', 'landmark_{id}')
        self.declare_parameter('tag_frame_ids', [])
        self.declare_parameter('tag_frame_names', [])
        self.declare_parameter('default_tag_size', 0.08)
        self.declare_parameter('tag_size_ids', [])
        self.declare_parameter('tag_size_values', [])
        self.declare_parameter('smoothing_alpha', 0.2)
        self.declare_parameter('state_timeout_sec', 0.0)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('marker_lifetime_sec', 0.2)
        self.declare_parameter('publish_individual_markers', False)
        self.declare_parameter('marker_namespace', 'tag_planes')
        self.declare_parameter('combined_marker_id', 0)
        self.declare_parameter('combined_marker_color', [1.0, 0.9, 0.1, 0.75])
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('tf_stale_threshold_sec', 0.5)

        self.marker_topic: str = self.get_parameter('marker_topic').value
        self.target_ids: List[int] = [int(v) for v in self.get_parameter('target_ids').value]
        self.output_frame: str = self.get_parameter('output_frame').value
        self.frame_template: str = self.get_parameter('tag_frame_template').value

        self.tag_frame_map = self._build_lookup(
            self.get_parameter('tag_frame_ids').value,
            self.get_parameter('tag_frame_names').value,
            "frame",
        )

        self.default_tag_size: float = float(self.get_parameter('default_tag_size').value)
        self.tag_size_map = self._build_lookup(
            self.get_parameter('tag_size_ids').value,
            self.get_parameter('tag_size_values').value,
            "size",
        )

        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.alpha = min(max(self.alpha, 0.01), 1.0)
        self.state_timeout = max(0.0, float(self.get_parameter('state_timeout_sec').value))
        self.marker_lifetime = max(0.0, float(self.get_parameter('marker_lifetime_sec').value))
        self.publish_individual = bool(self.get_parameter('publish_individual_markers').value)
        self.marker_ns = str(self.get_parameter('marker_namespace').value)
        self.combined_marker_id = int(self.get_parameter('combined_marker_id').value)
        combined_color_param = list(self.get_parameter('combined_marker_color').value)
        if len(combined_color_param) != 4:
            combined_color_param = [0.2, 0.7, 0.9, 0.6]
        self.combined_color = make_color(*combined_color_param)

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        publish_rate = publish_rate if publish_rate > 1e-3 else 10.0
        self.tf_lookup_timeout = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self.tf_stale_threshold = float(self.get_parameter('tf_stale_threshold_sec').value)

        # ---- Runtime ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.tick)

        self.states: Dict[int, PlaneState] = {}
        self.corner_cache: Dict[float, np.ndarray] = {}

        self.get_logger().info(
            f"[tag_plane_marker_node] polling TF for ids={self.target_ids}, publishing markers to {self.marker_topic}, "
            f"output_frame={self.output_frame}"
        )

    def _build_lookup(self, keys: List[int], values: List, label: str) -> Dict[int, Any]:
        if len(keys) != len(values):
            if values:
                self.get_logger().warn(
                    f"Ignoring {label} overrides because list sizes differ "
                    f"(ids={len(keys)} vs values={len(values)})."
                )
            return {}
        return {int(k): values[i] for i, k in enumerate(keys)}

    def resolve_frame(self, tag_id: int) -> str:
        if tag_id in self.tag_frame_map:
            return str(self.tag_frame_map[tag_id])
        if '{id}' in self.frame_template:
            return self.frame_template.format(id=tag_id)
        return self.frame_template

    def resolve_size(self, tag_id: int) -> float:
        return float(self.tag_size_map.get(tag_id, self.default_tag_size))

    def get_corner_template(self, size: float) -> np.ndarray:
        """
        Returns homogeneous coordinates (4x4) for the four tag corners in the tag frame.
        Cache per size to avoid reallocation.
        """
        key = round(size, 6)
        if key not in self.corner_cache:
            half = size * 0.5
            self.corner_cache[key] = np.array(
                [
                    [half, half, 0.0, 1.0],
                    [-half, half, 0.0, 1.0],
                    [-half, -half, 0.0, 1.0],
                    [half, -half, 0.0, 1.0],
                ],
                dtype=float,
            )
        return self.corner_cache[key]

    def tick(self) -> None:
        self._poll_tag_transforms()
        self.publish_markers()

    def _poll_tag_transforms(self) -> None:
        query_time = Time()
        now_ns = self.get_clock().now().nanoseconds
        for tag_id in self.target_ids:
            tag_frame = self.resolve_frame(tag_id)
            tag_size = self.resolve_size(tag_id)
            if not math.isfinite(tag_size) or tag_size <= 0.0:
                self.get_logger().warn(f"Tag size for id {tag_id} is invalid ({tag_size}); skipping.")
                continue
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    tag_frame,
                    query_time,
                    timeout=Duration(seconds=self.tf_lookup_timeout),
                )
            except TransformException as exc:
                self.get_logger().debug(f"TF lookup failed for {self.output_frame} <- {tag_frame}: {exc}")
                continue

            stamp_ns = tf_msg.header.stamp.sec * 1_000_000_000 + tf_msg.header.stamp.nanosec
            if stamp_ns == 0:
                stamp_ns = now_ns
            age = (now_ns - stamp_ns) * 1e-9
            if self.tf_stale_threshold > 0.0 and age > self.tf_stale_threshold:
                self.get_logger().debug(
                    f"Skipping {tag_frame} due to stale TF (age={age:.2f}s > {self.tf_stale_threshold:.2f}s)"
                )
                continue

            corners_world = self._transform_corners(tf_msg, tag_size)
            self._update_state(tag_id, corners_world, stamp_ns)

    def _transform_corners(self, tf_msg, tag_size: float) -> np.ndarray:
        translation = [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ]
        rotation = [
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ]
        T = mat4_from_tf(translation, rotation)
        corners_local = self.get_corner_template(tag_size)
        corners_world = (T @ corners_local.T).T[:, :3]
        return corners_world

    def _update_state(self, tag_id: int, corners: np.ndarray, stamp_ns: int) -> None:
        if tag_id in self.states:
            prev = self.states[tag_id].corners
            blended = self.alpha * corners + (1.0 - self.alpha) * prev
        else:
            blended = corners
        self.states[tag_id] = PlaneState(corners=blended, last_update_ns=stamp_ns)

    def publish_markers(self) -> None:
        if not self.states:
            return
        now_ns = self.get_clock().now().nanoseconds
        expired_ids = []

        per_tag_markers: List[Marker] = []

        for idx, tag_id in enumerate(sorted(self.states.keys())):
            state = self.states[tag_id]
            if self.state_timeout > 0.0 and state.age(now_ns) > self.state_timeout:
                expired_ids.append(tag_id)
                continue
            if self.publish_individual:
                marker = self._build_marker(tag_id, state, idx)
                per_tag_markers.append(marker)

        for tag_id in expired_ids:
            del self.states[tag_id]

        combined_marker = self._build_combined_marker()
        markers_to_publish = []
        if self.publish_individual:
            markers_to_publish.extend(per_tag_markers)
        if combined_marker is not None:
            markers_to_publish.append(combined_marker)

        if markers_to_publish:
            marker_array = MarkerArray()
            marker_array.markers = markers_to_publish
            self.marker_pub.publish(marker_array)

    def _build_marker(self, tag_id: int, state: PlaneState, color_idx: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_ns
        marker.id = tag_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()

        corners = state.corners
        triangles = [
            corners[0], corners[1], corners[2],
            corners[0], corners[2], corners[3],
        ]
        marker.points = [to_point(p) for p in triangles]

        color = self.COLOR_TABLE[color_idx % len(self.COLOR_TABLE)]
        marker.color = color

        return marker

    def _build_combined_marker(self) -> Optional[Marker]:
        if not self.states:
            return None

        all_corners = [state.corners for state in self.states.values()]
        points = np.vstack(all_corners)
        if points.shape[0] < 3:
            return None

        centroid = np.mean(points, axis=0)
        centered = points - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None

        normal = vh[2]
        tangent1 = vh[0]
        tangent2 = vh[1]

        tangent1 /= (np.linalg.norm(tangent1) + 1e-12)
        tangent2 /= (np.linalg.norm(tangent2) + 1e-12)
        if np.linalg.norm(normal) < 1e-9:
            normal = np.cross(tangent1, tangent2)
        normal /= (np.linalg.norm(normal) + 1e-12)

        u_coords = centered @ tangent1
        v_coords = centered @ tangent2
        min_u, max_u = float(np.min(u_coords)), float(np.max(u_coords))
        min_v, max_v = float(np.min(v_coords)), float(np.max(v_coords))

        if abs(max_u - min_u) < 1e-4:
            half = 0.05
            min_u, max_u = -half, half
        if abs(max_v - min_v) < 1e-4:
            half = 0.05
            min_v, max_v = -half, half

        corners = [
            centroid + min_u * tangent1 + min_v * tangent2,
            centroid + max_u * tangent1 + min_v * tangent2,
            centroid + max_u * tangent1 + max_v * tangent2,
            centroid + min_u * tangent1 + max_v * tangent2,
        ]

        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_ns
        marker.id = self.combined_marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()

        triangles = [
            corners[0], corners[1], corners[2],
            corners[0], corners[2], corners[3],
        ]
        marker.points = [to_point(p) for p in triangles]
        marker.color = self.combined_color
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagPlaneMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
