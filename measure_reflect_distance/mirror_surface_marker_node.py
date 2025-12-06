#!/usr/bin/env python3

"""
Estimate a square mirror surface from AprilTag poses and visualize it with RViz markers.

Assumptions:
- Mirror is a square of side `square_size_m` (default 0.60 m).
- Tags are mounted on the mirror as:
    ID 1: top-left, ID 2: top-right, ID 3: bottom-left, ID 4: bottom-right, ID 5: center.
- All tag frames are parallel to the mirror plane (tag +Z is mirror normal).
- Tag frame names are resolved via tag_config_path (default reflect_tag_36h11.yaml in apriltag_ros)
  and/or tag_frame_template / tag_frame_ids + tag_frame_names.

With at least one tag, the node reconstructs the mirror center, corners, and normal in the camera frame,
and publishes MarkerArray for RViz: plane patch, corners, center, normal arrow, tag points, and status text.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple
import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from measure_reflect_distance.util.mirror_geometry import rot_from_quat, quat_from_two_vectors


@dataclass
class TagEstimate:
    center: np.ndarray  # 3,
    normal: np.ndarray  # 3,
    tag_pos: np.ndarray  # 3,


def to_point(vec3: np.ndarray) -> Point:
    msg = Point()
    msg.x = float(vec3[0])
    msg.y = float(vec3[1])
    msg.z = float(vec3[2])
    return msg


def make_color(r: float, g: float, b: float, a: float = 0.8) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def _default_tag_config_path() -> str:
    try:
        share = get_package_share_directory("apriltag_ros")
        candidate = os.path.join(share, "cfg", "reflect_tag_36h11.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return ""


class MirrorSurfaceMarkerNode(Node):
    """Reconstruct a known square mirror surface from AprilTag TF poses and publish RViz markers."""

    COLOR_PLANE = make_color(1.0, 0.85, 0.2, 0.5)
    COLOR_CORNER = make_color(0.2, 0.8, 0.4, 0.9)
    COLOR_CENTER = make_color(0.2, 0.6, 0.9, 0.9)
    COLOR_TAG = make_color(0.9, 0.2, 0.6, 0.9)
    COLOR_NORMAL = make_color(0.9, 0.1, 0.1, 0.9)
    COLOR_TEXT = make_color(1.0, 1.0, 1.0, 1.0)

    def __init__(self) -> None:
        super().__init__("mirror_surface_marker_node")

        # --- Parameters ---
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("marker_topic", "/mirror_surface_markers")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("tag_frame_template", "landmark_{id}")
        self.declare_parameter("tag_frame_ids", [])
        self.declare_parameter("tag_frame_names", [])
        self.declare_parameter("tag_config_path", _default_tag_config_path())
        self.declare_parameter("square_size_m", 0.60)
        self.declare_parameter("tag_ids", [1, 2, 3, 4, 5])
        self.declare_parameter("layout_pos_x", [-0.3, 0.3, -0.3, 0.3, 0.0])
        self.declare_parameter("layout_pos_y", [0.3, 0.3, -0.3, -0.3, 0.0])
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("marker_lifetime_sec", 0.2)
        self.declare_parameter("normal_length_m", 0.2)
        self.declare_parameter("tf_timeout_sec", 0.05)

        detections_topic = self.get_parameter("detections_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.frame_template = str(self.get_parameter("tag_frame_template").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.square_size = float(self.get_parameter("square_size_m").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime_sec").value)
        self.normal_length = float(self.get_parameter("normal_length_m").value)

        cfg_path_param = str(self.get_parameter("tag_config_path").value)
        frame_ids_param = self.get_parameter("tag_frame_ids").value
        frame_names_param = self.get_parameter("tag_frame_names").value
        self.tag_frame_map: Dict[int, str] = {}
        if cfg_path_param:
            self.tag_frame_map.update(self._load_tag_config(cfg_path_param))
        param_map = self._build_frame_lookup(frame_ids_param, frame_names_param)
        if param_map:
            self.tag_frame_map.update(param_map)

        tag_ids_param = [int(v) for v in self.get_parameter("tag_ids").value]
        layout_x = [float(v) for v in self.get_parameter("layout_pos_x").value]
        layout_y = [float(v) for v in self.get_parameter("layout_pos_y").value]
        self.layout = self._build_layout(tag_ids_param, layout_x, layout_y)

        # --- Runtime ---
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.last_detections: Optional[AprilTagDetectionArray] = None
        self.last_detection_time: Optional[Time] = None

        # --- ROS entities ---
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._tick)

        self.get_logger().info(
            f"[mirror_surface_marker_node] listening to {detections_topic}, publishing markers to {marker_topic}, "
            f"camera_frame={self.camera_frame}, square_size={self.square_size:.3f} m"
        )
        if cfg_path_param:
            self.get_logger().info(f"Using tag config: {cfg_path_param}")

    # ---------------- Parameter helpers ----------------
    def _load_tag_config(self, path: str) -> Dict[int, str]:
        path = os.path.expanduser(path)
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().warn(f"Failed to load tag config from {path}: {exc}")
            return {}

        def _extract_tag_block(obj):
            if not isinstance(obj, dict):
                return None
            if "tag" in obj:
                return obj.get("tag")
            root = obj.get("/**", {})
            if isinstance(root, dict):
                params = root.get("ros__parameters", {})
                if isinstance(params, dict) and "tag" in params:
                    return params.get("tag")
            return None

        tag_block = _extract_tag_block(data)
        if not isinstance(tag_block, dict):
            self.get_logger().warn(f"No 'tag' block found in config {path}")
            return {}

        ids = tag_block.get("ids", [])
        frames = tag_block.get("frames", [])
        if len(ids) != len(frames):
            self.get_logger().warn(
                f"Ignoring tag config in {path} because ids and frames length differ "
                f"(ids={len(ids)}, frames={len(frames)})"
            )
            return {}
        mapping = {int(ids[i]): str(frames[i]) for i in range(len(ids))}
        self.get_logger().info(f"Loaded {len(mapping)} tag frames from {path}")
        return mapping

    def _build_frame_lookup(self, ids: Sequence[int], names: Sequence[str]) -> Dict[int, str]:
        if len(ids) != len(names):
            if names:
                self.get_logger().warn(
                    f"Ignoring frame overrides because list sizes differ (ids={len(ids)} vs names={len(names)})."
                )
            return {}
        return {int(ids[i]): str(names[i]) for i in range(len(ids))}

    def _build_layout(self, ids: Sequence[int], xs: Sequence[float], ys: Sequence[float]) -> Dict[int, np.ndarray]:
        if not (len(ids) == len(xs) == len(ys)):
            self.get_logger().warn(
                f"Ignoring layout overrides because sizes differ (ids={len(ids)}, xs={len(xs)}, ys={len(ys)}). "
                "Using defaults."
            )
            ids = [1, 2, 3, 4, 5]
            xs = [-0.3, 0.3, -0.3, 0.3, 0.0]
            ys = [0.3, 0.3, -0.3, -0.3, 0.0]
        return {int(ids[i]): np.array([xs[i], ys[i], 0.0], dtype=float) for i in range(len(ids))}

    # ---------------- Callbacks ----------------
    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self.last_detections = msg
        self.last_detection_time = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)

    def _tick(self) -> None:
        if self.last_detections is None or self.last_detection_time is None:
            return
        used_estimates = self._collect_tag_estimates()
        if not used_estimates:
            return
        center, normal = self._fuse_center_normal(used_estimates)
        if center is None or normal is None:
            return
        corners = self._compute_corners(center, normal)
        markers = self._build_markers(center, normal, corners, used_estimates)
        self.marker_pub.publish(markers)

    # ---------------- Estimation ----------------
    def _resolve_frame(self, tag_id: int) -> str:
        if tag_id in self.tag_frame_map:
            return self.tag_frame_map[tag_id]
        if "{id}" in self.frame_template:
            return self.frame_template.format(id=tag_id)
        return self.frame_template

    def _lookup_tag_pose(self, tag_id: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        tag_frame = self._resolve_frame(tag_id)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame,
                tag_frame,
                Time(),  # latest
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().debug(f"TF lookup failed for {self.camera_frame} <- {tag_frame}: {exc}")
            return None
        t = np.array(
            [
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            ],
            dtype=float,
        )
        R = rot_from_quat(
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        )
        return R, t

    def _collect_tag_estimates(self) -> List[TagEstimate]:
        estimates: List[TagEstimate] = []
        for tag_id, tag_pos_board in self.layout.items():
            pose = self._lookup_tag_pose(tag_id)
            if pose is None:
                continue
            R_tag, t_tag = pose
            # board center is tag position minus its offset on the board
            offset_to_center = -tag_pos_board
            center_world = t_tag + R_tag @ offset_to_center
            normal = R_tag[:, 2]
            estimates.append(TagEstimate(center=center_world, normal=normal, tag_pos=t_tag))
        return estimates

    def _fuse_center_normal(self, estimates: List[TagEstimate]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not estimates:
            return None, None
        centers = np.stack([e.center for e in estimates], axis=0)
        normals = np.stack([e.normal for e in estimates], axis=0)
        center = np.mean(centers, axis=0)
        normal = np.mean(normals, axis=0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            return center, None
        normal = normal / norm
        return center, normal

    def _compute_corners(self, center: np.ndarray, normal: np.ndarray) -> List[np.ndarray]:
        half = self.square_size * 0.5
        # Build an orthonormal basis: x_axis in plane, z_axis = normal
        ref = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(ref, normal)) > 0.9:
            ref = np.array([0.0, 1.0, 0.0])
        x_axis = ref - np.dot(ref, normal) * normal
        x_axis /= (np.linalg.norm(x_axis) + 1e-12)
        y_axis = np.cross(normal, x_axis)
        y_axis /= (np.linalg.norm(y_axis) + 1e-12)
        offsets = [
            np.array([-half, half, 0.0]),
            np.array([half, half, 0.0]),
            np.array([half, -half, 0.0]),
            np.array([-half, -half, 0.0]),
        ]
        R_board = np.stack([x_axis, y_axis, normal], axis=1)
        corners = [center + R_board @ off for off in offsets]
        return corners

    # ---------------- Markers ----------------
    def _build_markers(
        self,
        center: np.ndarray,
        normal: np.ndarray,
        corners: List[np.ndarray],
        estimates: List[TagEstimate],
    ) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        lifetime_msg = Duration(seconds=self.marker_lifetime).to_msg()
        markers: List[Marker] = []
        ns = "mirror_surface"
        marker_id = 0

        # Plane patch
        plane = Marker()
        plane.header.frame_id = self.camera_frame
        plane.header.stamp = now
        plane.ns = ns
        plane.id = marker_id
        marker_id += 1
        plane.type = Marker.TRIANGLE_LIST
        plane.action = Marker.ADD
        plane.scale.x = plane.scale.y = plane.scale.z = 1.0
        plane.pose.orientation.w = 1.0
        plane.lifetime = lifetime_msg
        tri = [corners[0], corners[1], corners[2], corners[0], corners[2], corners[3]]
        plane.points = [to_point(p) for p in tri]
        plane.color = self.COLOR_PLANE
        markers.append(plane)

        # Corners
        for c in corners:
            m = Marker()
            m.header.frame_id = self.camera_frame
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.pose.position = to_point(c)
            m.pose.orientation.w = 1.0
            m.lifetime = lifetime_msg
            m.color = self.COLOR_CORNER
            markers.append(m)

        # Center
        center_marker = Marker()
        center_marker.header.frame_id = self.camera_frame
        center_marker.header.stamp = now
        center_marker.ns = ns
        center_marker.id = marker_id
        marker_id += 1
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.04
        center_marker.pose.position = to_point(center)
        center_marker.pose.orientation.w = 1.0
        center_marker.lifetime = lifetime_msg
        center_marker.color = self.COLOR_CENTER
        markers.append(center_marker)

        # Normal arrow
        arrow = Marker()
        arrow.header.frame_id = self.camera_frame
        arrow.header.stamp = now
        arrow.ns = ns
        arrow.id = marker_id
        marker_id += 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = self.normal_length  # shaft length
        arrow.scale.y = 0.02  # shaft diameter
        arrow.scale.z = 0.04  # head diameter
        arrow.pose.position = to_point(center)
        # Marker ARROW points along +X in its local frame, so rotate X-axis to the plane normal.
        q = quat_from_two_vectors(np.array([1.0, 0.0, 0.0]), normal)
        arrow.pose.orientation.x = float(q[0])
        arrow.pose.orientation.y = float(q[1])
        arrow.pose.orientation.z = float(q[2])
        arrow.pose.orientation.w = float(q[3])
        arrow.lifetime = lifetime_msg
        arrow.color = self.COLOR_NORMAL
        markers.append(arrow)

        # Tag positions used
        for est in estimates:
            m = Marker()
            m.header.frame_id = self.camera_frame
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.025
            m.pose.position = to_point(est.tag_pos)
            m.pose.orientation.w = 1.0
            m.lifetime = lifetime_msg
            m.color = self.COLOR_TAG
            markers.append(m)

        # Status text
        text = Marker()
        text.header.frame_id = self.camera_frame
        text.header.stamp = now
        text.ns = ns
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.04
        text.pose.position = to_point(center + np.array([0.0, 0.0, 0.05]))
        text.pose.orientation.w = 1.0
        text.lifetime = lifetime_msg
        text.color = self.COLOR_TEXT
        text.text = f"tags used: {len(estimates)}"
        markers.append(text)

        msg = MarkerArray()
        msg.markers = markers
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MirrorSurfaceMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
