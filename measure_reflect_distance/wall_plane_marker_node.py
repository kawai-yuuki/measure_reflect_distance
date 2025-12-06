#!/usr/bin/env python3

"""
Estimate and visualize a wall plane from AprilTag TFs (IDs 7, 8).

Assumptions:
- Tags with ID 7 and 8 are mounted on the same wall; at least one tag yields a plane.
- Tag +Z is aligned with the wall normal (orientation matters).
- Frame names are loaded from a YAML config (default: apriltag_ros/cfg/reflect_tag_36h11.yaml).

Markers published:
- Center sphere (wall point)
- Normal arrow (points along wall normal)
- Tag positions used (cubes)
- Status text (tags used count)

No square patch or corners are drawn (user requested minimal visualization).
"""

from typing import Dict, List, Optional, Sequence, Tuple
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


class WallPlaneMarkerNode(Node):
    """Minimal wall plane visualization from tag TFs."""

    COLOR_CENTER = make_color(0.2, 0.6, 0.9, 0.9)
    COLOR_NORMAL = make_color(0.9, 0.2, 0.2, 0.9)
    COLOR_PLANE = make_color(0.2, 0.8, 0.8, 0.4)  # semi-transparent cyan-ish (different from mirror)
    COLOR_TAG = make_color(0.9, 0.2, 0.6, 0.9)
    COLOR_TEXT = make_color(1.0, 1.0, 1.0, 1.0)

    def __init__(self) -> None:
        super().__init__("wall_plane_marker_node")

        # Parameters
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("marker_topic", "/wall_plane_markers")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("tag_config_path", _default_tag_config_path())
        self.declare_parameter("tag_frame_ids", [])
        self.declare_parameter("tag_frame_names", [])
        self.declare_parameter("tag_ids", [7, 8])
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("marker_lifetime_sec", 0.2)
        self.declare_parameter("normal_length_m", 0.3)
        self.declare_parameter("plane_size_m", 1.0)

        detections_topic = self.get_parameter("detections_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime_sec").value)
        self.normal_length = float(self.get_parameter("normal_length_m").value)
        self.plane_size = float(self.get_parameter("plane_size_m").value)
        cfg_path_param = str(self.get_parameter("tag_config_path").value)

        # Frame mapping: YAML -> explicit overrides
        self.tag_frame_map: Dict[int, str] = {}
        if cfg_path_param:
            self.tag_frame_map.update(self._load_tag_config(cfg_path_param))
        param_map = self._build_frame_lookup(
            self.get_parameter("tag_frame_ids").value,
            self.get_parameter("tag_frame_names").value,
        )
        if param_map:
            self.tag_frame_map.update(param_map)

        self.target_ids = [int(v) for v in self.get_parameter("tag_ids").value]

        # Runtime
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.last_detections: Optional[AprilTagDetectionArray] = None
        self.last_detection_time: Optional[Time] = None

        # ROS entities
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._tick)

        self.get_logger().info(
            f"[wall_plane_marker_node] listening to {detections_topic}, publishing markers to {marker_topic}, "
            f"camera_frame={self.camera_frame}"
        )
        if cfg_path_param:
            self.get_logger().info(f"Using tag config: {cfg_path_param}")

    # -------- helpers --------
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

    def _resolve_frame(self, tag_id: int) -> Optional[str]:
        return self.tag_frame_map.get(tag_id)

    # -------- callbacks --------
    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self.last_detections = msg
        self.last_detection_time = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)

    def _tick(self) -> None:
        if self.last_detections is None or self.last_detection_time is None:
            return
        estimates = self._collect_tag_estimates()
        if not estimates:
            return
        center, normal = self._fuse_center_normal(estimates)
        if center is None or normal is None:
            return
        markers = self._build_markers(center, normal, estimates)
        self.marker_pub.publish(markers)

    # -------- estimation --------
    def _lookup_tag_pose(self, tag_id: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        frame = self._resolve_frame(tag_id)
        if frame is None:
            return None
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame,
                frame,
                Time(),  # latest
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().debug(f"TF lookup failed for {self.camera_frame} <- {frame}: {exc}")
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

    def _collect_tag_estimates(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        out: List[Tuple[np.ndarray, np.ndarray]] = []
        for tag_id in self.target_ids:
            pose = self._lookup_tag_pose(tag_id)
            if pose is None:
                continue
            R, t = pose
            normal = R[:, 2]
            out.append((t, normal))
        return out

    def _fuse_center_normal(
        self, estimates: List[Tuple[np.ndarray, np.ndarray]]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not estimates:
            return None, None
        centers = np.stack([e[0] for e in estimates], axis=0)
        normals = np.stack([e[1] for e in estimates], axis=0)
        center = np.mean(centers, axis=0)
        normal = np.mean(normals, axis=0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            return center, None
        return center, normal / norm

    # -------- markers --------
    def _build_markers(
        self,
        center: np.ndarray,
        normal: np.ndarray,
        estimates: List[Tuple[np.ndarray, np.ndarray]],
    ) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        lifetime_msg = Duration(seconds=self.marker_lifetime).to_msg()
        markers: List[Marker] = []
        ns = "wall_plane"
        marker_id = 0

        # Plane patch (square for visualization only)
        half = self.plane_size * 0.5
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
        R_plane = np.stack([x_axis, y_axis, normal], axis=1)
        corners = [center + R_plane @ off for off in offsets]

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

        # Center sphere
        m_center = Marker()
        m_center.header.frame_id = self.camera_frame
        m_center.header.stamp = now
        m_center.ns = ns
        m_center.id = marker_id
        m_center.type = Marker.SPHERE
        m_center.action = Marker.ADD
        m_center.scale.x = m_center.scale.y = m_center.scale.z = 0.05
        m_center.pose.position = to_point(center)
        m_center.pose.orientation.w = 1.0
        m_center.lifetime = lifetime_msg
        m_center.color = self.COLOR_CENTER
        markers.append(m_center)
        marker_id += 1

        # Normal arrow (arrow points along +X in local marker frame)
        arrow = Marker()
        arrow.header.frame_id = self.camera_frame
        arrow.header.stamp = now
        arrow.ns = ns
        arrow.id = marker_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = self.normal_length  # shaft length
        arrow.scale.y = 0.02  # shaft diameter
        arrow.scale.z = 0.04  # head diameter
        arrow.pose.position = to_point(center)
        q = quat_from_two_vectors(np.array([1.0, 0.0, 0.0]), normal)
        arrow.pose.orientation.x = float(q[0])
        arrow.pose.orientation.y = float(q[1])
        arrow.pose.orientation.z = float(q[2])
        arrow.pose.orientation.w = float(q[3])
        arrow.lifetime = lifetime_msg
        arrow.color = self.COLOR_NORMAL
        markers.append(arrow)
        marker_id += 1

        # Tag positions used
        for t, _n in estimates:
            m = Marker()
            m.header.frame_id = self.camera_frame
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.pose.position = to_point(t)
            m.pose.orientation.w = 1.0
            m.lifetime = lifetime_msg
            m.color = self.COLOR_TAG
            markers.append(m)
            marker_id += 1

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
    node = WallPlaneMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
