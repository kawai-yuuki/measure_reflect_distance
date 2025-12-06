#!/usr/bin/env python3

"""
Overlay AprilTag detections on the input image with distance and orientation text.

- Subscribes to apriltag_msgs/msg/AprilTagDetectionArray (default: /detections)
- Subscribes to camera image (default: /camera/camera/color/image_raw)
- Uses TF only (camera_frame -> tag_frame) to compute distance + RPY
- Publishes an annotated image topic for quick visual debugging
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple
import math
import os

import cv2
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException


@dataclass
class PoseInfo:
    distance_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def quaternion_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to roll, pitch, yaw in radians (ROS XYZ convention).
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def clamp(val: int, low: int, high: int) -> int:
    return max(low, min(high, val))


def _default_tag_config_path() -> str:
    """
    Try to resolve the shared reflect_tag_36h11.yaml from apriltag_ros.
    Return empty string if not found so the parameter remains optional.
    """
    try:
        share = get_package_share_directory("apriltag_ros")
        candidate = os.path.join(share, "cfg", "reflect_tag_36h11.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return ""


class AprilTagOverlayNode(Node):
    """
    Draw AprilTag bounding boxes, distance, and RPY (color-coded to TF axes) on the camera image.
    """

    BOX_COLOR = (0, 255, 255)  # yellow in BGR
    TEXT_COLOR = (255, 255, 255)
    SHADOW_COLOR = (0, 0, 0)
    R_COLOR = (0, 0, 255)  # X axis -> red
    P_COLOR = (0, 255, 0)  # Y axis -> green
    Y_COLOR = (255, 0, 0)  # Z axis -> blue

    def __init__(self) -> None:
        super().__init__("apriltag_overlay_node")

        # --- Parameters ---
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "~/overlay")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("tag_frame_template", "landmark_{id}")
        self.declare_parameter("tag_frame_ids", [])
        self.declare_parameter("tag_frame_names", [])
        default_cfg_path = _default_tag_config_path()
        self.declare_parameter("tag_config_path", default_cfg_path)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("max_detection_age_sec", 0.5)
        self.declare_parameter("font_scale", 0.5)
        self.declare_parameter("line_thickness", 2)

        detections_topic = self.get_parameter("detections_topic").value
        image_topic = self.get_parameter("image_topic").value
        output_topic = self.get_parameter("output_image_topic").value
        self.max_detection_age = float(self.get_parameter("max_detection_age_sec").value)
        self.font_scale = float(self.get_parameter("font_scale").value)
        self.line_thickness = int(self.get_parameter("line_thickness").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.frame_template = str(self.get_parameter("tag_frame_template").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        cfg_path_param = str(self.get_parameter("tag_config_path").value)
        frame_ids_param = self.get_parameter("tag_frame_ids").value
        frame_names_param = self.get_parameter("tag_frame_names").value
        self.tag_frame_map: Dict[int, str] = {}

        if cfg_path_param:
            self.tag_frame_map.update(self._load_tag_config(cfg_path_param))

        param_map = self._build_frame_lookup(frame_ids_param, frame_names_param)
        if param_map:
            # Explicit parameters override config
            self.tag_frame_map.update(param_map)

        # --- Runtime state ---
        self.bridge = CvBridge()
        self.last_detection_msg: Optional[AprilTagDetectionArray] = None
        self.last_detection_time: Optional[Time] = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- ROS entities ---
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.image_pub = self.create_publisher(Image, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f"[apriltag_overlay_node] listening to {detections_topic} + {image_topic}, "
            f"publishing overlay to {output_topic}, using TF only"
        )
        if cfg_path_param:
            self.get_logger().info(f"Using tag config: {cfg_path_param}")

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

    @staticmethod
    def _time_from_msg(stamp) -> Time:
        return Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self.last_detection_msg = msg
        self.last_detection_time = self._time_from_msg(msg.header.stamp)

    def _on_image(self, msg: Image) -> None:
        if self.last_detection_msg is None or self.last_detection_time is None:
            return

        img_time = self._time_from_msg(msg.header.stamp)
        age = (img_time - self.last_detection_time).nanoseconds * 1e-9
        if self.max_detection_age > 0.0 and age > self.max_detection_age:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - cv_bridge raises various types
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        annotated = frame.copy()
        for detection in self.last_detection_msg.detections:
            self._draw_detection(annotated, detection)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    def _resolve_frame(self, tag_id: int) -> str:
        if tag_id in self.tag_frame_map:
            return self.tag_frame_map[tag_id]
        if "{id}" in self.frame_template:
            return self.frame_template.format(id=tag_id)
        return self.frame_template

    def _lookup_pose(self, detection) -> Optional[PoseInfo]:
        tag_frame = self._resolve_frame(detection.id)
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

        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z
        distance = float(math.sqrt(tx * tx + ty * ty + tz * tz))
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w
        roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
        return PoseInfo(distance_m=distance, roll_deg=to_deg(roll), pitch_deg=to_deg(pitch), yaw_deg=to_deg(yaw))

    def _draw_detection(self, image: np.ndarray, detection) -> None:
        corners = detection.corners
        if len(corners) != 4:
            return
        pts = np.array([[float(p.x), float(p.y)] for p in corners], dtype=np.float32)
        pts_int = pts.astype(np.int32)
        cv2.polylines(image, [pts_int], True, self.BOX_COLOR, self.line_thickness, lineType=cv2.LINE_AA)

        center = (int(detection.centre.x), int(detection.centre.y))
        cv2.circle(image, center, 4, self.BOX_COLOR, -1, lineType=cv2.LINE_AA)

        pose = self._lookup_pose(detection)
        text_lines: List[Tuple[str, Tuple[int, int, int]]] = []
        text_lines.append((f"ID {detection.id}", self.TEXT_COLOR))
        if pose is not None:
            text_lines.append((f"Dist {pose.distance_m:.2f} m", self.TEXT_COLOR))
            text_lines.append((f"R {pose.roll_deg:+.1f} deg", self.R_COLOR))
            text_lines.append((f"P {pose.pitch_deg:+.1f} deg", self.P_COLOR))
            text_lines.append((f"Y {pose.yaw_deg:+.1f} deg", self.Y_COLOR))
        else:
            text_lines.append(("Pose unavailable", (80, 80, 255)))

        anchor = (
            clamp(int(np.min(pts[:, 0])) - 4, 0, image.shape[1] - 1),
            clamp(int(np.min(pts[:, 1])) - 6, 0, image.shape[0] - 1),
        )
        self._draw_text_block(image, text_lines, anchor)

    def _draw_text_block(
        self,
        image: np.ndarray,
        lines: Sequence[Tuple[str, Tuple[int, int, int]]],
        anchor: Tuple[int, int],
    ) -> None:
        x0, y0 = anchor
        line_height = int(16 * self.font_scale) + 6
        for idx, (text, color) in enumerate(lines):
            y = y0 + line_height * (idx + 1)
            # Shadow for readability
            cv2.putText(
                image,
                text,
                (x0 + 1, y + 1),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                self.SHADOW_COLOR,
                thickness=self.line_thickness,
                lineType=cv2.LINE_AA,
            )
            cv2.putText(
                image,
                text,
                (x0, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                color,
                thickness=self.line_thickness,
                lineType=cv2.LINE_AA,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
