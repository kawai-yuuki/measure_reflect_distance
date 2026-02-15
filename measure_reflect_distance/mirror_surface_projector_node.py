#!/usr/bin/env python3

from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, Header
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from measure_reflect_distance.util.mirror_geometry import rot_from_quat


class MirrorSurfaceProjectorNode(Node):
    """Project mask pixels/contours to a mirror plane and publish markers + projected points."""

    def __init__(self) -> None:
        super().__init__("mirror_surface_projector_node")

        # Input/Output parameters
        self.declare_parameter("mask_topic", "/mask_image_processed")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("plane_topic", "mirror_plane_clustered")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("default_camera_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_topic", "mirror_surface_projected_markers_raw")
        self.declare_parameter("projected_points_topic", "mirror_surface_projected_points")

        # Sync / filtering parameters
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("plane_max_age_sec", 0.5)
        self.declare_parameter("marker_line_width", 0.01)
        self.declare_parameter("min_mask_area", 200.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("projection_mode", "contour")  # contour / dense
        self.declare_parameter("pixel_stride", 2)
        self.declare_parameter("point_scale", 0.03)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("projection_min_abs_denom", 1e-4)
        self.declare_parameter("max_projection_distance", 10.0)

        # Cluster/plane parameters
        self.declare_parameter("clustered_observation_topic", "mirror_plane_clustered_observation")
        self.declare_parameter("cluster_plane_topic", "mirror_plane_clustered")

        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        plane_topic = self.get_parameter("plane_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self._points_topic = (
            self.get_parameter("projected_points_topic").get_parameter_value().string_value
        )
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.default_camera_frame = (
            self.get_parameter("default_camera_frame").get_parameter_value().string_value
        )
        queue_size = max(1, self.get_parameter("sync_queue_size").get_parameter_value().integer_value)
        slop = max(0.0, self.get_parameter("sync_slop").get_parameter_value().double_value)
        self._plane_max_age_sec = max(
            0.0, self.get_parameter("plane_max_age_sec").get_parameter_value().double_value
        )
        self.marker_line_width = max(
            0.001, self.get_parameter("marker_line_width").get_parameter_value().double_value
        )
        self._min_mask_area = max(0.0, self.get_parameter("min_mask_area").get_parameter_value().double_value)
        self._tf_timeout = max(0.0, self.get_parameter("tf_timeout_sec").get_parameter_value().double_value)
        self._projection_mode = (
            self.get_parameter("projection_mode").get_parameter_value().string_value.lower()
        )
        if self._projection_mode not in ("contour", "dense"):
            self.get_logger().warn(
                f"Unknown projection_mode '{self._projection_mode}', fallback to 'contour'."
            )
            self._projection_mode = "contour"
        self._pixel_stride = max(
            1, int(self.get_parameter("pixel_stride").get_parameter_value().integer_value)
        )
        self._point_scale = max(0.001, self.get_parameter("point_scale").get_parameter_value().double_value)
        self._max_fps = float(self.get_parameter("max_fps").get_parameter_value().double_value)
        self._projection_min_abs_denom = max(
            1e-9, float(self.get_parameter("projection_min_abs_denom").get_parameter_value().double_value)
        )
        self._max_projection_distance = max(
            0.0, float(self.get_parameter("max_projection_distance").get_parameter_value().double_value)
        )
        self._min_interval_ns = 0 if self._max_fps <= 0.0 else int(1e9 / self._max_fps)
        self._last_process_time: Optional[Time] = None

        self._clustered_observation_topic = (
            self.get_parameter("clustered_observation_topic").get_parameter_value().string_value
        )
        self._cluster_plane_topic = (
            self.get_parameter("cluster_plane_topic").get_parameter_value().string_value
        )

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.points_pub = self.create_publisher(PointCloud2, self._points_topic, 10)

        self._latest_synced_stamp: Optional[Time] = None
        self._last_tf_warn_sec: float = -float("inf")
        self._last_plane_warn_sec: float = -float("inf")
        self._latest_cluster_observation: Optional[Tuple[int, Float64MultiArray, Time]] = None
        self._cluster_planes: dict[int, Tuple[Float64MultiArray, Time]] = {}
        self._persistent_marker_id: int = 0

        # Synchronize mask and camera info
        self.mask_sub = Subscriber(self, Image, mask_topic)
        self.camera_info_sub = Subscriber(self, CameraInfo, camera_info_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.mask_sub, self.camera_info_sub], queue_size=queue_size, slop=slop
        )
        self.sync.registerCallback(self._synced_callback)

        # Mirror plane observations arrive asynchronously
        self.create_subscription(
            Float64MultiArray, self._clustered_observation_topic, self._cluster_observation_callback, 10
        )
        self.create_subscription(Float64MultiArray, self._cluster_plane_topic, self._cluster_plane_callback, 10)

        self.get_logger().info(
            "MirrorSurfaceProjectorNode ready "
            f"(mask_topic={mask_topic}, camera_info_topic={camera_info_topic}, "
            f"cluster_plane_topic={self._cluster_plane_topic}, "
            f"projection_min_abs_denom={self._projection_min_abs_denom:.1e}, "
            f"max_projection_distance={self._max_projection_distance:.2f}m)"
        )

    def _cluster_observation_callback(self, msg: Float64MultiArray) -> None:
        data = msg.data
        if len(data) < 13:
            return
        cluster_id = int(round(data[0]))
        trimmed = list(data[1:11])  # normal(3), distance, point(3), tangent(3)
        obs_msg = Float64MultiArray()
        obs_msg.data = trimmed
        stamp_sec = float(data[12])
        sec = int(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        obs_time = Time(seconds=sec, nanoseconds=nanosec)
        self._latest_cluster_observation = (cluster_id, obs_msg, obs_time)

    def _cluster_plane_callback(self, msg: Float64MultiArray) -> None:
        data = msg.data
        if not data:
            return
        cluster_id = int(round(data[0]))
        trimmed = list(data[1:])
        if len(trimmed) >= 11:
            trimmed = trimmed[:-1]  # drop support count
        plane_msg = Float64MultiArray()
        plane_msg.data = trimmed
        self._cluster_planes[cluster_id] = (plane_msg, self.get_clock().now())

    def _synced_callback(self, mask_msg: Image, camera_info_msg: CameraInfo) -> None:
        mask_time = Time.from_msg(mask_msg.header.stamp)
        camera_frame = camera_info_msg.header.frame_id or self.default_camera_frame
        self._latest_synced_stamp = mask_time

        if self._min_interval_ns > 0:
            now = self.get_clock().now()
            if self._last_process_time is not None:
                elapsed_ns = (now - self._last_process_time).nanoseconds
                if elapsed_ns < self._min_interval_ns:
                    return
            self._last_process_time = now

        plane_msg: Optional[Float64MultiArray] = None
        plane_stamp: Optional[Time] = None
        plane_available = False
        used_cluster_id: Optional[int] = None

        if self._latest_cluster_observation is not None:
            obs_cluster_id, obs_msg, obs_time = self._latest_cluster_observation
            age_obs = max(0.0, (mask_time.nanoseconds - obs_time.nanoseconds) / 1e9)
            if self._plane_max_age_sec <= 0.0 or age_obs <= self._plane_max_age_sec:
                plane_entry = self._cluster_planes.get(obs_cluster_id)
                if plane_entry is not None:
                    plane_msg, plane_stamp = plane_entry
                    plane_available = True
                    used_cluster_id = obs_cluster_id
                else:
                    plane_msg = obs_msg
                    plane_stamp = obs_time
                    plane_available = True
                    used_cluster_id = obs_cluster_id
            else:
                self.get_logger().debug(
                    f"Latest cluster observation (id={obs_cluster_id}) too old ({age_obs:.2f}s); ignoring."
                )

        if not plane_available:
            self._log_throttled_warning(
                "_last_plane_warn_sec",
                5.0,
                "No clustered mirror plane data received yet; waiting before projection.",
            )
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        age_sec = 0.0
        if isinstance(plane_stamp, Time):
            age_sec = (mask_time.nanoseconds - plane_stamp.nanoseconds) / 1e9
        elif plane_stamp is not None:
            age_sec = (mask_time.nanoseconds - Time.from_msg(plane_stamp).nanoseconds) / 1e9
        if age_sec > self._plane_max_age_sec > 0.0:
            self._log_throttled_warning(
                "_last_plane_warn_sec",
                5.0,
                f"Mirror plane data too old ({age_sec:.2f}s); skipping projection.",
            )
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        tf_ok = False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                mask_time,
                timeout=Duration(seconds=self._tf_timeout),
            )
            tf_ok = True
        except tf2_ros.ExtrapolationException as ex:
            latest_time = Time()
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    latest_time,
                    timeout=Duration(seconds=self._tf_timeout),
                )
                tf_ok = True
                self.get_logger().debug(
                    f"Using latest TF for projection due to extrapolation at {mask_time.to_msg()}"
                )
            except Exception as ex_latest:
                self._log_throttled_warning(
                    "_last_tf_warn_sec",
                    5.0,
                    f"TF lookup failed (future extrapolation) for {self.target_frame} <- {camera_frame} "
                    f"at {mask_time.to_msg()}: {ex}; latest lookup error: {ex_latest}",
                )
                self._publish_marker_delete(mask_msg.header.stamp)
                return
        except Exception as ex:
            self._log_throttled_warning(
                "_last_tf_warn_sec",
                5.0,
                f"TF lookup failed for {self.target_frame} <- {camera_frame} at {mask_time.to_msg()}: {ex}",
            )
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        try:
            mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
        except CvBridgeError as err:
            self.get_logger().error(f"Failed to convert mask image: {err}")
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        fx = camera_info_msg.k[0]
        fy = camera_info_msg.k[4]
        cx = camera_info_msg.k[2]
        cy = camera_info_msg.k[5]
        if fx == 0.0 or fy == 0.0:
            self.get_logger().error("Camera intrinsics invalid (fx or fy is zero).")
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        plane_data = np.array(plane_msg.data, dtype=np.float64)
        if plane_data.size < 4:
            self.get_logger().warn("Mirror plane message too short; expected at least 4 elements.")
            self._publish_marker_delete(mask_msg.header.stamp)
            return
        n = plane_data[:3]
        d = plane_data[3]
        norm_n = np.linalg.norm(n)
        if norm_n < 1e-6:
            self.get_logger().warn("Mirror plane normal has near-zero norm; skipping projection.")
            self._publish_marker_delete(mask_msg.header.stamp)
            return
        n = n / norm_n
        d = d / norm_n

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        origin = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        R_target_camera = rot_from_quat(rotation.x, rotation.y, rotation.z, rotation.w)

        frame_polygons: List[List[np.ndarray]] = []
        dense_points: Optional[np.ndarray] = None
        dense_points_eval = self._project_mask_points(
            mask_image, fx, fy, cx, cy, origin, R_target_camera, n, d
        )

        world_frame_polygons: List[np.ndarray] = []

        if self._projection_mode == "dense":
            dense_points = dense_points_eval
            if dense_points is None or dense_points.size == 0:
                self._publish_marker_delete(mask_msg.header.stamp)
                return
        else:
            frame_polygons = self._project_mask_to_world(
                mask_image,
                fx,
                fy,
                cx,
                cy,
                origin,
                R_target_camera,
                n,
                d,
            )
            if not frame_polygons:
                self._publish_marker_delete(mask_msg.header.stamp)
                return
            for poly in frame_polygons:
                arr = np.asarray(poly, dtype=np.float64)
                if arr.ndim != 2 or arr.shape[0] < 3:
                    continue
                world_frame_polygons.append(arr)

        polygons_world = world_frame_polygons

        self._publish_markers(mask_msg.header.stamp, polygons=polygons_world, dense_points=dense_points)
        self._publish_projected_points(mask_msg.header, dense_points_eval, used_cluster_id)

        stamp_msg = mask_time.to_msg()
        tf_status = "ok" if tf_ok else "ng"
        if self._projection_mode == "dense":
            points_count = dense_points.shape[0] if dense_points is not None else 0
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, dense_points={points_count}, "
                f"cluster={used_cluster_id if used_cluster_id is not None else 'none'})"
            )
        else:
            poly_count = len(polygons_world)
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, polygons={poly_count}, "
                f"cluster={used_cluster_id if used_cluster_id is not None else 'none'})"
            )

    def _project_mask_to_world(
        self,
        mask_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        origin_target: np.ndarray,
        R_target_camera: np.ndarray,
        plane_normal: np.ndarray,
        plane_offset: float,
    ) -> List[List[np.ndarray]]:
        """Project mask contours onto the mirror plane and return polygons in target frame."""
        _, binary = cv2.threshold(mask_image, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        polygons: List[List[np.ndarray]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._min_mask_area:
                continue

            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) < 3:
                continue

            polygon_points: List[np.ndarray] = []
            for pt in approx:
                u = float(pt[0][0])
                v = float(pt[0][1])
                dir_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
                dir_target = R_target_camera @ dir_camera
                denom = float(np.dot(plane_normal, dir_target))
                if abs(denom) < self._projection_min_abs_denom:
                    continue
                t = -(float(np.dot(plane_normal, origin_target)) + plane_offset) / denom
                if t <= 0.0:
                    continue
                if self._max_projection_distance > 0.0 and t > self._max_projection_distance:
                    continue
                point_world = origin_target + dir_target * t
                polygon_points.append(point_world)

            if len(polygon_points) >= 3:
                polygons.append(polygon_points)

        return polygons

    def _project_mask_points(
        self,
        mask_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        origin_target: np.ndarray,
        R_target_camera: np.ndarray,
        plane_normal: np.ndarray,
        plane_offset: float,
    ) -> Optional[np.ndarray]:
        """Project mask pixels (with stride) directly onto the plane."""
        _, binary = cv2.threshold(mask_image, 127, 255, cv2.THRESH_BINARY)
        ys, xs = np.nonzero(binary)
        if ys.size == 0:
            return None

        if self._pixel_stride > 1:
            stride_mask = ((ys % self._pixel_stride) == 0) & ((xs % self._pixel_stride) == 0)
            ys = ys[stride_mask]
            xs = xs[stride_mask]
            if ys.size == 0:
                return None

        dirs_camera = np.stack(
            ((xs - cx) / fx, (ys - cy) / fy, np.ones_like(xs, dtype=np.float64)), axis=1
        )
        dir_target = dirs_camera @ R_target_camera.T
        denom = dir_target @ plane_normal
        origin_dot = float(plane_normal @ origin_target) + plane_offset
        valid = np.abs(denom) > self._projection_min_abs_denom
        if not np.any(valid):
            return None
        denom = denom[valid]
        dir_target = dir_target[valid]
        t = -origin_dot / denom
        valid = t > 0.0
        if self._max_projection_distance > 0.0:
            valid = valid & (t <= self._max_projection_distance)
        if not np.any(valid):
            return None
        dir_target = dir_target[valid]
        t = t[valid]
        points = origin_target + dir_target * t[:, None]
        return points

    def _publish_markers(
        self,
        stamp,
        polygons: Optional[List[np.ndarray]] = None,
        dense_points: Optional[np.ndarray] = None,
    ) -> None:
        marker_array = MarkerArray()
        marker_id = self._persistent_marker_id
        if polygons:
            for polygon in polygons:
                points_arr = np.asarray(polygon, dtype=np.float64)
                if points_arr.ndim != 2 or points_arr.shape[0] < 2:
                    continue
                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = stamp
                marker.ns = "mirror_surface_projector"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = self.marker_line_width
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.9

                for point in points_arr:
                    marker.points.append(
                        Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                    )
                if points_arr.shape[0] > 0:
                    first = points_arr[0]
                    marker.points.append(
                        Point(x=float(first[0]), y=float(first[1]), z=float(first[2]))
                    )
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0
                marker_array.markers.append(marker)
                marker_id += 1

        if dense_points is not None and dense_points.size > 0:
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = stamp
            marker.ns = "mirror_surface_projector_dense"
            marker.id = marker_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = self._point_scale
            marker.scale.y = self._point_scale
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.6
            for point in dense_points:
                marker.points.append(
                    Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                )
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array.markers.append(marker)

        self._persistent_marker_id = marker_id
        self.marker_pub.publish(marker_array)

    def _publish_projected_points(
        self, header, points: Optional[np.ndarray], cluster_id: Optional[int]
    ) -> None:
        if points is None or points.size == 0:
            return
        stamp_header = Header()
        stamp_header.stamp = header.stamp
        stamp_header.frame_id = self.target_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="cluster_id", offset=12, datatype=PointField.INT32, count=1),
        ]
        cid = int(cluster_id) if cluster_id is not None else -1
        cloud_points = [(float(p[0]), float(p[1]), float(p[2]), cid) for p in points]
        cloud = pc2.create_cloud(stamp_header, fields, cloud_points)
        self.points_pub.publish(cloud)

    def _publish_marker_delete(self, stamp) -> None:
        delete_all = Marker()
        delete_all.header.frame_id = self.target_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_all)
        self.marker_pub.publish(marker_array)

    def _log_throttled_warning(self, attr_name: str, interval_sec: float, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        last_sec = getattr(self, attr_name, -float("inf"))
        if (now_sec - last_sec) >= interval_sec:
            self.get_logger().warn(message)
            setattr(self, attr_name, now_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MirrorSurfaceProjectorNode()
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
