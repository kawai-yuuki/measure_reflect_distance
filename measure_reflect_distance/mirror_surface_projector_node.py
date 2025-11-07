#!/usr/bin/env python3  # ROS 2 用の Python 実行エントリ

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple  # 型ヒント用の Optional / Tuple / List をインポート

import cv2  # OpenCV（輪郭抽出に使用）
import numpy as np  # 数値計算ライブラリ
import rclpy  # ROS 2 Python クライアントライブラリ
from cv_bridge import CvBridge, CvBridgeError  # ROS Image と OpenCV 画像の相互変換
from geometry_msgs.msg import Point  # Marker へ渡す 3D 座標
from message_filters import ApproximateTimeSynchronizer, Subscriber  # メッセージ同期ユーティリティ
from rclpy.duration import Duration  # 時間長さの表現
from rclpy.node import Node  # ROS 2 ノード基底クラス
from rclpy.time import Time  # ROS Time 型のユーティリティ
from sensor_msgs.msg import CameraInfo, Image  # カメラ情報と画像メッセージ型
from std_msgs.msg import Float64MultiArray  # 平面データが載る汎用配列メッセージ
import tf2_ros  # TF2 の Python ラッパー
from visualization_msgs.msg import Marker, MarkerArray  # 可視化用 Marker

from measure_reflect_distance.util.mirror_geometry import rot_from_quat  # クォータニオン→回転行列


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


@dataclass
class StoredPolygon:
    camera_frame: str
    local_points: np.ndarray
    stamp: Time



class MirrorSurfaceProjectorNode(Node):
    """Collect synchronized mask, camera intrinsics, and mirror plane information."""  # ノードの目的を説明する docstring

    def __init__(self) -> None:
        super().__init__("mirror_surface_projector_node")  # ノード名を指定して初期化

        # Declare configurable topics/frames and synchronizer behavior
        self.declare_parameter("mask_topic", "/mask_image_processed")  # マスク画像トピック名
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")  # CameraInfo トピック名
        self.declare_parameter("plane_topic", "mirror_plane")  # 鏡面平面情報トピック名
        self.declare_parameter("target_frame", "map")  # 投影先座標系のフレーム名
        self.declare_parameter("default_camera_frame", "camera_color_optical_frame")  # CameraInfo が空のときのフォールバックフレーム
        self.declare_parameter("sync_queue_size", 10)  # 同期キューの長さ
        self.declare_parameter("sync_slop", 0.05)  # 許容タイムスタンプ差（秒）
        self.declare_parameter("plane_max_age_sec", 0.5)  # 平面データ許容遅延
        self.declare_parameter("marker_topic", "mirror_surface_projected_markers")  # Marker の出力トピック
        self.declare_parameter("marker_line_width", 0.01)  # Marker の線幅
        self.declare_parameter("min_mask_area", 200.0)  # 輪郭として扱う最小マスク面積 [pixel]
        self.declare_parameter("tf_timeout_sec", 0.1)  # TF 取得の待機時間
        self.declare_parameter("projection_mode", "contour")  # contour / dense
        self.declare_parameter("pixel_stride", 2)  # dense 投影時の間引き間隔
        self.declare_parameter("point_scale", 0.03)  # dense 投影時の Marker.POINTS サイズ
        self.declare_parameter("max_fps", 5.0)  # 投影処理の最大周波数
        self.declare_parameter("clustered_observation_topic", "mirror_plane_clustered_observation")
        self.declare_parameter("cluster_plane_topic", "mirror_plane_clustered")
        self.declare_parameter("grid_resolution", 0.01)  # [m] accumulation grid resolution
        self.declare_parameter("grid_update_period_sec", 1.0)
        self.declare_parameter("grid_density_threshold", 0.2)

        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value  # パラメータからマスクトピックを取得
        camera_info_topic = (  # CameraInfo トピック名を取得
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        plane_topic = self.get_parameter("plane_topic").get_parameter_value().string_value  # 平面トピック名を取得
        self.target_frame = (  # 投影先フレーム名を保持
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.default_camera_frame = (  # CameraInfo が持つべきフレーム名の既定値
            self.get_parameter("default_camera_frame").get_parameter_value().string_value
        )
        queue_size = max(  # 同期用キューサイズを 1 以上に制限
            1, self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        )
        slop = max(  # 同期許容差を 0 以上に制限
            0.0, self.get_parameter("sync_slop").get_parameter_value().double_value
        )
        self._plane_max_age_sec = max(  # 平面データ許容遅延
            0.0, self.get_parameter("plane_max_age_sec").get_parameter_value().double_value
        )
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value  # Marker 出力トピック
        self.marker_line_width = max(  # Marker の線幅を保存
            0.001,
            self.get_parameter("marker_line_width").get_parameter_value().double_value,
        )
        self._min_mask_area = max(  # 取り扱う最小マスク面積
            0.0, self.get_parameter("min_mask_area").get_parameter_value().double_value
        )
        self._tf_timeout = max(
            0.0, self.get_parameter("tf_timeout_sec").get_parameter_value().double_value
        )
        self._max_fps = float(self.get_parameter("max_fps").get_parameter_value().double_value)
        if self._max_fps <= 0.0:
            self._min_interval_ns = 0
        else:
            self._min_interval_ns = int(1e9 / self._max_fps)
        self._last_process_time = None
        self._projection_mode = self.get_parameter("projection_mode").get_parameter_value().string_value.lower()
        if self._projection_mode not in ("contour", "dense"):
            self.get_logger().warn(
                f"Unknown projection_mode '{self._projection_mode}', fallback to 'contour'."
            )
            self._projection_mode = "contour"
        self._pixel_stride = max(
            1, int(self.get_parameter("pixel_stride").get_parameter_value().integer_value)
        )
        self._point_scale = max(
            0.001, self.get_parameter("point_scale").get_parameter_value().double_value
        )
        self._clustered_observation_topic = (
            self.get_parameter("clustered_observation_topic").get_parameter_value().string_value
        )
        self._cluster_plane_topic = (
            self.get_parameter("cluster_plane_topic").get_parameter_value().string_value
        )
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

        self.bridge = CvBridge()  # Image ⇔ OpenCV 変換のためのブリッジを作成
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))  # TF を一定期間キャッシュするバッファ
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # バッファを更新する TF リスナー

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)  # 投影結果の MarkerArray 出力先

        self._latest_plane_msg: Optional[Tuple[Float64MultiArray, Time]] = None  # 直近の平面メッセージと受信時刻
        self._latest_synced_stamp: Optional[Time] = None  # 直近の同期済みスタンプを記録
        self._last_tf_warn_sec: float = -float("inf")  # TF 警告のレート制限用タイムスタンプ
        self._last_plane_warn_sec: float = -float("inf")  # 平面未受信警告のレート制限用タイムスタンプ
        self._latest_cluster_observation: Optional[Tuple[int, Float64MultiArray, Time]] = None
        self._cluster_planes: Dict[int, Tuple[Float64MultiArray, Time]] = {}
        self._cluster_grids: Dict[int, ClusterGrid] = {}
        self._cluster_polygons: Dict[int, List[StoredPolygon]] = {}
        self._orphan_polygons: List[StoredPolygon] = []
        self._persistent_marker_id: int = 0

        # Synchronize mask and camera info
        self.mask_sub = Subscriber(self, Image, mask_topic)  # マスク画像を購読するサブスクライバ
        self.camera_info_sub = Subscriber(self, CameraInfo, camera_info_topic)  # CameraInfo を購読するサブスクライバ
        self.sync = ApproximateTimeSynchronizer(  # 同期オブジェクトを生成
            [self.mask_sub, self.camera_info_sub], queue_size=queue_size, slop=slop
        )
        self.sync.registerCallback(self._synced_callback)  # 同期データがそろったときのコールバックを登録

        # Mirror plane observations arrive asynchronously
        self.create_subscription(Float64MultiArray, plane_topic, self._plane_callback, 10)  # 平面観測を購読するサブスクライバを作成
        self.create_subscription(
            Float64MultiArray,
            self._clustered_observation_topic,
            self._cluster_observation_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self._cluster_plane_topic,
            self._cluster_plane_callback,
            10,
        )
        self._grid_timer = self.create_timer(self._grid_update_period, self._process_grids)

        self.get_logger().info(  # 初期化内容をログ出力
            "MirrorSurfaceProjectorNode ready "
            f"(mask_topic={mask_topic}, camera_info_topic={camera_info_topic}, plane_topic={plane_topic})"
        )

    def _plane_callback(self, msg: Float64MultiArray) -> None:
        self._latest_plane_msg = (msg, self.get_clock().now())  # 受信した平面データと時刻を保存

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
        normal = np.array(data[1:4], dtype=np.float64)
        distance = float(data[4])
        point = np.array(data[5:8], dtype=np.float64)
        if len(data) >= 11:
            tangent = np.array(data[8:11], dtype=np.float64)
        else:
            tangent = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        norm_n = np.linalg.norm(normal)
        if norm_n > 1e-6:
            normal = normal / norm_n
        tangent = tangent - float(tangent @ normal) * normal
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

        trimmed = list(data[1:])
        if len(trimmed) >= 11:
            trimmed = trimmed[:-1]  # drop support count
        plane_msg = Float64MultiArray()
        plane_msg.data = trimmed
        self._cluster_planes[cluster_id] = (plane_msg, self.get_clock().now())

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

    def _synced_callback(self, mask_msg: Image, camera_info_msg: CameraInfo) -> None:
        mask_time = Time.from_msg(mask_msg.header.stamp)  # マスクメッセージのタイムスタンプを Time に変換
        camera_frame = camera_info_msg.header.frame_id or self.default_camera_frame  # CameraInfo のフレーム名を取得（空なら既定値）
        self._latest_synced_stamp = mask_time  # 同期時刻を記録

        if self._min_interval_ns > 0:
            now = self.get_clock().now()
            if self._last_process_time is not None:
                elapsed_ns = (now - self._last_process_time).nanoseconds
                if elapsed_ns < self._min_interval_ns:
                    return
            self._last_process_time = now

        plane_msg = None
        plane_stamp = None
        plane_available = False
        used_cluster_id: Optional[int] = None
        plane_source = "raw"

        if self._latest_cluster_observation is not None:
            obs_cluster_id, obs_msg, obs_time = self._latest_cluster_observation
            age_obs = max(0.0, (mask_time.nanoseconds - obs_time.nanoseconds) / 1e9)
            if self._plane_max_age_sec <= 0.0 or age_obs <= self._plane_max_age_sec:
                plane_entry = self._cluster_planes.get(obs_cluster_id)
                if plane_entry is not None:
                    plane_msg, plane_stamp = plane_entry
                    plane_available = True
                    used_cluster_id = obs_cluster_id
                    plane_source = "cluster"
                else:
                    plane_msg = obs_msg
                    plane_stamp = obs_time
                    plane_available = True
                    used_cluster_id = obs_cluster_id
                    plane_source = "observation"
            else:
                self.get_logger().debug(
                    f"Latest cluster observation (id={obs_cluster_id}) too old ({age_obs:.2f}s); ignoring."
                )

        if not plane_available and self._latest_plane_msg is not None:
            plane_msg, plane_stamp = self._latest_plane_msg
            plane_available = True

        if not plane_available:
            self._log_throttled_warning(  # 平面データ未受信を一定間隔で警告
                "_last_plane_warn_sec",
                5.0,
                "No mirror plane data received yet; waiting before projection.",
            )
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        age_sec = 0.0
        if isinstance(plane_stamp, Time):
            age_sec = (mask_time.nanoseconds - plane_stamp.nanoseconds) / 1e9  # 平面データの鮮度を秒に換算
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

        tf_ok = False  # TF 取得成否フラグ
        try:
            transform = self.tf_buffer.lookup_transform(  # ターゲットフレームからカメラフレームへの TF を取得
                self.target_frame,
                camera_frame,
                mask_time,
                timeout=Duration(seconds=self._tf_timeout),
            )
            tf_ok = True  # TF 取得成功を記録
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
            mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")  # マスク画像を OpenCV 形式に変換して検証
        except CvBridgeError as err:
            self.get_logger().error(f"Failed to convert mask image: {err}")  # 変換失敗をエラーログ
            self._publish_marker_delete(mask_msg.header.stamp)
            return  # 処理を中断

        fx = camera_info_msg.k[0]  # カメラ行列の fx
        fy = camera_info_msg.k[4]  # カメラ行列の fy
        cx = camera_info_msg.k[2]  # 画像中心 x
        cy = camera_info_msg.k[5]  # 画像中心 y
        if fx == 0.0 or fy == 0.0:
            self.get_logger().error("Camera intrinsics invalid (fx or fy is zero).")
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        plane_data = np.array(plane_msg.data, dtype=np.float64)  # 平面データを numpy 配列化
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

        translation = transform.transform.translation  # カメラ原点のターゲット座標
        rotation = transform.transform.rotation  # カメラ姿勢（クォータニオン）
        origin = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        R_target_camera = rot_from_quat(rotation.x, rotation.y, rotation.z, rotation.w)  # 回転行列へ変換

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
            world_frame_polygons = []
            for poly in frame_polygons:
                arr = np.asarray(poly, dtype=np.float64)
                if arr.ndim != 2 or arr.shape[0] < 3:
                    continue
                local = (arr - origin) @ R_target_camera.T
                self._store_polygon(used_cluster_id, camera_frame, local, mask_time)
                world_frame_polygons.append(arr)
        polygons_world = self._prepare_polygons_for_render(world_frame_polygons)

        self._publish_markers(mask_msg.header.stamp, polygons=polygons_world, dense_points=dense_points)

        if used_cluster_id is not None and dense_points_eval is not None and dense_points_eval.size > 0:
            self._accumulate_cluster_points(used_cluster_id, dense_points_eval)

        stamp_msg = mask_time.to_msg()
        tf_status = "ok" if tf_ok else "ng"
        if self._projection_mode == "dense":
            points_count = dense_points.shape[0] if dense_points is not None else 0
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, dense_points={points_count})"
            )
        else:
            poly_count = len(polygons_world)
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, polygons={poly_count}, "
                f"source={plane_source}, cluster={used_cluster_id if used_cluster_id is not None else 'none'})"
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
        _, binary = cv2.threshold(mask_image, 127, 255, cv2.THRESH_BINARY)  # マスクを二値化
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 外枠の輪郭を抽出

        polygons: List[List[np.ndarray]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._min_mask_area:
                continue  # 小さいノイズは無視

            epsilon = 0.02 * cv2.arcLength(contour, True)  # 輪郭を簡略化
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) < 3:
                continue  # 三角形未満は平面領域として扱えない

            polygon_points: List[np.ndarray] = []
            for pt in approx:
                u = float(pt[0][0])  # 画像 x 座標
                v = float(pt[0][1])  # 画像 y 座標
                dir_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)  # カメラ座標でのレイ方向
                dir_target = R_target_camera @ dir_camera  # ターゲット座標へ回転
                denom = float(np.dot(plane_normal, dir_target))
                if abs(denom) < 1e-9:
                    continue  # 平面とレイがほぼ平行
                t = - (float(np.dot(plane_normal, origin_target)) + plane_offset) / denom
                if t <= 0.0:
                    continue  # 平面がレイの前方に存在しない
                point_world = origin_target + dir_target * t  # 交点を算出
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
        valid = np.abs(denom) > 1e-9
        if not np.any(valid):
            return None
        denom = denom[valid]
        dir_target = dir_target[valid]
        t = -origin_dot / denom
        valid = t > 0.0
        if not np.any(valid):
            return None
        dir_target = dir_target[valid]
        t = t[valid]
        points = origin_target + dir_target * t[:, None]
        return points

    def _publish_markers(
        self,
        stamp,
        polygons: Optional[List[List[np.ndarray]]] = None,
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
                    )  # 終点を始点に戻して閉ループに
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

    def _store_polygon(
        self,
        cluster_id: Optional[int],
        camera_frame: str,
        local_points: np.ndarray,
        stamp: Time,
    ) -> None:
        if local_points.size == 0:
            return
        stored = StoredPolygon(
            camera_frame=camera_frame,
            local_points=local_points.astype(np.float64),
            stamp=stamp,
        )
        if cluster_id is None:
            self._orphan_polygons.append(stored)
        else:
            self._cluster_polygons.setdefault(cluster_id, []).append(stored)

    def _prepare_polygons_for_render(
        self, frame_polygons: List[np.ndarray]
    ) -> List[np.ndarray]:
        polygons: List[np.ndarray] = [poly.copy() for poly in frame_polygons]
        for stored in self._iter_stored_polygons():
            world = self._transform_polygon_to_world(stored)
            if world is not None and world.shape[0] >= 2:
                polygons.append(world)
        return polygons

    def _iter_stored_polygons(self):
        for stored_list in self._cluster_polygons.values():
            for stored in stored_list:
                yield stored
        for stored in self._orphan_polygons:
            yield stored

    def _transform_polygon_to_world(self, stored: StoredPolygon) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                stored.camera_frame,
                stored.stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except tf2_ros.ExtrapolationException:
            latest_time = Time()
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    stored.camera_frame,
                    latest_time,
                    timeout=Duration(seconds=self._tf_timeout),
                )
            except Exception:
                return None
        except Exception:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        origin = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        R_target_camera = rot_from_quat(rotation.x, rotation.y, rotation.z, rotation.w)

        local = stored.local_points
        world = local @ R_target_camera.T + origin
        return world

    def _publish_marker_delete(self, stamp) -> None:
        delete_all = Marker()
        delete_all.header.frame_id = self.target_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_all)
        self.marker_pub.publish(marker_array)

    def _log_throttled_warning(self, attr_name: str, interval_sec: float, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9  # 現在時刻を秒に変換
        last_sec = getattr(self, attr_name, -float("inf"))  # 前回警告時刻を取得
        if (now_sec - last_sec) >= interval_sec:  # インターバルを超えているか確認
            self.get_logger().warn(message)  # 警告を出力
            setattr(self, attr_name, now_sec)  # 最新の警告時刻を記録

    def _accumulate_cluster_points(self, cluster_id: int, points: np.ndarray) -> None:
        grid = self._cluster_grids.get(cluster_id)
        if grid is None:
            self.get_logger().debug(
                f"No grid initialized for cluster_id={cluster_id}; skipping accumulation."
            )
            return
        grid.integrate_points(points)
        grid.last_update = self.get_clock().now().nanoseconds / 1e9

    def _process_grids(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for cluster_id, grid in list(self._cluster_grids.items()):
            if grid.total_points < 10:
                continue
            density = grid.as_density_map()
            if density is None or density.size == 0:
                continue
            threshold = max(self._grid_density_threshold, 1e-3)
            _, binary = cv2.threshold(density, threshold, 1.0, cv2.THRESH_BINARY)
            binary_uint8 = (binary * 255).astype(np.uint8)
            contours, _ = cv2.findContours(binary_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            polygons: List[np.ndarray] = []
            for contour in contours:
                if contour.shape[0] < 3:
                    continue
                world_points: List[np.ndarray] = []
                for pt in contour:
                    ci = int(pt[0][0]) + grid.min_i
                    cj = int(pt[0][1]) + grid.min_j
                    world_points.append(grid.grid_to_world(ci, cj))
                if len(world_points) >= 3:
                    polygons.append(np.array(world_points, dtype=np.float64))
            if polygons:
                grid.polygon_cache = polygons
                self._cluster_polygons[cluster_id] = polygons
            elif cluster_id in self._cluster_polygons:
                del self._cluster_polygons[cluster_id]


def main(args=None) -> None:
    rclpy.init(args=args)  # ROS 2 を初期化
    node = MirrorSurfaceProjectorNode()  # ノードインスタンスを生成
    try:
        rclpy.spin(node)  # コールバック処理を開始
    finally:
        node.destroy_node()  # 終了時にノードを破棄
        rclpy.shutdown()  # ROS 2 をシャットダウン


if __name__ == "__main__":  # 直接実行された場合のエントリポイント
    main()  # main 関数を呼び出してノードを起動
