#!/usr/bin/env python3  # ROS 2 用の Python 実行エントリ

from typing import List, Optional, Tuple  # 型ヒント用の Optional / Tuple / List をインポート

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

        self.bridge = CvBridge()  # Image ⇔ OpenCV 変換のためのブリッジを作成
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))  # TF を一定期間キャッシュするバッファ
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # バッファを更新する TF リスナー

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)  # 投影結果の MarkerArray 出力先

        self._latest_plane_msg: Optional[Tuple[Float64MultiArray, Time]] = None  # 直近の平面メッセージと受信時刻
        self._latest_synced_stamp: Optional[Time] = None  # 直近の同期済みスタンプを記録
        self._last_tf_warn_sec: float = -float("inf")  # TF 警告のレート制限用タイムスタンプ
        self._last_plane_warn_sec: float = -float("inf")  # 平面未受信警告のレート制限用タイムスタンプ

        # Synchronize mask and camera info
        self.mask_sub = Subscriber(self, Image, mask_topic)  # マスク画像を購読するサブスクライバ
        self.camera_info_sub = Subscriber(self, CameraInfo, camera_info_topic)  # CameraInfo を購読するサブスクライバ
        self.sync = ApproximateTimeSynchronizer(  # 同期オブジェクトを生成
            [self.mask_sub, self.camera_info_sub], queue_size=queue_size, slop=slop
        )
        self.sync.registerCallback(self._synced_callback)  # 同期データがそろったときのコールバックを登録

        # Mirror plane observations arrive asynchronously
        self.create_subscription(Float64MultiArray, plane_topic, self._plane_callback, 10)  # 平面観測を購読するサブスクライバを作成

        self.get_logger().info(  # 初期化内容をログ出力
            "MirrorSurfaceProjectorNode ready "
            f"(mask_topic={mask_topic}, camera_info_topic={camera_info_topic}, plane_topic={plane_topic})"
        )

    def _plane_callback(self, msg: Float64MultiArray) -> None:
        self._latest_plane_msg = (msg, self.get_clock().now())  # 受信した平面データと時刻を保存

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

        plane_available = self._latest_plane_msg is not None  # 平面データがあるかを確認
        if not plane_available:
            self._log_throttled_warning(  # 平面データ未受信を一定間隔で警告
                "_last_plane_warn_sec",
                5.0,
                "No mirror plane data received yet; waiting before projection.",
            )
            self._publish_marker_delete(mask_msg.header.stamp)
            return

        plane_msg, plane_stamp = self._latest_plane_msg  # 保存している平面データを取り出す
        age_sec = (mask_time.nanoseconds - plane_stamp.nanoseconds) / 1e9  # 平面データの鮮度を秒に換算
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

        polygons: Optional[List[List[np.ndarray]]] = None
        dense_points: Optional[np.ndarray] = None
        if self._projection_mode == "dense":
            dense_points = self._project_mask_points(
                mask_image, fx, fy, cx, cy, origin, R_target_camera, n, d
            )
            if dense_points is None or dense_points.size == 0:
                self._publish_marker_delete(mask_msg.header.stamp)
                return
        else:
            polygons = self._project_mask_to_world(
                mask_image, fx, fy, cx, cy, origin, R_target_camera, n, d
            )
            if not polygons:
                self._publish_marker_delete(mask_msg.header.stamp)
                return

        self._publish_markers(mask_msg.header.stamp, polygons=polygons, dense_points=dense_points)

        stamp_msg = mask_time.to_msg()
        tf_status = "ok" if tf_ok else "ng"
        if self._projection_mode == "dense":
            points_count = dense_points.shape[0] if dense_points is not None else 0
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, dense_points={points_count})"
            )
        else:
            poly_count = len(polygons) if polygons is not None else 0
            self.get_logger().debug(
                f"Synced inputs at stamp {stamp_msg} (plane=yes, tf={tf_status}, polygons={poly_count})"
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
        marker_array = MarkerArray()  # MarkerArray を構築

        delete_all = Marker()
        delete_all.header.frame_id = self.target_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL  # 既存マーカーを消去
        marker_array.markers.append(delete_all)

        marker_id = 0
        if polygons:
            for polygon in polygons:
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

                for point in polygon:
                    marker.points.append(
                        Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                    )
                if polygon:
                    first = polygon[0]
                    marker.points.append(
                        Point(x=float(first[0]), y=float(first[1]), z=float(first[2]))
                    )  # 終点を始点に戻して閉ループに

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
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

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
