#!/usr/bin/env python3
import math
import csv
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener, TransformException

class AccuracyEvaluator(Node):
    def __init__(self):
        super().__init__('accuracy_evaluator')

        # --- 設定 ---
        # 鏡の四隅に貼られたタグID (左上:1, 右上:2, 左下:3, 右下:4)
        self.declare_parameter('gt_tag_ids', [1, 2, 3, 4])
        # AprilTagのTFフレーム名の接頭辞 (例: landmark_1 なら 'landmark_')
        self.declare_parameter('gt_tag_frame_prefix', 'landmark_') 
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('output_csv', 'evaluation_result.csv')
        self.declare_parameter('mirror_side_length_m', 0.6)
        self.declare_parameter('signed_angle_axis', [0.0, -1.0, 0.0])

        self.gt_ids = self.get_parameter('gt_tag_ids').value
        self.prefix = self.get_parameter('gt_tag_frame_prefix').value
        self.cam_frame = self.get_parameter('camera_frame').value
        self.csv_path = self.get_parameter('output_csv').value
        self.mirror_side = float(self.get_parameter('mirror_side_length_m').value)
        axis_param = self.get_parameter('signed_angle_axis').value
        self.signed_angle_axis = np.array(axis_param, dtype=float)
        if self.signed_angle_axis.shape != (3,) or np.linalg.norm(self.signed_angle_axis) < 1e-9:
            self.get_logger().warning(
                "signed_angle_axis must be a non-zero 3D vector; using default [0.0, -1.0, 0.0]."
            )
            self.signed_angle_axis = np.array([0.0, -1.0, 0.0], dtype=float)
        self.signed_angle_axis /= np.linalg.norm(self.signed_angle_axis)
        self.use_known_geometry = len(self.gt_ids) <= 2
        if self.use_known_geometry and not set(self.gt_ids).issubset({1, 2}):
            self.get_logger().warning(
                "Known-geometry mode only supports tag IDs 1 and 2; falling back to averaging visible tags."
            )
            self.use_known_geometry = False

        # --- TF & Sub ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 推定結果の購読
        self.sub = self.create_subscription(
            Float64MultiArray,
            'mirror_plane_cam',
            self.callback,
            10
        )

        # CSVの準備（ヘッダー書き込み）
        default_header = [
            'timestamp',
            'dist_error_mm',
            'dist_error_signed_mm',
            'angle_error_deg',
            'angle_error_signed_deg',
            'est_dist_param_m',
            'gt_center_dist_m',
            'visible_gt_tags' # そのフレームで計算に使えた真値タグの数
        ]
        fallback_header = [
            'timestamp',
            'dist_error_mm',
            'angle_error_deg',
            'est_dist_param_m',
            'gt_center_dist_m',
            'visible_gt_tags'
        ]
        csv_needs_header = (not os.path.exists(self.csv_path)) or os.path.getsize(self.csv_path) == 0
        if csv_needs_header:
            self.csv_header = default_header
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_header)
        else:
            try:
                with open(self.csv_path, 'r') as f:
                    reader = csv.reader(f)
                    header = next(reader, None)
                if header:
                    self.csv_header = header
                else:
                    self.csv_header = fallback_header
                    self.get_logger().warning(
                        "Existing CSV has no header; falling back to unsigned logging. "
                        "Use a new output_csv to enable signed logging."
                    )
            except Exception:
                self.csv_header = fallback_header
                self.get_logger().warning(
                    "Failed to read CSV header; falling back to unsigned logging. "
                    "Use a new output_csv to enable signed logging."
                )
        missing_signed = []
        if 'dist_error_signed_mm' not in self.csv_header:
            missing_signed.append('dist_error_signed_mm')
        if 'angle_error_signed_deg' not in self.csv_header:
            missing_signed.append('angle_error_signed_deg')
        if missing_signed:
            missing = ", ".join(missing_signed)
            self.get_logger().warning(
                f"Existing CSV has no {missing} column(s); signed error will not be logged. "
                "Use a new output_csv to enable signed logging."
            )
        
        self.get_logger().info(f"Evaluation started for Tags {self.gt_ids}. Saving to {self.csv_path}")
        if self.use_known_geometry:
            self.get_logger().info(
                f"Using known mirror geometry (side={self.mirror_side:.3f}m) for GT center."
            )

    def callback(self, msg):
        # 1. 推定平面の取得 [nx, ny, nz, d]
        n_est = np.array([msg.data[0], msg.data[1], msg.data[2]])
        d_est = msg.data[3]

        # 2. 真値（GT）の取得 (TFからタグ位置を引く)
        gt_points = []
        gt_normals = []
        gt_tag_data = {}
        visible_count = 0
        
        for tid in self.gt_ids:
            # フレーム名 (例: tag_36h11_1)
            tag_frame = f"{self.prefix}{tid}" 
            try:
                # 最新のTFを取得
                t = self.tf_buffer.lookup_transform(
                    self.cam_frame, tag_frame, rclpy.time.Time())
                
                # 位置
                p = np.array([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ])
                gt_points.append(p)

                # 法線 (タグ座標系のZ軸が法線であると仮定)
                q = [
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                ]
                R = self.quat_to_rot(q)
                normal = R[:, 2] # Z axis
                gt_normals.append(normal)
                gt_tag_data[tid] = {"p": p, "R": R}
                visible_count += 1

            except TransformException:
                pass # 見えていないタグはスキップ

        # 真値タグが1つも見えていない場合は評価できないのでスキップ
        if visible_count == 0:
            return 

        # 真値の中心と法線（見えているタグの平均）
        n_gt_mean = np.mean(gt_normals, axis=0)
        n_gt_norm = np.linalg.norm(n_gt_mean)
        if n_gt_norm == 0.0:
            return
        n_gt_mean /= n_gt_norm

        if self.use_known_geometry:
            p_gt_center = self.compute_center_from_known_geometry(gt_tag_data, n_gt_mean)
            if p_gt_center is None:
                return
        else:
            # 4隅の平均を取ることで、鏡の中心座標が得られる
            p_gt_center = np.mean(gt_points, axis=0)

        # 3. 誤差計算
        # 距離誤差 (点と平面の距離): | n_est * p_gt + d_est |
        # 推定された無限平面が、真値の中心点からどれだけ離れているか
        signed_error_m = np.dot(n_est, p_gt_center) + d_est
        n_est_aligned = n_est
        if np.dot(n_est, n_gt_mean) < 0.0:
            signed_error_m *= -1.0
            n_est_aligned = -n_est
        dist_error_m = abs(signed_error_m)
        
        # 角度誤差: acos(|n_est * n_gt|)
        dot = np.clip(np.dot(n_est_aligned, n_gt_mean), -1.0, 1.0)
        angle_error_rad = math.acos(abs(dot)) # 裏向きも許容するためabs
        angle_error_deg = math.degrees(angle_error_rad)
        signed_angle_deg = self.compute_signed_angle_deg(n_est_aligned, n_gt_mean)

        # 参考: カメラから鏡中心までの距離
        gt_dist_from_cam = np.linalg.norm(p_gt_center)

        # 4. 保存
        with open(self.csv_path, 'a') as f:
            writer = csv.writer(f)
            row_values = {
                'timestamp': self.get_clock().now().nanoseconds,
                'dist_error_mm': dist_error_m * 1000.0, # mm変換
                'dist_error_signed_mm': signed_error_m * 1000.0,
                'angle_error_deg': angle_error_deg,
                'angle_error_signed_deg': signed_angle_deg,
                'est_dist_param_m': d_est,
                'gt_center_dist_m': gt_dist_from_cam,
                'visible_gt_tags': visible_count
            }
            row = [row_values.get(name, '') for name in self.csv_header]
            writer.writerow(row)
            
        self.get_logger().info(
            f"Logged: DistErr={dist_error_m*1000:.1f}mm, AngErr={angle_error_deg:.1f}deg (GT tags: {visible_count})"
        )

    def compute_center_from_known_geometry(self, tag_data, n_gt_mean):
        half = self.mirror_side * 0.5
        centers = []

        # Two-tag geometry: use the right edge (id1 upper right, id2 lower right).
        if 1 in tag_data and 2 in tag_data:
            p1 = tag_data[1]["p"]
            p2 = tag_data[2]["p"]
            up_dir = p1 - p2
            up_norm = np.linalg.norm(up_dir)
            if up_norm > 1e-9:
                up_dir /= up_norm
                right_dir = np.cross(up_dir, n_gt_mean)
                right_norm = np.linalg.norm(right_dir)
                if right_norm > 1e-9:
                    right_dir /= right_norm
                    avg_right = tag_data[1]["R"][:, 0] + tag_data[2]["R"][:, 0]
                    avg_right_norm = np.linalg.norm(avg_right)
                    if avg_right_norm > 1e-9 and np.dot(right_dir, avg_right) < 0.0:
                        right_dir *= -1.0
                    right_edge_mid = (p1 + p2) * 0.5
                    centers.append(right_edge_mid - right_dir * half)

        # Fallback: single-tag geometry using tag axes (x: right, y: down, z: out).
        if not centers:
            for tid in (1, 2):
                if tid not in tag_data:
                    continue
                p = tag_data[tid]["p"]
                R = tag_data[tid]["R"]
                right_dir = R[:, 0]
                up_dir = -R[:, 1]
                if tid == 1:
                    centers.append(p - right_dir * half - up_dir * half)
                else:
                    centers.append(p - right_dir * half + up_dir * half)

        if not centers:
            return None
        return np.mean(centers, axis=0)

    def compute_signed_angle_deg(self, n_est, n_gt_mean):
        axis = self.signed_angle_axis
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            return float('nan')
        axis = axis / axis_norm

        n_est_proj = n_est - axis * float(n_est @ axis)
        n_gt_proj = n_gt_mean - axis * float(n_gt_mean @ axis)
        n_est_norm = np.linalg.norm(n_est_proj)
        n_gt_norm = np.linalg.norm(n_gt_proj)
        if n_est_norm < 1e-9 or n_gt_norm < 1e-9:
            return float('nan')

        n_est_proj /= n_est_norm
        n_gt_proj /= n_gt_norm

        dot = np.clip(float(n_gt_proj @ n_est_proj), -1.0, 1.0)
        cross = np.cross(n_gt_proj, n_est_proj)
        angle_rad = math.atan2(np.linalg.norm(cross), dot)
        sign = float(cross @ axis)
        if abs(sign) < 1e-9:
            sign = 1.0
        else:
            sign = 1.0 if sign > 0.0 else -1.0
        return math.degrees(angle_rad) * sign

    def quat_to_rot(self, q):
        x, y, z, w = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
        ])

def main():
    rclpy.init()
    node = AccuracyEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
