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

        self.gt_ids = self.get_parameter('gt_tag_ids').value
        self.prefix = self.get_parameter('gt_tag_frame_prefix').value
        self.cam_frame = self.get_parameter('camera_frame').value
        self.csv_path = self.get_parameter('output_csv').value

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
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 
                    'dist_error_mm', 
                    'angle_error_deg', 
                    'est_dist_param_m', 
                    'gt_center_dist_m',
                    'visible_gt_tags' # そのフレームで計算に使えた真値タグの数
                ])
        
        self.get_logger().info(f"Evaluation started for Tags {self.gt_ids}. Saving to {self.csv_path}")

    def callback(self, msg):
        # 1. 推定平面の取得 [nx, ny, nz, d]
        n_est = np.array([msg.data[0], msg.data[1], msg.data[2]])
        d_est = msg.data[3]

        # 2. 真値（GT）の取得 (TFからタグ位置を引く)
        gt_points = []
        gt_normals = []
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
                visible_count += 1

            except TransformException:
                pass # 見えていないタグはスキップ

        # 真値タグが1つも見えていない場合は評価できないのでスキップ
        if visible_count == 0:
            return 

        # 真値の中心と法線（見えているタグの平均）
        # 4隅の平均を取ることで、鏡の中心座標が得られる
        p_gt_center = np.mean(gt_points, axis=0)
        n_gt_mean = np.mean(gt_normals, axis=0)
        n_gt_mean /= np.linalg.norm(n_gt_mean)

        # 3. 誤差計算
        # 距離誤差 (点と平面の距離): | n_est * p_gt + d_est |
        # 推定された無限平面が、真値の中心点からどれだけ離れているか
        dist_error_m = abs(np.dot(n_est, p_gt_center) + d_est)
        
        # 角度誤差: acos(|n_est * n_gt|)
        dot = np.clip(np.dot(n_est, n_gt_mean), -1.0, 1.0)
        angle_error_rad = math.acos(abs(dot)) # 裏向きも許容するためabs
        angle_error_deg = math.degrees(angle_error_rad)

        # 参考: カメラから鏡中心までの距離
        gt_dist_from_cam = np.linalg.norm(p_gt_center)

        # 4. 保存
        with open(self.csv_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.get_clock().now().nanoseconds,
                dist_error_m * 1000.0, # mm変換
                angle_error_deg,
                d_est, 
                gt_dist_from_cam,
                visible_count
            ])
            
        self.get_logger().info(
            f"Logged: DistErr={dist_error_m*1000:.1f}mm, AngErr={angle_error_deg:.1f}deg (GT tags: {visible_count})"
        )

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