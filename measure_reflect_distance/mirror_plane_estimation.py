#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
鏡像AprilTagから鏡平面（法線ベクトル n と距離 d）を推定して出力する ROS2 ノード。
- 入力: 
  * TF: camera_color_optical_frame <- tag_<ID>（鏡で反射して見える "仮想タグ" の姿勢）
  * パラメータ: 実タグ(カメラ近傍に剛体固定) -> カメラ の外部 T_c<-t
- 出力:
  * /mirror_plane_cam : カメラ座標での [n_x, n_y, n_z, d]
  * /mirror_plane     : output_frame（例: base_link / map）に変換した [n_x, n_y, n_z, d]
  * TF: "mirror_plane_cam" / "mirror_plane"（可視化用。原点=平面の最近点、姿勢=法線向き）
  
理論の要点:
鏡像は「平面反射変換 S」で表現でき、S は 4x4 の同次変換で
    S = [[ R, t ],
         [ 0, 1 ]]
R = I - 2 n n^T,  t = 2 d n
ここで n は単位法線、d は平面方程式 n^T x + d = 0 のオフセット。
実タグ→カメラの外部 T_c<-t と「鏡に映った仮想タグ→カメラ」の外部 T_c<-tv が分かると、
    S = T_c<-tv * (T_c<-t)^(-1)
から S を構成でき、上式の関係から n と d を復元できる（5.4 単一画像推定）。
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

from measure_reflect_distance.util.mirror_geometry import (
    rot_from_rpy, rot_from_quat, quat_from_two_vectors,
    mat4_from_rt, mat4_from_tf, inv_T, plane_from_reflection, transform_plane
)

class MirrorPlaneEstimator(Node):
    def __init__(self):
        super().__init__('mirror_plane_estimator')

        # ---- パラメータ定義 ----
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')  # D455カラー光学フレーム
        self.declare_parameter('output_frame', 'map')  # 出力先フレーム（例: base_link / map）
        self.declare_parameter('tag_frame_name', 'reflected')                     # 鏡像タグのフレーム名（apriltag_ros の出力）
        # 実タグ（カメラ近傍に剛体固定）→ カメラ の外部 T_c<-t
        self.declare_parameter('t_ct_xyz', [0.017545,-0.080829,-0.021476])  # [m]
        self.declare_parameter('t_ct_rpy', [-0.023080,0.001224,-3.131105])  # [rad] roll, pitch, yaw
        self.declare_parameter('publish_tf', True)  # 可視化TFを出すか

        # ---- パラメータ取得 ----
        self.cam_frame = self.get_parameter('camera_frame').value
        self.out_frame = self.get_parameter('output_frame').value
        self.tag_frame = str(self.get_parameter('tag_frame_name').value)
        t_ct_xyz = np.array(self.get_parameter('t_ct_xyz').value, dtype=float)
        t_ct_rpy = np.array(self.get_parameter('t_ct_rpy').value, dtype=float)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        # 実タグ→カメラ: T_c<-t を4x4に構成
        R_ct = rot_from_rpy(t_ct_rpy[0], t_ct_rpy[1], t_ct_rpy[2])
        self.T_c_t = mat4_from_rt(R_ct, t_ct_xyz)

        # ---- TF 準備 ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # ---- Publisher ----
        self.pub_plane_cam = self.create_publisher(Float64MultiArray, 'mirror_plane_cam', 10)
        self.pub_plane_out = self.create_publisher(Float64MultiArray, 'mirror_plane', 10)  # output_frame側で出す

        # ---- タイマ（30Hz） ----
        self.timer = self.create_timer(1.0/30.0, self.tick)

        self.get_logger().info(
            f'[mirror_plane_estimator] camera_frame={self.cam_frame}, '
            f'output_frame={self.out_frame}, tag_frame={self.tag_frame}'
        )

        self._last_log_ns = 0


    def tick(self):
        """毎フレーム、鏡像タグのTFから S を作り、平面 (n,d) を推定して配信。"""
        now = rclpy.time.Time()

        # 1) 鏡像タグ（仮想タグ）: T_c<-tv を取得
        #    apriltag_ros が camera_frame を親、tag_<ID> を子に出している前提で lookup。
        try:
            ts = self.tf_buffer.lookup_transform(self.cam_frame, self.tag_frame, now)
        except Exception as ex:
            self.get_logger().warn(f'No TF: {self.cam_frame} <- {self.tag_frame} ({ex})')
            return  # そのフレームでタグが見えない
        trans = [ts.transform.translation.x,
                 ts.transform.translation.y,
                 ts.transform.translation.z]
        quat = [ts.transform.rotation.x,
                ts.transform.rotation.y,
                ts.transform.rotation.z,
                ts.transform.rotation.w]
        R_ctv = rot_from_quat(*quat)
        T_c_tv = mat4_from_rt(R_ctv, np.array(trans, dtype=float))

        # 2) 反射変換 S = T_c<-tv * (T_c<-t)^(-1)
        S = T_c_tv @ inv_T(self.T_c_t)

        # 3) S から平面 (n, d) を復元（カメラ座標系）
        n_cam, d_cam = plane_from_reflection(S)

        p_real = self.T_c_t[:3, 3]                      # camera<-tag_real の並進
        p_virt = np.array(trans, dtype=float)           # camera<-tag_virtual の並進

        dir_vec = p_real - p_virt
        norm_dir = np.linalg.norm(dir_vec)
        if norm_dir > 1e-9:
            # 1) 向き補正：n_cam を (p_real - p_virt) と同方向に
            if float(n_cam @ dir_vec) < 0.0:
                n_cam = -n_cam
                d_cam = -d_cam
            # 2) 二等分面を必ず通るよう d を上書き
            m = 0.5 * (p_real + p_virt)
            d_cam = - float(n_cam @ m)

        N_cam = np.array([n_cam[0], n_cam[1], n_cam[2], d_cam], dtype=float)

        # 4) カメラ座標で publish
        msg_cam = Float64MultiArray()
        msg_cam.data = [float(N_cam[0]), float(N_cam[1]), float(N_cam[2]), float(N_cam[3])]
        self.pub_plane_cam.publish(msg_cam)

        # 5) 必要なら output_frame（例: base_link / map）に変換して publish
        try:
            # T_out<-cam を TF から取得
            ts_out = self.tf_buffer.lookup_transform(self.out_frame, self.cam_frame, now)
            T_out_cam = mat4_from_tf(
                [ts_out.transform.translation.x, ts_out.transform.translation.y, ts_out.transform.translation.z],
                [ts_out.transform.rotation.x, ts_out.transform.rotation.y, ts_out.transform.rotation.z, ts_out.transform.rotation.w]
            )
            N_out = transform_plane(N_cam, T_out_cam)
        except Exception:
            # 変換がまだ出ていない等のときは、そのままカメラ系で出す
            N_out = N_cam

        msg_out = Float64MultiArray()
        msg_out.data = [float(N_out[0]), float(N_out[1]), float(N_out[2]), float(N_out[3])]
        self.pub_plane_out.publish(msg_out)

        # 6) 可視化 TF: 平面の最近点 p0 と法線向き
        if self.publish_tf and self.tf_broadcaster is not None:
            # カメラ座標側
            stamp = ts.header.stamp

            p0_cam = -d_cam * n_cam                 # 平面上でカメラ原点に最も近い点
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_log_ns > 1_000_000_000:  # 1秒
                self.get_logger().info(f'plane_cam: n={n_cam}, d={d_cam:.3f}, p0_cam={p0_cam}')
                self._last_log_ns = now_ns
            q_cam = quat_from_two_vectors(np.array([0.0, 0.0, 1.0]), n_cam)  # Z軸→法線
            tmsg = TransformStamped()
            tmsg.header.stamp = stamp
            tmsg.header.frame_id = self.cam_frame
            tmsg.child_frame_id = 'mirror_plane_cam'
            tmsg.transform.translation.x = float(p0_cam[0])
            tmsg.transform.translation.y = float(p0_cam[1])
            tmsg.transform.translation.z = float(p0_cam[2])
            tmsg.transform.rotation.x = float(q_cam[0])
            tmsg.transform.rotation.y = float(q_cam[1])
            tmsg.transform.rotation.z = float(q_cam[2])
            tmsg.transform.rotation.w = float(q_cam[3])
            self.tf_broadcaster.sendTransform(tmsg)

            # 出力フレーム側（出力座標での最近点・法線）
            n_out = N_out[:3]
            d_out = N_out[3]
            p0_out = -d_out * n_out
            q_out = quat_from_two_vectors(np.array([0.0, 0.0, 1.0]), n_out)
            tmsg2 = TransformStamped()
            tmsg2.header.stamp = stamp
            tmsg2.header.frame_id = self.out_frame
            tmsg2.child_frame_id = 'mirror_plane'
            tmsg2.transform.translation.x = float(p0_out[0])
            tmsg2.transform.translation.y = float(p0_out[1])
            tmsg2.transform.translation.z = float(p0_out[2])
            tmsg2.transform.rotation.x = float(q_out[0])
            tmsg2.transform.rotation.y = float(q_out[1])
            tmsg2.transform.rotation.z = float(q_out[2])
            tmsg2.transform.rotation.w = float(q_out[3])
            self.tf_broadcaster.sendTransform(tmsg2)


def main():
    rclpy.init()
    node = MirrorPlaneEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
