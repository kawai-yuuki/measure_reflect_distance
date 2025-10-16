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

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
import tf2_ros

from measure_reflect_distance.util.mirror_geometry import (
    rot_from_rpy, rot_from_quat, quat_from_R,
    mat4_from_rt, mat4_from_tf, inv_T, plane_from_reflection, transform_plane
)

class MirrorPlaneEstimator(Node):
    def __init__(self):
        super().__init__('mirror_plane_estimator')

        # ---- パラメータ定義 ----
        # カメラ座標・出力座標・鏡像タグのフレーム名を外部から受け取る
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')  # D455カラー光学フレーム
        self.declare_parameter('output_frame', 'map')  # 出力先フレーム（例: base_link / map）
        self.declare_parameter('tag_frame_name', 'reflected')                     # 鏡像タグのフレーム名（apriltag_ros の出力）
        # 実タグ（カメラ近傍に剛体固定）→ カメラ の外部 T_c<-t
        self.declare_parameter('t_ct_xyz', [0.017545,-0.080829,-0.021476])  # [m]
        self.declare_parameter('t_ct_rpy', [-0.023080,0.001224,-3.131105])  # [rad] roll, pitch, yaw
        self.declare_parameter('publish_tf', True)  # 可視化TFを出すか
        self.declare_parameter('tag_tf_timeout_sec', 0.5)  # 鏡像タグの TF がこの秒数古ければ無効とみなす

        # ---- パラメータ取得 ----
        self.cam_frame = self.get_parameter('camera_frame').value
        self.out_frame = self.get_parameter('output_frame').value
        self.tag_frame = str(self.get_parameter('tag_frame_name').value)
        t_ct_xyz = np.array(self.get_parameter('t_ct_xyz').value, dtype=float)
        t_ct_rpy = np.array(self.get_parameter('t_ct_rpy').value, dtype=float)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tag_tf_timeout = float(self.get_parameter('tag_tf_timeout_sec').value)
        # 実タグ→カメラ: T_c<-t を4x4に構成
        R_ct = rot_from_rpy(t_ct_rpy[0], t_ct_rpy[1], t_ct_rpy[2])
        self.T_c_t = mat4_from_rt(R_ct, t_ct_xyz)

        # ---- TF 準備 ----
        # 鏡像タグ（仮想タグ）の姿勢を取り出し、必要に応じて TF を再配信する
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        self._plane_visible = False
        self._warned_missing_out_tf = False  # output_frame との TF が未接続な場合の一度きり警告制御
        self._warned_stale_tag_tf = False

        # ---- Publisher ----
        # 数値は Float64MultiArray として出し、可視化は別ノード（mapper）に任せる
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
        query_time = rclpy.time.Time()
        now_ros = self.get_clock().now()
        stamp_now = now_ros.to_msg()

        # 1) 鏡像タグ（仮想タグ）: T_c<-tv を取得
        #    apriltag_ros が camera_frame を親、tag_<ID> を子に出している前提で lookup。
        try:
            # apriltag_ros publishes camera_frame <- tag_frame. Query latest transform.
            ts = self.tf_buffer.lookup_transform(self.cam_frame, self.tag_frame, query_time)
        except Exception as ex:
            if self._plane_visible:
                self.get_logger().warn(
                    f'No TF: {self.cam_frame} <- {self.tag_frame} ({ex}); stop publishing mirror plane'
                )
                self._plane_visible = False
            return  # そのフレームでタグが見えない

        # --- 古い鏡像 TF は無効 ---
        tag_stamp = ts.header.stamp
        tag_age = (now_ros.nanoseconds - (tag_stamp.sec * 1_000_000_000 + tag_stamp.nanosec)) / 1e9
        if tag_age < 0.0:
            tag_age = 0.0
        if self.tag_tf_timeout > 0.0 and tag_age > self.tag_tf_timeout:
            # Stale TF means the tag disappeared from view; stop publishing until it returns.
            if not self._warned_stale_tag_tf:
                self.get_logger().warn(
                    f'Stale mirror tag TF (age={tag_age:.2f}s > {self.tag_tf_timeout:.2f}s); treating as not visible'
                )
                self._warned_stale_tag_tf = True
            if self._plane_visible:
                self._plane_visible = False
            return
        self._warned_stale_tag_tf = False

        if not self._plane_visible:
            self.get_logger().info('Mirror plane tag detected; resume publishing')
        self._plane_visible = True

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
        #    (カメラから見た仮想タグと実タグの関係行列)
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
        trans_vec = np.array(trans, dtype=float)
        denom = float(n_cam @ trans_vec)
        residual_trans = float(n_cam @ trans_vec) + d_cam
        proj_cam = trans_vec - residual_trans * n_cam

        self.get_logger().info(
            "Plane debug: "
            f"norm_dir={norm_dir:.6f}, "
            f"dot_dir={float(n_cam @ dir_vec):.6f}, "
            f"denom={denom:.6f}, "
            f"residual_trans={residual_trans:.6f}"
        )

        N_cam = np.array([n_cam[0], n_cam[1], n_cam[2], d_cam], dtype=float)

        # 鏡像タグ方向の線分と平面との交点（反射点）を算出
        MIN_PROJECTION_DISTANCE = 0.05
        MIN_REAL_PLANE_DISTANCE = 0.05
        MIN_TANGENT_NORM = 1e-2

        p0_cam = -d_cam * n_cam                     # 平面の最近点
        p_plane_cam = proj_cam                      # 鏡像タグ中心の平面への直交投影点
        dist_real = np.linalg.norm(p_plane_cam - p_real)

        diff_real_virtual = p_real - p_virt
        tangent_candidate = diff_real_virtual - float(diff_real_virtual @ n_cam) * n_cam
        reliable_tangent = True
        tangent_norm_candidate = np.linalg.norm(tangent_candidate)
        if tangent_norm_candidate > MIN_TANGENT_NORM:
            tangent_cam = tangent_candidate / tangent_norm_candidate
        else:
            axis_candidates = [
                np.array([1.0, 0.0, 0.0]),
                np.array([0.0, 1.0, 0.0]),
                np.array([0.0, 0.0, 1.0]),
            ]
            tangent_cam = None
            for axis in axis_candidates:
                candidate = axis - float(axis @ n_cam) * n_cam
                norm_candidate = np.linalg.norm(candidate)
                if norm_candidate > 1e-6:
                    tangent_cam = candidate / norm_candidate
                    break
            if tangent_cam is None:
                tangent_cam = np.array([1.0, 0.0, 0.0])
            reliable_tangent = False
        binormal_cam = np.cross(n_cam, tangent_cam)
        norm_bin_cam = np.linalg.norm(binormal_cam)
        if norm_bin_cam > MIN_TANGENT_NORM:
            binormal_cam /= norm_bin_cam
        else:
            binormal_cam = np.cross(n_cam, np.array([0.0, 0.0, 1.0]))
            binormal_cam /= (np.linalg.norm(binormal_cam) + 1e-12)
            reliable_tangent = False
        tangent_cam = np.cross(binormal_cam, n_cam)
        tangent_norm = np.linalg.norm(tangent_cam)
        if tangent_norm > MIN_TANGENT_NORM:
            tangent_cam /= tangent_norm
        else:
            reliable_tangent = False

        expected_normal = np.array([0.0, 0.0, -1.0])
        cos_angle = float(np.clip(n_cam @ expected_normal, -1.0, 1.0))
        angle_deg = math.degrees(math.acos(cos_angle))
        if angle_deg > 180.0:
            angle_deg = 360.0 - angle_deg
        reliable_tf = reliable_tangent and angle_deg <= 50.0
        if not reliable_tf:
            if self.publish_tf and self.tf_broadcaster is not None:
                self.get_logger().debug(
                    "Skipping mirror plane output due to reliability filters "
                    f"(residual_trans={residual_trans:.3f} m, dist_real={dist_real:.3f} m, "
                    f"tangent_norm={tangent_norm:.3f})"
                )
            return

        # 4) カメラ座標で publish (法線・距離・反射点・接線)
        msg_cam = Float64MultiArray()
        msg_cam.data = [
            float(N_cam[0]),
            float(N_cam[1]),
            float(N_cam[2]),
            float(N_cam[3]),
            float(p_plane_cam[0]),
            float(p_plane_cam[1]),
            float(p_plane_cam[2]),
            float(tangent_cam[0]),
            float(tangent_cam[1]),
            float(tangent_cam[2]),
        ]
        self.pub_plane_cam.publish(msg_cam)

        # 5) 必要なら output_frame（例: base_link / map）に変換して publish
        plane_out_available = False
        try:
            # T_out<-cam を TF から取得
            ts_out = self.tf_buffer.lookup_transform(self.out_frame, self.cam_frame, query_time)
            T_out_cam = mat4_from_tf(
                [ts_out.transform.translation.x, ts_out.transform.translation.y, ts_out.transform.translation.z],
                [ts_out.transform.rotation.x, ts_out.transform.rotation.y, ts_out.transform.rotation.z, ts_out.transform.rotation.w]
            )
            N_out = transform_plane(N_cam, T_out_cam)
            plane_out_available = True
            self._warned_missing_out_tf = False
        except Exception as ex:
            if not self._warned_missing_out_tf:
                self.get_logger().warn(
                    f'No TF: {self.out_frame} <- {self.cam_frame} ({ex}); skip mirror plane output in {self.out_frame}'
                )
                self._warned_missing_out_tf = True
            plane_out_available = False

        R_out = None
        if plane_out_available:
            # map 側へ平面を流す（以降の可視化／平均化ノードの入力）
            n_out = N_out[:3]
            d_out = N_out[3]
            p0_out = -d_out * n_out
            p_plane_out = T_out_cam[:3, :3] @ p_plane_cam + T_out_cam[:3, 3]
            tangent_out = T_out_cam[:3, :3] @ tangent_cam
            # Project back to plane to ensure orthogonality
            tangent_out = tangent_out - float(tangent_out @ n_out) * n_out
            norm_tangent_out = np.linalg.norm(tangent_out)
            if norm_tangent_out > 1e-6:
                tangent_out /= norm_tangent_out
            else:
                tangent_out = np.array([1.0, 0.0, 0.0])
                reliable_tf = False

            binormal_out = np.cross(n_out, tangent_out)
            norm_bin_out = np.linalg.norm(binormal_out)
            if norm_bin_out > 1e-6:
                binormal_out /= norm_bin_out
            else:
                binormal_out = np.cross(n_out, np.array([0.0, 0.0, 1.0]))
                binormal_out /= (np.linalg.norm(binormal_out) + 1e-12)
                reliable_tf = False
            tangent_out = np.cross(binormal_out, n_out)
            tangent_out /= (np.linalg.norm(tangent_out) + 1e-12)
            R_out = np.column_stack((tangent_out, binormal_out, n_out))
            q_out = quat_from_R(R_out)

            msg_out = Float64MultiArray()
            msg_out.data = [
                float(N_out[0]),
                float(N_out[1]),
                float(N_out[2]),
                float(N_out[3]),
                float(p_plane_out[0]),
                float(p_plane_out[1]),
                float(p_plane_out[2]),
                float(tangent_out[0]),
                float(tangent_out[1]),
                float(tangent_out[2]),
            ]
            self.pub_plane_out.publish(msg_out)

        else:
            n_out = None
            d_out = None
            p0_out = None
            p_plane_out = None
            tangent_out = None
            q_out = None
            R_out = None

        # 6) 可視化 TF: 平面の最近点 p0 と法線向き
        if self.publish_tf and self.tf_broadcaster is not None and reliable_tf:
            anchor_cam = p_plane_cam
            # カメラ座標側
            now_ns = now_ros.nanoseconds
            if now_ns - self._last_log_ns > 1_000_000_000:  # 1秒
                self.get_logger().info(f'plane_cam: n={n_cam}, d={d_cam:.3f}, p_plane_cam={p_plane_cam}')
                self._last_log_ns = now_ns
            binormal_cam = np.cross(n_cam, tangent_cam)
            norm_bin_cam = np.linalg.norm(binormal_cam)
            if norm_bin_cam > 1e-6:
                binormal_cam /= norm_bin_cam
            else:
                binormal_cam = np.cross(n_cam, np.array([0.0, 0.0, 1.0]))
                binormal_cam /= (np.linalg.norm(binormal_cam) + 1e-12)
            tangent_cam = np.cross(binormal_cam, n_cam)
            tangent_cam /= (np.linalg.norm(tangent_cam) + 1e-12)
            R_cam = np.column_stack((tangent_cam, binormal_cam, n_cam))
            q_cam = quat_from_R(R_cam)
            tmsg = TransformStamped()
            tmsg.header.stamp = stamp_now
            tmsg.header.frame_id = self.cam_frame
            tmsg.child_frame_id = 'mirror_plane_cam'
            tmsg.transform.translation.x = float(anchor_cam[0])
            tmsg.transform.translation.y = float(anchor_cam[1])
            tmsg.transform.translation.z = float(anchor_cam[2])
            tmsg.transform.rotation.x = float(q_cam[0])
            tmsg.transform.rotation.y = float(q_cam[1])
            tmsg.transform.rotation.z = float(q_cam[2])
            tmsg.transform.rotation.w = float(q_cam[3])
            self.tf_broadcaster.sendTransform(tmsg)

            # 出力フレーム側（出力座標での最近点・法線）
            if plane_out_available and p_plane_out is not None and tangent_out is not None and q_out is not None:
                anchor_out = p_plane_out if p0_out is not None and np.linalg.norm(p_plane_out - p0_out) > 1e-6 else p0_out
                if anchor_out is None:
                    anchor_out = p_plane_out
                offset_out = float(n_out @ anchor_out) + d_out
                anchor_out = anchor_out - offset_out * n_out
                tmsg2 = TransformStamped()
                tmsg2.header.stamp = stamp_now
                tmsg2.header.frame_id = self.out_frame
                tmsg2.child_frame_id = 'mirror_plane'
                tmsg2.transform.translation.x = float(anchor_out[0])
                tmsg2.transform.translation.y = float(anchor_out[1])
                tmsg2.transform.translation.z = float(anchor_out[2])
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
