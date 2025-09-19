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
from geometry_msgs.msg import TransformStamped
import tf2_ros


# =========================
# 線形代数の最小ユーティリティ（依存削減のため自前実装）
# =========================

def rot_from_rpy(roll, pitch, yaw):
    """
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    - ROSのRPY規約に合わせて Z(ヨー)→Y(ピッチ)→X(ロール) の順
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0,  cr, -sr],
                   [0.0,  sr,  cr]])
    return Rz @ Ry @ Rx

def rot_from_quat(x, y, z, w):
    """
    クォータニオン (x, y, z, w) から回転行列 3x3 を作る。
    - 正規化を内部で行う
    - 標準的な式に基づく実装
    """
    n = x*x + y*y + z*z + w*w
    if n < 1e-16:
        return np.eye(3)
    s = 2.0 / n
    X, Y, Z = x*s, y*s, z*s
    wx, wy, wz = w*X, w*Y, w*Z
    xx, xy, xz = x*X, x*Y, x*Z
    yy, yz, zz = y*Y, y*Z, z*Z
    return np.array([
        [1.0-(yy+zz),   xy-wz,       xz+wy],
        [  xy+wz,     1.0-(xx+zz),   yz-wx],
        [  xz-wy,       yz+wx,     1.0-(xx+yy)]
    ])

def quat_from_two_vectors(a, b):
    """
    単位ベクトル a を b に最短回転で合わせるクォータニオン (x,y,z,w) を返す。
    - 境界ケース: a と b がほぼ反対のときは任意の直交軸で 180 度回転
    - 可視化用の姿勢生成（Z軸→法線）に使用
    """
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    w = 1.0 + float(a @ b)
    if w < 1e-8:
        # 180度回転：aに直交する任意軸を使う
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / (np.linalg.norm(v) + 1e-12)
        return np.array([v[0], v[1], v[2], 0.0])
    q = np.array([v[0], v[1], v[2], w])
    return q / (np.linalg.norm(q) + 1e-12)

def mat4_from_rt(R, t):
    """回転 R(3x3) と 並進 t(3,) から 4x4 同次変換を作る。"""
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = t
    return T

def mat4_from_tf(translation, quat_xyzw):
    """geometry_msgs/Transform の値から 4x4 同次変換を作る。"""
    x, y, z = translation
    qx, qy, qz, qw = quat_xyzw
    R = rot_from_quat(qx, qy, qz, qw)
    return mat4_from_rt(R, np.array([x, y, z], dtype=float))

def inv_T(T):
    """同次変換の逆行列（剛体変換）"""
    R = T[:3,:3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def plane_from_reflection(S):
    """
    反射変換 S から平面パラメータ (n, d) を復元。
    - S = [[R, t],[0,1]],  R = I - 2 n n^T,  t = 2 d n
    - M = (I - R)/2 = n n^T は rank-1。SVD の最大特異ベクトルが n。
    - d = 0.5 * n^T t
    - 規約: カメラ前方の平面で d < 0 になるよう符号を揃える（n はカメラに向く向き）
    """
    R = S[:3,:3]
    t = S[:3, 3]
    M = 0.5 * (np.eye(3) - R)
    U, s, Vt = np.linalg.svd(M)
    n = U[:, 0]
    n = n / (np.linalg.norm(n) + 1e-12)
    d = 0.5 * float(n @ t)
    if d > 0:
        n = -n
        d = -d
    return n, d

def transform_plane(N_a, T_b_a):
    """
    平面4ベクトル N の座標変換（論文と同じ転置ルール）:
        N_b = (T_b_a^{-1})^T * N_a
    - N = [nx, ny, nz, d]^T,  平面方程式は n^T x + d = 0
    - 最後に ||n||=1 へ正規化
    """
    T_inv = inv_T(T_b_a)
    N_b = T_inv.T @ N_a
    n = N_b[:3]
    d = N_b[3]
    norm = np.linalg.norm(n) + 1e-12
    n /= norm
    d /= norm
    return np.array([n[0], n[1], n[2], d])


# =========================
# ROS2 ノード本体
# =========================

class MirrorPlaneEstimator(Node):
    def __init__(self):
        super().__init__('mirror_plane_estimator')

        # ---- パラメータ定義 ----
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')  # D455カラー光学フレーム
        self.declare_parameter('output_frame', 'camera_link')  # 出力先フレーム（例: base_link / map）
        self.declare_parameter('tag_id', 0)  # ミラー用タグID
        # 実タグ（カメラ近傍に剛体固定）→ カメラ の外部 T_c<-t
        self.declare_parameter('t_ct_xyz', [0.0, 0.0, 0.0])  # [m]
        self.declare_parameter('t_ct_rpy', [0.0, 0.0, 0.0])  # [rad] roll, pitch, yaw
        self.declare_parameter('publish_tf', True)  # 可視化TFを出すか

        # ---- パラメータ取得 ----
        self.cam_frame = self.get_parameter('camera_frame').value
        self.out_frame = self.get_parameter('output_frame').value
        self.tag_id = int(self.get_parameter('tag_id').value)
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

        self.tag_frame = f'tag_{self.tag_id}'
        self.get_logger().info(
            f'[mirror_plane_estimator] camera_frame={self.cam_frame}, '
            f'output_frame={self.out_frame}, tag_frame={self.tag_frame}'
        )

    def tick(self):
        """毎フレーム、鏡像タグのTFから S を作り、平面 (n,d) を推定して配信。"""
        now = rclpy.time.Time()

        # 1) 鏡像タグ（仮想タグ）: T_c<-tv を取得
        #    apriltag_ros が camera_frame を親、tag_<ID> を子に出している前提で lookup。
        try:
            ts = self.tf_buffer.lookup_transform(self.cam_frame, self.tag_frame, now)
        except Exception:
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
            p0_cam = -d_cam * n_cam                 # 平面上でカメラ原点に最も近い点
            q_cam = quat_from_two_vectors(np.array([0.0, 0.0, 1.0]), n_cam)  # Z軸→法線
            tmsg = TransformStamped()
            tmsg.header.stamp = self.get_clock().now().to_msg()
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
            tmsg2.header.stamp = self.get_clock().now().to_msg()
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
