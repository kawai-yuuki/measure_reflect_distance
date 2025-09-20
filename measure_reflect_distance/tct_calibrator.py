#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
T_ct（本体タグ→RealSenseカラー光学）の外部パラメータを、1ショット（または複数ショット平均）で求めるノード。

前提：
- RealSense 側 apriltag_ros： camera_color_optical_frame <- base        （参照タグ, ID=7）
- OAK-D   側 apriltag_ros：   oak_rgb_camera_optical_frame <- oak_base   （参照タグ, ID=7）
                               oak_rgb_camera_optical_frame <- oak_reflected（本体タグ, ID=0）
- 参照タグ（base/oak_base）は壁や床などに固定
- 本体タグ（oak_reflected）は RealSense に剛体接続

式：
  T_{c←t} = T_{c←tr_rs} * (T_{e←tr_oak})^{-1} * T_{e←tc}
    c : RealSense color optical  (= camera_color_optical_frame)
    e : OAK-D color optical      (= oak_rgb_camera_optical_frame)
    tr_rs  : RealSenseが観測する参照タグ（base）
    tr_oak : OAKが観測する参照タグ（oak_base）
    tc     : OAKが観測する本体タグ（oak_reflected）
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros


# ---- 小さな行列ユーティリティ ----

def inv_T(T):
    """剛体変換の逆行列"""
    R = T[:3,:3]; t = T[:3,3]
    Ti = np.eye(4); Ti[:3,:3]=R.T; Ti[:3,3]= -R.T@t
    return Ti

def rot_from_quat(x,y,z,w):
    """クォータニオン→回転行列 3x3（内部正規化つき）"""
    n = x*x+y*y+z*z+w*w
    if n < 1e-16:
        return np.eye(3)
    s = 2.0/n
    X,Y,Z = x*s, y*s, z*s
    wx,wy,wz = w*X, w*Y, w*Z
    xx,xy,xz = x*X, x*Y, x*Z
    yy,yz,zz = y*Y, y*Z, z*Z
    return np.array([
        [1-(yy+zz),  xy-wz,     xz+wy],
        [  xy+wz, 1-(xx+zz),    yz-wx],
        [  xz-wy,    yz+wx,  1-(xx+yy)]
    ])

def mat4_from_tf(trans, quat_xyzw):
    """geometry_msgs/Transform を 4x4 同次行列へ"""
    x,y,z = trans
    qx,qy,qz,qw = quat_xyzw
    T = np.eye(4)
    T[:3,:3] = rot_from_quat(qx,qy,qz,qw)
    T[:3, 3] = np.array([x,y,z], dtype=float)
    return T

def quat_from_R(R):
    """回転行列→クォータニオン（x,y,z,w）。安定な分岐版。"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1.0)*2
        w = 0.25*s
        x = (R[2,1]-R[1,2])/s
        y = (R[0,2]-R[2,0])/s
        z = (R[1,0]-R[0,1])/s
    else:
        i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0,0]-R[1,1]-R[2,2])*2
            x = 0.25*s
            y = (R[0,1]+R[1,0])/s
            z = (R[0,2]+R[2,0])/s
            w = (R[2,1]-R[1,2])/s
        elif i == 1:
            s = math.sqrt(1.0 - R[0,0] + R[1,1]- R[2,2])*2
            x = (R[0,1]+R[1,0])/s
            y = 0.25*s
            z = (R[1,2]+R[2,1])/s
            w = (R[0,2]-R[2,0])/s
        else:
            s = math.sqrt(1.0 - R[0,0]- R[1,1]+ R[2,2])*2
            x = (R[0,2]+R[2,0])/s
            y = (R[1,2]+R[2,1])/s
            z = 0.25*s
            w = (R[1,0]-R[0,1])/s
    q = np.array([x,y,z,w], dtype=float)
    q /= np.linalg.norm(q)
    return q

def rpy_from_R(R):
    """R から roll, pitch, yaw（ZYX順）"""
    sy = -R[2,0]
    sy = max(-1.0, min(1.0, float(sy)))
    pitch = math.asin(sy)
    yaw = math.atan2(R[1,0], R[0,0])
    roll = math.atan2(R[2,1], R[2,2])
    return roll, pitch, yaw

def quat_avg(qs):
    """Markley 法によるクォータニオン平均（w>=0 に揃える）"""
    A = np.zeros((4,4))
    for q in qs:
        q = q/np.linalg.norm(q)
        A += np.outer(q,q)
    _, V = np.linalg.eigh(A)
    q = V[:, -1]
    if q[3] < 0: q = -q
    return q


# ---- キャリブレーションノード ----

class TctCalibrator(Node):
    def __init__(self):
        super().__init__('tct_calibrator')

        # ★ここをあなたの環境に合わせて初期値設定（上書きも可）
        self.declare_parameter('rs_cam_frame',  'camera_color_optical_frame')   # RealSense color optical
        self.declare_parameter('oak_cam_frame', 'oak_rgb_camera_optical_frame') # OAK-D   color optical

        self.declare_parameter('rs_tag_ref_frame', 'base')                      # 参照タグ   (ID=7)
        self.declare_parameter('oak_tag_ref_frame', 'oak_base')                      # 参照タグ   (ID=7)

        self.declare_parameter('tag_cam_frame', 'oak_reflected')                      # 本体のタグ (ID=0)

        self.declare_parameter('samples', 12)        # 取得サンプル数（静止姿勢を少し変えながら）
        self.declare_parameter('interval_sec', 0.3)  # 取得間隔秒

        self.c  = self.get_parameter('rs_cam_frame').value
        self.e  = self.get_parameter('oak_cam_frame').value
        self.tr_rs = self.get_parameter('rs_tag_ref_frame').value
        self.tr_oak = self.get_parameter('oak_tag_ref_frame').value
        self.tc = self.get_parameter('tag_cam_frame').value
        self.N  = int(self.get_parameter('samples').value)
        self.dt = float(self.get_parameter('interval_sec').value)

        # TF 準備
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.k = 0
        self.T_list = []
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'Collecting {self.N} samples.\n'
            f'RS : {self.c} <- {self.tr_rs}  (ref tag seen by RealSense)\n'
            f'OAK: {self.e} <- {self.tr_oak} (ref tag), {self.e} <- {self.tc} (cam tag)\n'
            f'Please keep all three TFs available.'
        )


    def tick(self):
        """一定間隔で TF を読み、1サンプルずつ T_{c←t} を推定して貯める"""
        now = rclpy.time.Time()
        try:
            # 置き換え：tick() の try ブロック内の lookup_transform
            ts_ctr = self.tf_buffer.lookup_transform(self.c, self.tr_rs,  now)  # T_c<-tr_rs
            ts_etr = self.tf_buffer.lookup_transform(self.e, self.tr_oak, now)  # T_e<-tr_oak
            ts_etc = self.tf_buffer.lookup_transform(self.e, self.tc,     now)  # T_e<-tc

        except Exception as ex:
            self.get_logger().warn(f'Waiting TF... ({ex})')
            return

        # geometry_msgs/Transform → 4x4 に変換
        T_c_tr = mat4_from_tf(
            [ts_ctr.transform.translation.x, ts_ctr.transform.translation.y, ts_ctr.transform.translation.z],
            [ts_ctr.transform.rotation.x, ts_ctr.transform.rotation.y, ts_ctr.transform.rotation.z, ts_ctr.transform.rotation.w]
        )
        T_e_tr = mat4_from_tf(
            [ts_etr.transform.translation.x, ts_etr.transform.translation.y, ts_etr.transform.translation.z],
            [ts_etr.transform.rotation.x, ts_etr.transform.rotation.y, ts_etr.transform.rotation.z, ts_etr.transform.rotation.w]
        )
        T_e_tc = mat4_from_tf(
            [ts_etc.transform.translation.x, ts_etc.transform.translation.y, ts_etc.transform.translation.z],
            [ts_etc.transform.rotation.x, ts_etc.transform.rotation.y, ts_etc.transform.rotation.z, ts_etc.transform.rotation.w]
        )

        # ★コア式：T_{c←t} = T_{c←tr} * (T_{e←tr})^{-1} * T_{e←tc}
        T_c_t = T_c_tr @ inv_T(T_e_tr) @ T_e_tc

        self.T_list.append(T_c_t)
        self.k += 1
        self.get_logger().info(f'sample {self.k}/{self.N} collected')

        if self.k >= self.N:
            self.timer.cancel()
            self.finish()

    def finish(self):
        """複数サンプルからロバストに平均を取り、t_ct_xyz / t_ct_rpy を印字"""
        if len(self.T_list) == 0:
            self.get_logger().error('No samples collected.')
            rclpy.shutdown()
            return

        Ts = np.stack(self.T_list, axis=0)

        # 並進は中央値、回転はクォータニオン平均
        ts = Ts[:, :3, 3]
        t_med = np.median(ts, axis=0)

        qs = []
        for T in Ts:
            q = quat_from_R(T[:3, :3])
            if q[3] < 0:   # w>=0 に揃える
                q = -q
            qs.append(q)

        q_avg = quat_avg(np.stack(qs))  # ← ループの外でまとめて平均
        R_avg = rot_from_quat(q_avg[0], q_avg[1], q_avg[2], q_avg[3])
        roll, pitch, yaw = rpy_from_R(R_avg)

        print('\n==== Estimated extrinsics  T_{c←t}  (cam tag -> RS color optical) ====')
        print(f'frames:')
        print(f'  RS cam frame (c):        {self.c}')
        print(f'  OAK cam frame (e):       {self.e}')
        print(f'  RS ref tag (tr_rs):      {self.tr_rs}')
        print(f'  OAK ref tag (tr_oak):    {self.tr_oak}')
        print(f'  OAK cam tag (t = tc):    {self.tc}')
        print(f'samples used: {Ts.shape[0]} / requested: {self.N}')

        # 並進の分散目安も併記
        t_std = np.std(ts, axis=0)
        print(f'\nT_c←t translation [m]')
        print(f'  t_ct_xyz: [{t_med[0]:.6f}, {t_med[1]:.6f}, {t_med[2]:.6f}]')
        print(f'  std_xyz : [{t_std[0]:.4f}, {t_std[1]:.4f}, {t_std[2]:.4f}]')

        # 角度はrad/deg両方
        roll_deg, pitch_deg, yaw_deg = [v*180.0/math.pi for v in (roll, pitch, yaw)]
        print(f'\nT_c←t rotation (ZYX rpy)')
        print(f'  t_ct_rpy [rad]: [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]')
        print(f'  t_ct_rpy [deg]: [{roll_deg:.3f}, {pitch_deg:.3f}, {yaw_deg:.3f}]')

        print('\nPaste t_ct_xyz / t_ct_rpy [rad] into mirror_plane_estimator params.')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = TctCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
