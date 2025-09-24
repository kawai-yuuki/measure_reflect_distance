#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import tf2_ros

# 既に util に切り出している関数を利用
from measure_reflect_distance.util.mirror_geometry import rot_from_rpy, quat_from_R

class TagRealStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('tag_real_static_broadcaster')

        # ---- パラメータ ----
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame',  'tag_real')
        self.declare_parameter('t_ct_xyz', [0.017545, -0.080829, -0.021476])   # [m]
        self.declare_parameter('t_ct_rpy', [-0.023080, 0.001224, -3.131105])   # [rad] roll, pitch, yaw

        parent = self.get_parameter('parent_frame').value
        child  = self.get_parameter('child_frame').value
        xyz = np.array(self.get_parameter('t_ct_xyz').value, dtype=float)
        rpy = np.array(self.get_parameter('t_ct_rpy').value, dtype=float)

        # ---- バリデーション ----
        if parent == child:
            raise ValueError('parent_frame と child_frame が同一です。別名にしてください。')
        if xyz.shape != (3,):
            raise ValueError('t_ct_xyz は [x,y,z] の3要素で指定してください。')
        if rpy.shape != (3,):
            raise ValueError('t_ct_rpy は [roll,pitch,yaw] の3要素で指定してください（単位: rad）')

        # ---- 同次変換の構築（カメラ<-実タグ） ----
        R = rot_from_rpy(rpy[0], rpy[1], rpy[2])  # Rz*Ry*Rx
        q = quat_from_R(R)

        # ---- static broadcaster ----
        self._static_bc = tf2_ros.StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = Time(sec=0, nanosec=0)   # static は 0 でOK
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self._static_bc.sendTransform(t)

        # ---- ログ（1回だけ） ----
        self.get_logger().info(
            f'[tag_real STATIC] {parent} -> {child}\n'
            f'  t = [{xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f}] m\n'
            f'  rpy(rad) = [{rpy[0]:+.4f}, {rpy[1]:+.4f}, {rpy[2]:+.4f}]\n'
            f'  quat(xyzw) = [{q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f}]'
        )

def main():
    rclpy.init()
    node = TagRealStaticBroadcaster()
    try:
        rclpy.spin(node)  # 常駐でOK（/tf_staticはlatchedだが、後続ノード起動の安全策として）
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
