import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam_node',  # ノード名を指定
            namespace='webcam',      # 名前空間を指定
            output='screen',
            parameters=[
                # ===================================
                # 基本設定 (Basic Settings)
                # ===================================
                {'video_device': '/dev/video0'},          # カメラデバイスのパス
                {'camera_frame_id': 'camera_link'},       # TFフレームID

                # ===================================
                # 画像フォーマット・解像度 (Image Format & Resolution)
                # ===================================
                # 利用可能なフォーマットは `v4l2-ctl --list-formats-ext` で確認
                {'pixel_format': 'YUYV'},                 # ピクセルフォーマット (例: 'YUYV', 'MJPG')
                {'image_size': [640, 480]},               # [幅, 高さ]
                {'framerate': 30.0},                      # フレームレート

                # ===================================
                # V4L2コントロール (V4L2 Controls)
                # 利用可能なコントロールは `v4l2-ctl --list-ctrls` で確認
                # ===================================
                # 明るさ (0-255など)
                # {'brightness': 128},
                # コントラスト (0-255など)
                # {'contrast': 32},
                # 彩度 (0-255など)
                # {'saturation': 32},
                # ゲイン (0-255など)
                # {'gain': 100},
                # ホワイトバランス (True: 自動, False: 手動)
                {'white_balance_automatic': True},
                # ↑をFalseにした場合、手動で値を設定
                # {'white_balance_temperature': 4000},
                # 露出 (True: 自動, False: 手動)
                {'auto_exposure': 3}, # 1: Manual Mode, 3: Aperture Priority Mode
                # ↑をManualにした場合、手動で値を設定
                # {'exposure_absolute': 250},

                # ===================================
                # ROSトピック・QoS (ROS Topics & QoS)
                # ===================================
                {'qos_overrides./camera_info.publisher.reliability': 'best_effort'},
                {'qos_overrides./image_raw.publisher.reliability': 'best_effort'},
            ]
        )
    ])