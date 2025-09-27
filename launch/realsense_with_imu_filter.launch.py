from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # realsense2_camera の公式ローンチを取り込む
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        # あなたのコマンドで指定していた引数をそのまま反映
        launch_arguments={
            # アライン
            'align_depth.enable': 'true',

            # IMU
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',

            # 解像度・フレームレート
            "rgb_camera.color_profile": '640x480x30',
            "depth_module.depth_profile": '848x480x30',
            # "colorizer.enable": 'true',

            # ▼ 露出固定（深度/IR）
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': '8500',     # μs: 環境で調整
            'depth_module.gain': '16',
            'depth_module.emitter_enabled': '1', # 0:off, 1:on, 2:toggle系
            'depth_module.laser_power': '150',

            # High Accuracy相当（任意）
            # ドライバ実装では preset の整数値:
            # 0:Custom, 1:Default, 2:Hand, 3:HighAccuracy, 4:HighDensity, 5:MediumDensity
            'depth_module.visual_preset': '3',

            # ▼ 露出固定（カラー）
            'rgb_camera.enable_auto_exposure': 'false',
            'rgb_camera.exposure': '128',       # μs
            'rgb_camera.gain': '70',
            'rgb_camera.enable_auto_white_balance': 'false',
            'rgb_camera.auto_exposure_priority': 'false',

            # フィルタ
            "hole_filling_filter.enable": 'true',
            "hole_filling_filter.holes_fill": '1',

            "decimation_filter.enable": 'true',
            "decimation_filter.decimation_magnitude": '2',

            "temporal_filter.enable": 'true',
            "temporal_filter.holes_fill": '0',
            "temporal_filter.smooth_alpha": '0.2',
            "temporal_filter.smooth_delta": '80',
            "temporal_filter.persistency_index": '3',

            "spatial_filter.enable": 'true',
            "spatial_filter.holes_fill": '0',
            "spatial_filter.magnitude": '2',
            "spatial_filter.smooth_alpha": '0.5',
            "spatial_filter.smooth_delta": '20',

            # その他
            # 'initial_reset': 'true',
            'depth_module.frame_queue_size': '32',
            'rgb_camera.frame_queue_size': '32',
        }.items()
    )

    # Madgwick フィルタノード（imu_filter_madgwick）
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
            {
                'use_mag': False,     # --ros-args -p use_mag:=False
                'publish_tf': False,  # --ros-args -p publish_tf:=False
            }
        ],
        # あなたのコマンドで指定していたリマップ
        #   -r imu/data_raw:=/camera/camera/imu
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
        ]
    )

    return LaunchDescription([
        realsense_launch,
        imu_filter_node,
    ])
