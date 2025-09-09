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
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            "rgb_camera.color_profile": '640x480x30',
            "depth_module.depth_profile": '640x480x30',
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
