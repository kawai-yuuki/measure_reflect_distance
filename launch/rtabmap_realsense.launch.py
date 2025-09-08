import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)

def generate_launch_description() -> LaunchDescription:
    rtabmap_share = get_package_share_directory("rtabmap_launch")
    rtabmap_launch = os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")

    # デフォルト値（ご提示の CLI と同じ）
    defaults = {
        "frame_id": "camera_link",
        "args": "-d",  # デバッグ（RTAB-Mapの-d）
        "rgb_topic": "/camera/camera/color/image_raw",
        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/camera/color/camera_info",
        "approx_sync": "true",
        "rviz": "true",
        "wait_imu_to_init": "true",
    }

    declares = [
        _declare("frame_id", defaults["frame_id"], "Base frame id for RTAB-Map"),
        _declare("args", defaults["args"], "Additional rtabmap args (e.g., -d)"),
        _declare("rgb_topic", defaults["rgb_topic"], "RGB image topic"),
        _declare("depth_topic", defaults["depth_topic"], "Depth image topic"),
        _declare("camera_info_topic", defaults["camera_info_topic"], "Camera info topic"),
        _declare("approx_sync", defaults["approx_sync"], "Use approximate time sync (true/false)"),
        _declare("rviz", defaults["rviz"], "Launch RViz (true/false)"),
        _declare("wait_imu_to_init", defaults["wait_imu_to_init"], "Wait IMU before init (true/false)"),
    ]

    forward_args = {
        key: LaunchConfiguration(key)
        for key in [
            "frame_id",
            "args",
            "rgb_topic",
            "depth_topic",
            "camera_info_topic",
            "approx_sync",
            "rviz",
            "wait_imu_to_init",
        ]
    }

    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments=forward_args.items()
    )

    return LaunchDescription([*declares, include_rtabmap])
