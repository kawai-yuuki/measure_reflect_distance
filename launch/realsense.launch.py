import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    """Keep all args centrally defined (why: ensure easy overrides & consistency)."""
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)



def generate_launch_description() -> LaunchDescription:
    realsense_pkg_share = get_package_share_directory("realsense2_camera")
    rs_launch = os.path.join(realsense_pkg_share, "launch", "rs_launch.py")


    # Defaults copied from your CLI
    defaults = {
        "align_depth.enable": "true",
        "enable_gyro": "true",
        "enable_accel": "true",
        "unite_imu_method": "1",
        "enable_sync": "true",
        "rgb_camera.color_profile": "640x480x30",
        "depth_module.depth_profile": "640x480x30",
        # imu_filter_madgwick additions
        "imu_input_topic": "/camera/camera/imu", # match your CLI
        "use_mag": "false",
        "publish_tf": "false",
    }


    # Make every key a launch arg so users can override at runtime
    declares = [
        _declare("align_depth.enable", defaults["align_depth.enable"], "Enable depth alignment"),
        _declare("enable_gyro", defaults["enable_gyro"], "Enable gyro stream"),
        _declare("enable_accel", defaults["enable_accel"], "Enable accel stream"),
        _declare("unite_imu_method", defaults["unite_imu_method"], "IMU sync mode (0: none, 1: copy, etc.)"),
        _declare("enable_sync", defaults["enable_sync"], "Sync depth/color frames"),
        _declare("rgb_camera.color_profile", defaults["rgb_camera.color_profile"], "Color stream profile WxHxFPS"),
        _declare("depth_module.depth_profile", defaults["depth_module.depth_profile"], "Depth stream profile WxHxFPS"),
        # imu_filter_madgwick args
        _declare("imu_input_topic", defaults["imu_input_topic"], "Input IMU topic for filter"),
        _declare("use_mag", defaults["use_mag"], "Use magnetometer in filter"),
        _declare("publish_tf", defaults["publish_tf"], "Publish orientation TF"),
    ]


    # Forward all declared args to the official rs_launch.py
    forward_args = {
        key: LaunchConfiguration(key)
        for key in [
            "align_depth.enable",
            "enable_gyro",
            "enable_accel",
            "unite_imu_method",
            "enable_sync",
            "rgb_camera.color_profile",
            "depth_module.depth_profile",
        ]
    }


    include_rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments=forward_args.items(),
    )

    # Madgwick filter node using the united IMU (/imu)
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[
            {"use_mag": LaunchConfiguration("use_mag")},
            {"publish_tf": LaunchConfiguration("publish_tf")},
        ],
        remappings=[
            ("imu/data_raw", LaunchConfiguration("imu_input_topic")),
        ],
    )


    return LaunchDescription([*declares, include_rs, imu_filter])