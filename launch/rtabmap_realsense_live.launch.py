import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("measure_reflect_distance")
    cfg_path = os.path.join(pkg_share, "config", "rtabmap_d455.ini")
    rtabmap_share = get_package_share_directory("rtabmap_launch")
    rtabmap_launch = os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")
    # default_rviz_cfg = os.path.join(rtabmap_share, "launch", "config", "rgbd.rviz")
    default_rviz_cfg = os.path.join(pkg_share, "config", "debug.rgbd.rviz")  # 独自 RViz 設定

    defaults = {
        "frame_id": "camera_link",
        "args": (
            "-d "
            "--Optimizer/Strategy 2 "
            "--Vis/CorNNDR 0.75 "
            "--Vis/MaxFeatures 1500 "
            # "--RGBD/ProximityBySpace false "
            # "--RGBD/ProximityByTime false "
            # "--RGBD/LoopClosureReextractFeatures false "
            # "--Rtabmap/LoopThr 1 "
            # "--Rtabmap/MaxRetrieved 0 "
            "--RGBD/CreateOccupancyGrid  true "
            "--Grid/CellSize 0.05 "
            # "--Vis/InlierDistance 0.05 "
            # "--Icp/MaxCorrespondenceDistance 0.07 "
            "--Vis/MinInliers 30 "
            "--Icp/Iterations 30 "
        ),
        "rgb_topic": "/camera/camera/color/image_raw",
        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/camera/color/camera_info",
        "approx_sync": "true",
        "rviz": "true",
        "rviz_cfg": default_rviz_cfg,
        "rtabmap_viz": "false",
        "wait_imu_to_init": "true",
        "cfg": cfg_path,
        "use_sim_time": "false",
        # "odom_args": (
        #     "--Reg/Force3DoF true "
        #     "--Vis/FeatureType 6 "
        #     "--ORB/NLevels 8 "
        #     "--ORB/ScaleFactor 1.2 "
        #     "--ORB/EdgeThreshold 31 "
        #     "--Vis/MaxFeatures 2000 "
        #     "--Vis/MinInliers 15 "
        #     "--Vis/MinDepth 0.2 "
        #     "--Vis/MaxDepth 5.0 "
        #     "--Vis/DepthAsMask true "
        #     "--Vis/InlierDistance 0.05 "
        #     "--Vis/CorNNDR 0.75 "
        #     "--Vis/Iterations 150 "
        #     "--Odom/VisKeyFrameThr 120 "
        #     "--Odom/KeyFrameThr 0.7 "
        #     "--OdomF2M/MaxSize 5000 "
        #     "--OdomF2M/InitDepthFactor 0.03 "
        # ),
    }

    declares = [
        _declare("frame_id", defaults["frame_id"], "Base frame id for RTAB-Map"),
        _declare("args", defaults["args"], "Additional rtabmap args (e.g., -d)"),
        _declare("rgb_topic", defaults["rgb_topic"], "RGB image topic"),
        _declare("depth_topic", defaults["depth_topic"], "Depth image topic"),
        _declare("camera_info_topic", defaults["camera_info_topic"], "Camera info topic"),
        _declare("approx_sync", defaults["approx_sync"], "Use approximate time sync (true/false)"),
        _declare("rviz", defaults["rviz"], "Launch RViz (true/false)"),
        _declare("rviz_cfg", defaults["rviz_cfg"], "Path to RViz config file"),
        _declare("rtabmap_viz", defaults["rtabmap_viz"], "Launch rtabmap_viz UI (true/false)"),
        _declare("wait_imu_to_init", defaults["wait_imu_to_init"], "Wait IMU before init (true/false)"),
        _declare("cfg", defaults["cfg"], "Path to rtabmap.ini"),
        # _declare("odom_args", defaults["odom_args"], "Additional odometry args"),
        _declare("use_sim_time", defaults["use_sim_time"], "Use simulation clock if true"),
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
            "cfg",
            # "odom_args",
            "rviz_cfg",
            "rtabmap_viz",
            "use_sim_time",
        ]
    }

    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments=forward_args.items()
    )

    return LaunchDescription([*declares, include_rtabmap])
