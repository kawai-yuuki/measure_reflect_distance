import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)

def generate_launch_description() -> LaunchDescription:
    # 既存のパス取得
    pkg_share = get_package_share_directory("measure_reflect_distance")
    cfg_path = os.path.join(pkg_share, "config", "rtabmap_d455.ini")
    rtabmap_share = get_package_share_directory("rtabmap_launch")
    rtabmap_launch = os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")
    default_rviz_cfg = os.path.join(pkg_share, "config", "debug.rgbd.rviz")
    
    # --- [追加] Nav2関連のパス取得 ---
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    # デフォルトのパラメータファイルパス（必要に応じて自分のパッケージ内のパスに変更してください）
    # 例: os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml") 
    # -----------------------------

    defaults = {
        # 注意: Nav2を使う場合、ロボットのベースフレームを指定するのが一般的です (例: base_link)
        # カメラ単体で動かす場合はcamera_linkでも動きますが、ナビゲーションとしては不自然になることがあります
        "frame_id": "camera_link", 
        "args": (
            "-d "
            "--Optimizer/Strategy 2 "
            "--Vis/CorNNDR 0.75 "
            "--Vis/MaxFeatures 1500 "
            "--RGBD/CreateOccupancyGrid true " # Nav2にはこれが必須 (OK)
            "--Grid/CellSize 0.05 "
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
        "nav2_params_file": default_nav2_params, # [追加]
    }

    declares = [
        _declare("frame_id", defaults["frame_id"], "Base frame id for RTAB-Map"),
        _declare("args", defaults["args"], "Additional rtabmap args (e.g., -d)"),
        _declare("rgb_topic", defaults["rgb_topic"], "RGB image topic"),
        _declare("depth_topic", defaults["depth_topic"], "Depth image topic"),
        _declare("camera_info_topic", defaults["camera_info_topic"], "Camera info topic"),
        _declare("approx_sync", defaults["approx_sync"], "Use approximate time sync"),
        _declare("rviz", defaults["rviz"], "Launch RViz"),
        _declare("rviz_cfg", defaults["rviz_cfg"], "Path to RViz config file"),
        _declare("rtabmap_viz", defaults["rtabmap_viz"], "Launch rtabmap_viz UI"),
        _declare("wait_imu_to_init", defaults["wait_imu_to_init"], "Wait IMU before init"),
        _declare("cfg", defaults["cfg"], "Path to rtabmap.ini"),
        _declare("use_sim_time", defaults["use_sim_time"], "Use simulation clock"),
        # --- [追加] Nav2用引数 ---
        _declare("nav2_params_file", defaults["nav2_params_file"], "Full path to the ROS2 parameters file to use for all launched nodes"),
    ]

    forward_args = {
        key: LaunchConfiguration(key)
        for key in [
            "frame_id", "args", "rgb_topic", "depth_topic", "camera_info_topic",
            "approx_sync", "rviz", "wait_imu_to_init", "cfg", "rviz_cfg",
            "rtabmap_viz", "use_sim_time",
        ]
    }

    # 1. RTAB-Mapの起動
    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments=forward_args.items()
    )

    # 2. [追加] Nav2の起動
    # map_serverやamclを含まない navigation_launch.py を使用します
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
    )

    return LaunchDescription([*declares, include_rtabmap, include_nav2])