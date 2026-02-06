import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    
    # デフォルトのパラメータファイルパス（launchファイル基準で解決）
    # symlink-install 時は src 側、通常ビルド時は install 側の config を参照します
    default_nav2_params = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "nav2_params.yaml")
    )
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
            "--Grid/CellSize 0.01 "
            "--Grid\ClusterRadius = 0.05 "
            "--Grid/MinGroundHeight -0.45 "
            "--Grid/MaxGroundHeight 0.0 "
            "--Grid/MaxObstacleHeight 0.0 "
            "--Grid\MinObstacleHeight = -0.5 "
            "--Grid/MaxGroundAngle 15 "
            "--Grid\MinClusterSize = 5 "
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
        _declare("autostart", "true", "Automatically startup the nav2 stack"),
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
    # nav2_bringup経由だとparamsが反映されない場合があるため、必要なノードを直接起動
    nav2_params = LaunchConfiguration("nav2_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': use_sim_time,
                'FollowPath.critics': [
                    'RotateToGoal',
                    'Oscillation',
                    'BaseObstacle',
                    'GoalAlign',
                    'PathAlign',
                    'PathDist',
                    'GoalDist',
                ],
            },
        ],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        *declares,
        LogInfo(msg=["[nav2] params_file: ", LaunchConfiguration("nav2_params_file")]),
        LogInfo(msg=["[nav2] use_sim_time: ", LaunchConfiguration("use_sim_time")]),
        include_rtabmap,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
