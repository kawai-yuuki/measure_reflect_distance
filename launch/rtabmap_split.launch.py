# rtabmap_split.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('measure_reflect_distance')
    cfg_default = os.path.join(pkg, 'config', 'rtabmap_d455.ini')

    # Args
    cfg = DeclareLaunchArgument('cfg', default_value=cfg_default)
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    frame_id = DeclareLaunchArgument('frame_id', default_value='camera_link')

    # Topic args（Realsense D455）
    rgb = DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw')
    depth = DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw')
    caminfo = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info')

    # Substitutions
    cfg_path = LaunchConfiguration('cfg')
    use_sim = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('frame_id')
    rgb_t = LaunchConfiguration('rgb_topic')
    depth_t = LaunchConfiguration('depth_topic')
    caminfo_t = LaunchConfiguration('camera_info_topic')

    # rgbd_odometry（ODOM）
    odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'frame_id': base_frame,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'config_path': cfg_path,   # ← .ini を読む
        }],
        # ODOM 側だけ必要な最小上書き（ini でもOK、どちらかで重複可）
        arguments=[
            '--Reg/Force3DoF', 'true',
            '--Vis/FeatureType', '6',
            '--Vis/CorNNDR', '0.75',
            '--Vis/DepthAsMask', 'true',
            '--Vis/Iterations', '100',
            '--Vis/MinInliers', '30',
            '--Vis/MinDepth', '0.2',
            '--Vis/MaxDepth', '5.0',
            '--Odom/VisKeyFrameThr', '100',
            '--Odom/KeyFrameThr', '0.6',
            '--OdomF2M/MaxSize', '4000',
            '--OdomF2M/InitDepthFactor', '0.02',
        ],
        remappings=[
            ('rgb/image', rgb_t),
            ('depth/image', depth_t),
            ('rgb/camera_info', caminfo_t),
            ('odom', 'odom'),
            ('imu', '/imu/data'),
        ],
    )

    # rtabmap（SLAM 本体）
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'frame_id': base_frame,
            'approx_sync': True,
            'cfg': cfg_path,  # ← rtabmap は "cfg" でも ini を読む
        }],
        arguments=[
            '-d',  # debug prints
            '--Optimizer/Strategy', '2',
            '--RGBD/CreateOccupancyGrid', 'true',
            '--Grid/CellSize', '0.05',
            '--Vis/InlierDistance', '0.03',
            '--Vis/MaxFeatures', '1500',
            '--Icp/MaxCorrespondenceDistance', '0.05',
            '--Icp/Iterations', '30',
        ],
        remappings=[
            ('rgb/image', rgb_t),
            ('depth/image', depth_t),
            ('rgb/camera_info', caminfo_t),
            ('odom', 'odom'),
        ],
    )

    # rtabmap_viz（cfg は渡さない！）
    viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'approx_sync': True,
            'subscribe_rgb': True, 
            'subscribe_depth': True,
        }],
        remappings=[
            ('rgb/image', rgb_t),
            ('depth/image', depth_t),
            ('rgb/camera_info', caminfo_t),
            ('odom', 'odom'),
        ],
    )

    point_cloud_xyzrgb = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'approx_sync': True,
            # 可視化を軽くするなら（任意）
            'voxel_size': 0.03,            # 3cm 体素
            'decimation': 2,               # 画像間引き
            'max_range': 5.0               # D455 に合わせた範囲
        }],
        remappings=[
            ('rgb/image',        rgb_t),                  # ← bag に合わせる
            ('depth/image',      depth_t), # ← bag に合わせる
            ('rgb/camera_info',  caminfo_t),                # ← bag に合わせる
            # 出力トピック（既定で /cloud と /rtabmap/cloud_map を出します）
        ]
    )

    return LaunchDescription([
        cfg, use_sim_time, frame_id, rgb, depth, caminfo,
        odom_node, slam_node, viz_node, point_cloud_xyzrgb
    ])
