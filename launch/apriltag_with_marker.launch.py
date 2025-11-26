import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _default_param_path():
    """Resolve reflect_tag_36h11.yaml even when installed or running from source."""
    try:
        share_dir = get_package_share_directory('apriltag_ros')
        return os.path.join(share_dir, 'cfg', 'reflect_tag_36h11.yaml')
    except Exception:
        home = os.environ.get('HOME', '')
        return os.path.join(home, 'ros2_ws', 'src', 'apriltag_ros', 'cfg', 'reflect_tag_36h11.yaml')


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=TextSubstitution(text=_default_param_path()),
        description='apriltag_ros のパラメータYAML'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value=TextSubstitution(text='/camera/camera/color/image_raw'),
        description='入力画像 (apriltag_ros の image_rect にリマップ)'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value=TextSubstitution(text='/camera/camera/color/camera_info'),
        description='CameraInfo (apriltag_ros の camera_info にリマップ)'
    )
    node_name_arg = DeclareLaunchArgument(
        'apriltag_node_name',
        default_value=TextSubstitution(text='apriltag'),
        description='apriltag_ros ノード名'
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_output_frame',
        default_value=TextSubstitution(text='map'),
        description='TagPlaneMarker の出力フレームID'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='全ノードで /clock を使用するか'
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name=LaunchConfiguration('apriltag_node_name'),
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    marker_node = Node(
        package='measure_reflect_distance',
        executable='tag_plane_marker_node',
        name='tag_plane_marker',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'detections_topic': '/detections',
            'marker_topic': '/tag_plane_markers',
            'output_frame': LaunchConfiguration('marker_output_frame'),
            'target_ids': [1, 2, 3, 4, 5, 6],
            'smoothing_alpha': 0.2,
        }],
    )

    return LaunchDescription([
        params_file_arg,
        image_topic_arg,
        camera_info_topic_arg,
        node_name_arg,
        marker_frame_arg,
        use_sim_time_arg,
        apriltag_node,
        marker_node,
    ])
