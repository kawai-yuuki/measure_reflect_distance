import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- デフォルトYAMLをユーザー名非依存で推定 ---
    try:
        # 1) インストール済みのパッケージ共有ディレクトリから
        cfg_in_share = os.path.join(
            get_package_share_directory('apriltag_ros'),
            'cfg', 'reflect_tag_36h11.yaml'
        )
        default_params = cfg_in_share
    except Exception:
        # 2) ソースツリー想定 ($HOME 基点)
        default_params = os.path.join(
            os.environ.get('HOME', ''), 'ros2_ws', 'src',
            'apriltag_ros', 'cfg', 'reflect_tag_36h11.yaml'
        )

    return LaunchDescription([
        # ===== 引数 =====
        DeclareLaunchArgument(
            'params_file',
            default_value=TextSubstitution(text=default_params),
            description='apriltag_ros のパラメータYAML（明示指定で上書き可）'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value=TextSubstitution(text='/camera/camera/color/image_raw'),
            description='入力画像トピック（apriltag_ros 側の image_rect にリマップ）'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=TextSubstitution(text='/camera/camera/color/camera_info'),
            description='CameraInfo トピック（apriltag_ros 側の camera_info にリマップ）'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value=TextSubstitution(text='apriltag'),
            description='起動するノード名（YAML側の先頭キーと合わせると◎）'
        ),

        # ===== ノード =====
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('image_rect', LaunchConfiguration('image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
            ],
            emulate_tty=True,
        ),
    ])