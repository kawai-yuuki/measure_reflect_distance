from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1つ目の画像パブリッシャー
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher_rgb',  # 1. ユニークなノード名
            output='screen',
            parameters=[{'publish_rate': 30.0, 'frame_id': 'image_rgb'}],
            arguments=['/home/kawai/rosbag_record/20250726/for_rtabmap2_mirror/extracted_images/000774.png'], # 3. 配信したい画像のパス
            remappings=[
                ('/image_raw', '/camera/image_rgb') # 2. トピック名を変更
            ]
        ),

        # 2つ目の画像パブリッシャー
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher_mask',  # 1. ユニークなノード名
            output='screen',
            parameters=[{'publish_rate': 30.0, 'frame_id': 'image_mask'}],
            arguments=['/home/kawai/rosbag_record/20250726/for_rtabmap2_mirror/mask/SegmentationClass/000774.png'], # 3. 配信したい画像のパス
            remappings=[
                ('/image_raw', '/camera/image_mask') # 2. トピック名を変更
            ]
        ),

        # 3つ目以降も同様に追加可能...
    ])