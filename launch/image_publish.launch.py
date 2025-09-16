import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 現在のユーザ名を取得
    user_name = os.getenv('USER')
    
    return LaunchDescription([
        # 1つ目の画像パブリッシャー
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher_rgb',  # 1. ユニークなノード名
            output='screen',
            parameters=[{'publish_rate': 30.0, 'frame_id': 'image_rgb'}],
            arguments=[f'/media/{user_name}/KIOXIA/dataset/20250719/for_nvblox_2/image/002202.png'], # 3. 配信したい画像のパス
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
            arguments=[f'/media/{user_name}/KIOXIA/dataset/20250719/for_nvblox_2/mask/SegmentationClass/002202.png'], # 3. 配信したい画像のパス
            remappings=[
                ('/image_raw', '/camera/image_mask') # 2. トピック名を変更
            ]
        ),

        Node(
            package='measure_reflect_distance',              # sync_camera_info.py を入れたパッケージ
            executable='sync_camera_info',        # setup.py の entry_point に合わせる
            name='camera_info_sync',
            parameters=[{
                'image_topic': '/camera/image_rgb',
                'camera_info_topic': '/camera/camera_info',
                'camera_info_url': 'file:///path/to/calibration.yaml',
                # 'force_frame_id': 'camera_color_optical_frame',  # 必要なら固定
            }],
        ),

        # 3つ目以降も同様に追加可能...
    ])