from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---- Launch Args（必要なら上書き可能）----
    camera_frame   = LaunchConfiguration("camera_frame")
    output_frame   = LaunchConfiguration("output_frame")
    tag_frame_name = LaunchConfiguration("tag_frame_name")
    t_ct_xyz       = LaunchConfiguration("t_ct_xyz")
    t_ct_rpy       = LaunchConfiguration("t_ct_rpy")
    publish_tf     = LaunchConfiguration("publish_tf")
    use_sim_time   = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        # 引数宣言（デフォルトは現状コードと合わせています）
        DeclareLaunchArgument("camera_frame",   default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("output_frame",   default_value="map"),
        DeclareLaunchArgument("tag_frame_name", default_value="reflected"),
        DeclareLaunchArgument("t_ct_xyz",       default_value="[0.017545, -0.080829, -0.021476]"),
        DeclareLaunchArgument("t_ct_rpy",       default_value="[-0.023080, 0.001224, -3.131105]"),
        DeclareLaunchArgument("publish_tf",     default_value="true"),
        # rosbag / シミュ時間を使う場合 true
        DeclareLaunchArgument("use_sim_time",   default_value="true"),

        # --- 実タグ static TF broadcaster ---
        Node(
            package="measure_reflect_distance",
            executable="tag_real_static_broadcaster",
            name="tag_real_static_broadcaster",
            output="screen",
            parameters=[{
                "camera_frame":   camera_frame,
                "t_ct_xyz":       t_ct_xyz,  # 文字列 → ノード側で配列に解釈しているならOK
                "t_ct_rpy":       t_ct_rpy,
                "use_sim_time":   use_sim_time,
                # tag_real_frame をノードで declare しているならここで上書き可能
                # "tag_real_frame": "tag_real"
            }],
        ),

        # --- 鏡平面推定ノード ---
        Node(
            package="measure_reflect_distance",
            executable="mirror_plane_estimation",  # console_scripts 名に合わせる
            name="mirror_plane_estimator",
            output="screen",
            parameters=[{
                "camera_frame":   camera_frame,
                "output_frame":   output_frame,
                "tag_frame_name": tag_frame_name,
                "t_ct_xyz":       t_ct_xyz,
                "t_ct_rpy":       t_ct_rpy,
                "publish_tf":     publish_tf,
                "use_sim_time":   use_sim_time,
            }],
            # apriltag 側トピック名を変えている場合はここで remap も可能
            # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ])
