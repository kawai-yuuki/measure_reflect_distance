import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # apriltag_ros のパラメータ YAML を環境から推定
    try:
        # インストール済みパッケージから共通の YAML を探す
        cfg_in_share = os.path.join(
            get_package_share_directory("apriltag_ros"),
            "cfg",
            "reflect_tag_36h11.yaml",
        )
        default_params = cfg_in_share
    except Exception:
        # 開発中（ソースから直参照）の場合は HOME 配下をフォールバック
        default_params = os.path.join(
            os.environ.get("HOME", ""),
            "ros2_ws",
            "src",
            "apriltag_ros",
            "cfg",
            "reflect_tag_36h11.yaml",
        )

    # ---- Launch Args（必要なら上書き可能）----
    camera_frame   = LaunchConfiguration("camera_frame")
    output_frame   = LaunchConfiguration("output_frame")
    tag_frame_name = LaunchConfiguration("tag_frame_name")
    t_ct_xyz       = LaunchConfiguration("t_ct_xyz")
    t_ct_rpy       = LaunchConfiguration("t_ct_rpy")
    publish_tf     = LaunchConfiguration("publish_tf")
    use_sim_time   = LaunchConfiguration("use_sim_time")
    params_file    = LaunchConfiguration("params_file")
    image_topic    = LaunchConfiguration("image_topic")
    apriltag_masked_image_topic = LaunchConfiguration("apriltag_masked_image_topic")
    apriltag_exclude_top_ratio = LaunchConfiguration("apriltag_exclude_top_ratio")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    apriltag_camera_info_topic = LaunchConfiguration("apriltag_camera_info_topic")
    apriltag_node_name = LaunchConfiguration("apriltag_node_name")
    plane_angle_threshold = LaunchConfiguration("plane_angle_threshold_deg")
    plane_distance_threshold = LaunchConfiguration("plane_distance_threshold")
    plane_display_timeout = LaunchConfiguration("plane_display_timeout_sec")
    tag_tf_timeout = LaunchConfiguration("tag_tf_timeout_sec")
    tag_clock_mismatch_threshold = LaunchConfiguration("tag_clock_mismatch_threshold_sec")
    tf_reliability_max_normal_angle_deg = LaunchConfiguration("tf_reliability_max_normal_angle_deg")
    require_reliable_tf = LaunchConfiguration("require_reliable_tf")
    mapper_plane_topic = LaunchConfiguration("mapper_plane_topic")
    mapper_frame_id = LaunchConfiguration("mapper_frame_id")
    mapper_publish_markers = LaunchConfiguration("mapper_publish_markers")

    return LaunchDescription([
        # 引数宣言（デフォルトは現状コードと合わせています）
        DeclareLaunchArgument("camera_frame",   default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("output_frame",   default_value="map"),
        DeclareLaunchArgument("tag_frame_name", default_value="reflected"),
        DeclareLaunchArgument("t_ct_xyz",       default_value="[0.018305, -0.083156, -0.018434]"),
        DeclareLaunchArgument("t_ct_rpy",       default_value="[-0.010357, -0.000770, -3.127285]"),
        DeclareLaunchArgument("publish_tf",     default_value="true"),
        # rosbag / シミュ時間を使う場合 true
        DeclareLaunchArgument("use_sim_time",   default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=TextSubstitution(text=default_params),
            description="apriltag_ros のパラメータYAML（明示指定で上書き可）",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value=TextSubstitution(text="/camera/camera/color/image_raw"),
            description="apriltag_ros が購読する画像トピック",
        ),
        DeclareLaunchArgument(
            "apriltag_masked_image_topic",
            default_value=TextSubstitution(text="/camera/camera/apriltag/image_raw"),
            description="Apriltag 検出専用のマスク済み画像トピック（専用名前空間推奨）",
        ),
        DeclareLaunchArgument(
            "apriltag_exclude_top_ratio",
            default_value=TextSubstitution(text="0.1666667"),
            description="Apriltag 検出で観測対象外にする画像上部の比率 [0.0, 1.0)",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=TextSubstitution(text="/camera/camera/color/camera_info"),
            description="入力側 CameraInfo トピック（bag/driver の生データ）",
        ),
        DeclareLaunchArgument(
            "apriltag_camera_info_topic",
            default_value=TextSubstitution(text="/camera/camera/apriltag/camera_info"),
            description="apriltag_ros が購読する CameraInfo トピック（masked image と同名空間）",
        ),
        DeclareLaunchArgument(
            "apriltag_node_name",
            default_value=TextSubstitution(text="apriltag"),
            description="起動する apriltag_ros ノード名",
        ),
        DeclareLaunchArgument(
            "plane_angle_threshold_deg",
            default_value="10.0",
            description="鏡面クラスタをまとめる際の法線角度閾値 [deg]",
        ),
        DeclareLaunchArgument(
            "plane_distance_threshold",
            default_value="0.3",
            description="鏡面クラスタをまとめる際の距離閾値 [m]",
        ),
        DeclareLaunchArgument(
            "plane_display_timeout_sec",
            default_value="1.0",
            description="鏡面観測が止まってからマーカーを非表示にするまでの秒数（0 で常時表示）",
        ),
        DeclareLaunchArgument(
            "tag_tf_timeout_sec",
            default_value="0.8",
            description="鏡像タグ TF がこの秒数以上古い場合は観測無しとみなす",
        ),
        DeclareLaunchArgument(
            "tag_clock_mismatch_threshold_sec",
            default_value="3600.0",
            description="TF stamp がシステム時刻系とこの秒数以上ずれる場合、stale 判定を stamp 更新ベースへ切替",
        ),
        DeclareLaunchArgument(
            "tf_reliability_max_normal_angle_deg",
            default_value="40.0",
            description="mirror_plane_cam TF の信頼度判定に使う法線角度閾値 [deg]",
        ),
        DeclareLaunchArgument(
            "require_reliable_tf",
            default_value="true",
            description="true の場合、信頼度判定を満たさない鏡面推定は publish しない",
        ),
        DeclareLaunchArgument(
            "mapper_plane_topic",
            default_value="mirror_plane",
            description="mirror_plane_mapper が購読する鏡面トピック",
        ),
        DeclareLaunchArgument(
            "mapper_frame_id",
            default_value="map",
            description="mirror_plane_mapper がマーカーを描画するフレームID",
        ),
        DeclareLaunchArgument(
            "mapper_publish_markers",
            default_value="true",
            description="mirror_plane_mapper で MarkerArray を publish するかどうか",
        ),

        # --- Apriltag 入力前処理ノード ---
        # Apriltag 検出専用に画像上部をマスクする。元画像トピックは変更しない。
        Node(
            package="measure_reflect_distance",
            executable="apriltag_exclusion_mask_node",
            name="apriltag_exclusion_mask_node",
            output="screen",
            parameters=[{
                "input_topic": image_topic,
                "output_topic": apriltag_masked_image_topic,
                "exclude_top_ratio": ParameterValue(apriltag_exclude_top_ratio, value_type=float),
                "use_sim_time": use_sim_time,
            }],
        ),

        # --- Apriltag 用 CameraInfo 同期ノード ---
        # masked image の stamp に合わせて CameraInfo を再配信することで
        # image_transport の exact 同期失敗を防ぐ。
        Node(
            package="measure_reflect_distance",
            executable="sync_camera_info",
            name="apriltag_camera_info_sync",
            output="screen",
            parameters=[{
                "image_topic": apriltag_masked_image_topic,
                "source_camera_info_topic": camera_info_topic,
                "camera_info_topic": apriltag_camera_info_topic,
                "force_frame_id": "",
                "use_sim_time": use_sim_time,
            }],
        ),

        # --- AprilTag Detection ノード ---
        # カメラ画像から AprilTag を検出し TF を出力
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name=apriltag_node_name,
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("image_rect", apriltag_masked_image_topic),
                ("camera_info", apriltag_camera_info_topic),
            ],
            emulate_tty=True,
        ),

        # --- 実タグ static TF broadcaster ---
        # カメラに固定された実タグの位置（camera_frame ← tag_real）を静的 TF で流す
        Node(
            package="measure_reflect_distance",
            executable="tag_real_static_broadcaster",
            name="tag_real_static_broadcaster",
            output="screen",
            parameters=[{
                "parent_frame":   camera_frame,
                "t_ct_xyz":       t_ct_xyz,  # 文字列 → ノード側で配列に解釈しているならOK
                "t_ct_rpy":       t_ct_rpy,
                "use_sim_time":   use_sim_time,
                # tag_real_frame をノードで declare しているならここで上書き可能
                # "tag_real_frame": "tag_real"
            }],
        ),

        # --- 鏡平面推定ノード ---
        # 実タグ と 鏡像タグ の TF から鏡面パラメータを計算する（Marker は mapper 側で描画）
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
                "publish_marker": False,
                "tag_tf_timeout_sec": tag_tf_timeout,
                "tag_clock_mismatch_threshold_sec": tag_clock_mismatch_threshold,
                "tf_reliability_max_normal_angle_deg": tf_reliability_max_normal_angle_deg,
                "require_reliable_tf": ParameterValue(require_reliable_tf, value_type=bool),
                "use_sim_time":   use_sim_time,
            }],
            # apriltag 側トピック名を変えている場合はここで remap も可能
            # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),

        # --- 鏡面マップ投影ノード ---
        # 推定された鏡面（map 基準）をクラスタ化・平均化し、MarkerArray として可視化
        Node(
            package="measure_reflect_distance",
            executable="mirror_plane_mapper",
            name="mirror_plane_mapper",
            output="screen",
            parameters=[{
                "plane_topic": mapper_plane_topic,
                "frame_id":    mapper_frame_id,
                "angle_threshold_deg": plane_angle_threshold,
                "distance_threshold":  plane_distance_threshold,
                "display_timeout_sec": plane_display_timeout,
                "publish_markers": ParameterValue(mapper_publish_markers, value_type=bool),
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
