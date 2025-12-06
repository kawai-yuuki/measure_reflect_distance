from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)


def _default_tag_cfg() -> str:
    try:
        share = get_package_share_directory("apriltag_ros")
        candidate = os.path.join(share, "cfg", "reflect_tag_36h11.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return ""


def generate_launch_description() -> LaunchDescription:
    # Common
    detections = LaunchConfiguration("detections_topic")
    image = LaunchConfiguration("image_topic")
    camera_info = LaunchConfiguration("camera_info_topic")
    camera_frame = LaunchConfiguration("camera_frame")
    tag_cfg = LaunchConfiguration("tag_config_path")

    default_tag_cfg = _default_tag_cfg()

    return LaunchDescription(
        [
            _declare("detections_topic", "/detections", "AprilTag detections topic"),
            _declare("image_topic", "/camera/camera/color/image_raw", "RGB image topic"),
            _declare("camera_info_topic", "/camera/camera/color/camera_info", "CameraInfo topic"),
            _declare("camera_frame", "camera_color_optical_frame", "Camera optical frame"),
            _declare("tag_config_path", default_tag_cfg, "Path to tag config yaml"),
            _declare("mirror_marker_topic", "/mirror_surface_markers", "Marker topic for mirror"),
            _declare("wall_marker_topic", "/wall_plane_markers", "Marker topic for wall"),
            _declare("overlay_topic", "/apriltag_overlay/image", "Output image topic for overlay"),
            _declare("plane_size_m", "1.0", "Wall plane patch size (edge length, meters)"),
        ]
        + [
            Node(
                package="measure_reflect_distance",
                executable="apriltag_overlay_node",
                name="apriltag_overlay_node",
                output="screen",
                parameters=[
                    {
                        "detections_topic": detections,
                        "image_topic": image,
                        "camera_info_topic": camera_info,
                        "camera_frame": camera_frame,
                        "tag_config_path": tag_cfg,
                        "output_image_topic": LaunchConfiguration("overlay_topic"),
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="mirror_surface_marker_node",
                name="mirror_surface_marker_node",
                output="screen",
                parameters=[
                    {
                        "detections_topic": detections,
                        "camera_frame": camera_frame,
                        "tag_config_path": tag_cfg,
                        "marker_topic": LaunchConfiguration("mirror_marker_topic"),
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="wall_plane_marker_node",
                name="wall_plane_marker_node",
                output="screen",
                parameters=[
                    {
                        "detections_topic": detections,
                        "camera_frame": camera_frame,
                        "tag_config_path": tag_cfg,
                        "marker_topic": LaunchConfiguration("wall_marker_topic"),
                        "plane_size_m": LaunchConfiguration("plane_size_m"),
                    }
                ],
            ),
        ]
    )
