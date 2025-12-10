from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _declare(name: str, default: str, desc: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        name=name,
        default_value=TextSubstitution(text=default),
        description=desc,
    )


def generate_launch_description() -> LaunchDescription:
    mask_topic = LaunchConfiguration("mask_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    plane_topic = LaunchConfiguration("plane_topic")
    target_frame = LaunchConfiguration("target_frame")
    projection_mode = LaunchConfiguration("projection_mode")
    pixel_stride = LaunchConfiguration("pixel_stride")
    point_scale = LaunchConfiguration("point_scale")
    raw_marker_topic = LaunchConfiguration("raw_marker_topic")
    projected_points_topic = LaunchConfiguration("projected_points_topic")

    grid_resolution = LaunchConfiguration("grid_resolution")
    grid_density_threshold = LaunchConfiguration("grid_density_threshold")
    min_points = LaunchConfiguration("min_points")
    grid_update_period_sec = LaunchConfiguration("grid_update_period_sec")
    aggregated_marker_topic = LaunchConfiguration("aggregated_marker_topic")
    cluster_plane_topic = LaunchConfiguration("cluster_plane_topic")

    return LaunchDescription(
        [
            _declare("mask_topic", "/mask_image_processed", "Input mask topic"),
            _declare("camera_info_topic", "/camera/camera/color/camera_info", "CameraInfo topic"),
            _declare("plane_topic", "mirror_plane_clustered", "Clustered mirror plane topic"),
            _declare("target_frame", "map", "Target frame for projection"),
            _declare("projection_mode", "dense", "Projection mode: contour or dense"),
            _declare("pixel_stride", "2", "Pixel stride for dense projection"),
            _declare("point_scale", "0.03", "Marker scale for dense points"),
            _declare(
                "raw_marker_topic",
                "mirror_surface_projected_markers_raw",
                "Marker topic published directly by the projector",
            ),
            _declare(
                "projected_points_topic",
                "mirror_surface_projected_points",
                "PointCloud2 topic with projected points",
            ),
            _declare("grid_resolution", "0.01", "Aggregator grid resolution [m]"),
            _declare("grid_density_threshold", "0.2", "Density threshold for polygon extraction"),
            _declare("min_points", "10", "Minimum points required before rendering"),
            _declare("grid_update_period_sec", "1.0", "Aggregator update period [s]"),
            _declare(
                "aggregated_marker_topic",
                "mirror_surface_projected_markers",
                "Marker topic for aggregated polygons",
            ),
            Node(
                package="measure_reflect_distance",
                executable="mirror_surface_projector_node",
                name="mirror_surface_projector_node",
                output="screen",
                parameters=[
                    {
                        "mask_topic": mask_topic,
                        "camera_info_topic": camera_info_topic,
                        "plane_topic": plane_topic,
                        "target_frame": target_frame,
                        "projection_mode": projection_mode,
                        "pixel_stride": ParameterValue(pixel_stride, value_type=int),
                        "point_scale": ParameterValue(point_scale, value_type=float),
                        "marker_topic": raw_marker_topic,
                        "projected_points_topic": projected_points_topic,
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="mirror_surface_projection_aggregator_node",
                name="mirror_surface_projection_aggregator_node",
                output="screen",
                parameters=[
                    {
                        "projected_points_topic": projected_points_topic,
                        "cluster_plane_topic": plane_topic,
                        "marker_topic": aggregated_marker_topic,
                        "target_frame": target_frame,
                        "grid_resolution": ParameterValue(grid_resolution, value_type=float),
                        "grid_density_threshold": ParameterValue(
                            grid_density_threshold, value_type=float
                        ),
                        "min_points": ParameterValue(min_points, value_type=int),
                        "grid_update_period_sec": ParameterValue(
                            grid_update_period_sec, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
