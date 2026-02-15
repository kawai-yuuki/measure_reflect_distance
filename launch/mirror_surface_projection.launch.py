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
    use_sim_time = LaunchConfiguration("use_sim_time")
    target_frame = LaunchConfiguration("target_frame")
    projection_mode = LaunchConfiguration("projection_mode")
    pixel_stride = LaunchConfiguration("pixel_stride")
    point_scale = LaunchConfiguration("point_scale")
    projection_min_abs_denom = LaunchConfiguration("projection_min_abs_denom")
    max_projection_distance = LaunchConfiguration("max_projection_distance")
    raw_marker_topic = LaunchConfiguration("raw_marker_topic")
    projected_points_topic = LaunchConfiguration("projected_points_topic")
    polygon_points_topic = LaunchConfiguration("polygon_points_topic")
    polygon_point_stride_cells = LaunchConfiguration("polygon_point_stride_cells")
    mirror_points_topic = LaunchConfiguration("mirror_points_topic")
    mirror_points_use_current_stamp = LaunchConfiguration("mirror_points_use_current_stamp")

    grid_resolution = LaunchConfiguration("grid_resolution")
    grid_density_threshold = LaunchConfiguration("grid_density_threshold")
    min_points = LaunchConfiguration("min_points")
    grid_update_period_sec = LaunchConfiguration("grid_update_period_sec")
    max_grid_cells = LaunchConfiguration("max_grid_cells")
    grid_reset_translation_thresh = LaunchConfiguration("grid_reset_translation_thresh")
    grid_reset_angle_deg = LaunchConfiguration("grid_reset_angle_deg")
    aggregated_marker_topic = LaunchConfiguration("aggregated_marker_topic")
    cluster_plane_topic = LaunchConfiguration("cluster_plane_topic")

    return LaunchDescription(
        [
            _declare("mask_topic", "/mask_image_processed", "Input mask topic"),
            _declare("camera_info_topic", "/camera/camera/color/camera_info", "CameraInfo topic"),
            _declare("plane_topic", "mirror_plane_clustered", "Clustered mirror plane topic"),
            _declare("use_sim_time", "false", "Use simulation clock"),
            _declare("target_frame", "map", "Target frame for projection"),
            _declare("projection_mode", "dense", "Projection mode: contour or dense"),
            _declare("pixel_stride", "2", "Pixel stride for dense projection"),
            _declare("point_scale", "0.03", "Marker scale for dense points"),
            _declare("projection_min_abs_denom", "1e-4", "Minimum |n·ray| for stable plane-ray intersection"),
            _declare("max_projection_distance", "10.0", "Maximum projection distance [m]"),
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
            _declare(
                "polygon_points_topic",
                "mirror_surface_projected_points_polygon",
                "PointCloud2 topic generated from aggregated mirror polygons",
            ),
            _declare(
                "polygon_point_stride_cells",
                "2",
                "Grid stride when sampling points from polygon interior",
            ),
            _declare(
                "mirror_points_topic",
                "mirror_surface_projected_points_xyz",
                "PointCloud2 topic for Nav2 (XYZ-only)",
            ),
            _declare(
                "mirror_points_use_current_stamp",
                "true",
                "Use current node time for Nav2 point cloud stamp to tolerate sensor timestamp skew",
            ),
            _declare("grid_resolution", "0.01", "Aggregator grid resolution [m]"),
            _declare("grid_density_threshold", "0.2", "Density threshold for polygon extraction"),
            _declare("min_points", "10", "Minimum points required before rendering"),
            _declare("grid_update_period_sec", "1.0", "Aggregator update period [s]"),
            _declare("max_grid_cells", "2000000", "Upper bound of grid cells before aggregator resets a cluster"),
            _declare("grid_reset_translation_thresh", "0.5", "Reset grid when cluster anchor shifts more than this [m]"),
            _declare("grid_reset_angle_deg", "15.0", "Reset grid when cluster normal changes more than this [deg]"),
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
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "target_frame": target_frame,
                        "projection_mode": projection_mode,
                        "pixel_stride": ParameterValue(pixel_stride, value_type=int),
                        "point_scale": ParameterValue(point_scale, value_type=float),
                        "projection_min_abs_denom": ParameterValue(
                            projection_min_abs_denom, value_type=float
                        ),
                        "max_projection_distance": ParameterValue(
                            max_projection_distance, value_type=float
                        ),
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
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "marker_topic": aggregated_marker_topic,
                        "target_frame": target_frame,
                        "polygon_points_topic": polygon_points_topic,
                        "polygon_point_stride_cells": ParameterValue(
                            polygon_point_stride_cells, value_type=int
                        ),
                        "publish_polygon_pointcloud": True,
                        "grid_resolution": ParameterValue(grid_resolution, value_type=float),
                        "grid_density_threshold": ParameterValue(
                            grid_density_threshold, value_type=float
                        ),
                        "min_points": ParameterValue(min_points, value_type=int),
                        "grid_update_period_sec": ParameterValue(
                            grid_update_period_sec, value_type=float
                        ),
                        "max_grid_cells": ParameterValue(max_grid_cells, value_type=int),
                        "grid_reset_translation_thresh": ParameterValue(
                            grid_reset_translation_thresh, value_type=float
                        ),
                        "grid_reset_angle_deg": ParameterValue(grid_reset_angle_deg, value_type=float),
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="mirror_pointcloud_filter_node",
                name="mirror_pointcloud_filter_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": polygon_points_topic,
                        "output_topic": mirror_points_topic,
                        "use_current_stamp": ParameterValue(
                            mirror_points_use_current_stamp, value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
