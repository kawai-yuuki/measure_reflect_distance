from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rgb_topic = LaunchConfiguration("rgb_topic")
    raw_mask_topic = LaunchConfiguration("raw_mask_topic")
    processed_mask_topic = LaunchConfiguration("processed_mask_topic")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    sync_slop = LaunchConfiguration("sync_slop")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_topic",
                default_value=TextSubstitution(text="/camera/camera/color/image_raw"),
                description="RGB image topic consumed by the U-Net inference node.",
            ),
            DeclareLaunchArgument(
                "raw_mask_topic",
                default_value=TextSubstitution(text="/mask_image"),
                description="Intermediate mask topic published by the inference node.",
            ),
            DeclareLaunchArgument(
                "processed_mask_topic",
                default_value=TextSubstitution(text="/mask_image_processed"),
                description="Output mask topic after header alignment and post-processing.",
            ),
            DeclareLaunchArgument(
                "sync_queue_size",
                default_value=TextSubstitution(text="10"),
                description="Queue size handed to ApproximateTimeSynchronizer.",
            ),
            DeclareLaunchArgument(
                "sync_slop",
                default_value=TextSubstitution(text="0.05"),
                description="Allowed timestamp delta (seconds) for synchronized RGB and mask frames.",
            ),
            Node(
                package="measure_reflect_distance",
                executable="unet_inference_node",
                name="unet_inference_node",
                output="screen",
                remappings=[
                    ("/camera/camera/color/image_raw", rgb_topic),
                    ("/mask_image", raw_mask_topic),
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="mask_post_processor_node",
                name="mask_post_processor_node",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": rgb_topic,
                        "mask_topic": raw_mask_topic,
                        "output_topic": processed_mask_topic,
                        "sync_queue_size": ParameterValue(
                            sync_queue_size, value_type=int
                        ),
                        "sync_slop": ParameterValue(sync_slop, value_type=float),
                    }
                ],
            ),
        ]
    )
