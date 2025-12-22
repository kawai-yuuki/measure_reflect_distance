from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rgb_topic = LaunchConfiguration("rgb_topic")
    relay_rgb_topic = LaunchConfiguration("relay_rgb_topic")
    relay_max_fps = LaunchConfiguration("relay_max_fps")
    unet_max_fps = LaunchConfiguration("unet_max_fps")
    raw_mask_topic = LaunchConfiguration("raw_mask_topic")
    processed_mask_topic = LaunchConfiguration("processed_mask_topic")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    sync_slop = LaunchConfiguration("sync_slop")
    model_path = LaunchConfiguration("model_path")
    profile_enabled = LaunchConfiguration("profile_enabled")
    profile_interval = LaunchConfiguration("profile_interval")
    torch_num_threads = LaunchConfiguration("torch_num_threads")
    torch_num_interop_threads = LaunchConfiguration("torch_num_interop_threads")
    opencv_num_threads = LaunchConfiguration("opencv_num_threads")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_topic",
                default_value=TextSubstitution(text="/camera/camera/color/image_raw"),
                description="Camera RGB topic to subscribe for the relay node.",
            ),
            DeclareLaunchArgument(
                "relay_rgb_topic",
                default_value=TextSubstitution(text="/camera/unet/image_raw"),
                description="RGB topic after the relay, consumed by UNet and mask post processor.",
            ),
            DeclareLaunchArgument(
                "relay_max_fps",
                default_value=TextSubstitution(text="0.0"),
                description="Maximum frequency (Hz) for the relay node output.",
            ),
            DeclareLaunchArgument(
                "unet_max_fps",
                default_value=TextSubstitution(text="0.0"),
                description="Optional additional throttling inside the UNet node (0 disables).",
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
                default_value=TextSubstitution(text="30"),
                description="Queue size handed to ApproximateTimeSynchronizer.",
            ),
            DeclareLaunchArgument(
                "sync_slop",
                default_value=TextSubstitution(text="0.05"),
                description="Allowed timestamp delta (seconds) for synchronized RGB and mask frames.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=[
                    TextSubstitution(text="/media/"),
                    EnvironmentVariable("USER"),
                    TextSubstitution(text="/KIOXIA/segmentation_model/mirror/saved_model/original_dataset/best.pt"),
                ],
                description="Path to the UNet weight file; defaults to the KIOXIA drive mounted under /media/<user>/KIOXIA.",
            ),
            DeclareLaunchArgument(
                "profile_enabled",
                default_value=TextSubstitution(text="true"),
                description="Enable per-frame profiling logs inside the UNet node.",
            ),
            DeclareLaunchArgument(
                "profile_interval",
                default_value=TextSubstitution(text="30"),
                description="How many processed frames between profiling log lines.",
            ),
            DeclareLaunchArgument(
                "torch_num_threads",
                default_value=TextSubstitution(text="2"),
                description="Number of intra-op PyTorch threads (torch.set_num_threads).",
            ),
            DeclareLaunchArgument(
                "torch_num_interop_threads",
                default_value=TextSubstitution(text="2"),
                description="Number of inter-op PyTorch threads (torch.set_num_interop_threads).",
            ),
            DeclareLaunchArgument(
                "opencv_num_threads",
                default_value=TextSubstitution(text="1"),
                description="Number of OpenCV threads (cv2.setNumThreads).",
            ),
            Node(
                package="measure_reflect_distance",
                executable="rgb_relay_node",
                name="rgb_relay_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": rgb_topic,
                        "output_topic": relay_rgb_topic,
                        "max_fps": ParameterValue(relay_max_fps, value_type=float),
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="unet_inference_node",
                name="unet_inference_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": relay_rgb_topic,
                        "mask_topic": raw_mask_topic,
                        "max_fps": ParameterValue(unet_max_fps, value_type=float),
                        "model_path": model_path,
                        "profile_enabled": profile_enabled,
                        "profile_interval": ParameterValue(profile_interval, value_type=int),
                        "torch_num_threads": ParameterValue(torch_num_threads, value_type=int),
                        "torch_num_interop_threads": ParameterValue(
                            torch_num_interop_threads, value_type=int
                        ),
                        "opencv_num_threads": ParameterValue(opencv_num_threads, value_type=int),
                    }
                ],
            ),
            Node(
                package="measure_reflect_distance",
                executable="mask_post_processor_node",
                name="mask_post_processor_node",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": relay_rgb_topic,
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
