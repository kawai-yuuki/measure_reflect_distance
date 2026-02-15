import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    username = os.environ.get("USER") or os.path.basename(os.path.expanduser("~"))
    default_model_path = os.path.join(
        "/media",
        username,
        "KIOXIA",
        "segmentation_model",
        "mirror",
        "outputs_unet_min2",
        "finetune_best.pt",
    )

    rgb_topic = LaunchConfiguration("rgb_topic")
    relay_rgb_topic = LaunchConfiguration("relay_rgb_topic")
    relay_max_fps = LaunchConfiguration("relay_max_fps")
    unet_max_fps = LaunchConfiguration("unet_max_fps")
    raw_mask_topic = LaunchConfiguration("raw_mask_topic")
    processed_mask_topic = LaunchConfiguration("processed_mask_topic")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    sync_slop = LaunchConfiguration("sync_slop")
    binarize_threshold = LaunchConfiguration("binarize_threshold")
    binarize_max_value = LaunchConfiguration("binarize_max_value")
    morph_operation = LaunchConfiguration("morph_operation")
    morph_kernel_shape = LaunchConfiguration("morph_kernel_shape")
    morph_kernel_width = LaunchConfiguration("morph_kernel_width")
    morph_kernel_height = LaunchConfiguration("morph_kernel_height")
    morph_iterations = LaunchConfiguration("morph_iterations")
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
                "binarize_threshold",
                default_value=TextSubstitution(text="127"),
                description="Threshold for mask binarization (0-255).",
            ),
            DeclareLaunchArgument(
                "binarize_max_value",
                default_value=TextSubstitution(text="255"),
                description="Max value assigned to foreground pixels after binarization (1-255).",
            ),
            DeclareLaunchArgument(
                "morph_operation",
                default_value=TextSubstitution(text="close"),
                description="Morphology op: none|close|open|erode|dilate.",
            ),
            DeclareLaunchArgument(
                "morph_kernel_shape",
                default_value=TextSubstitution(text="rect"),
                description="Kernel shape: rect|ellipse|cross.",
            ),
            DeclareLaunchArgument(
                "morph_kernel_width",
                default_value=TextSubstitution(text="9"),
                description="Kernel width for morphological closing.",
            ),
            DeclareLaunchArgument(
                "morph_kernel_height",
                default_value=TextSubstitution(text="9"),
                description="Kernel height for morphological closing.",
            ),
            DeclareLaunchArgument(
                "morph_iterations",
                default_value=TextSubstitution(text="2"),
                description="Number of closing iterations (0 disables closing).",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=TextSubstitution(text=default_model_path),
                description="Path to the UNet weight file.",
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
            # Node(
            #     package="measure_reflect_distance",
            #     executable="rgb_relay_node",
            #     name="rgb_relay_node",
            #     output="screen",
            #     parameters=[
            #         {
            #             "input_topic": rgb_topic,
            #             "output_topic": relay_rgb_topic,
            #             "max_fps": ParameterValue(relay_max_fps, value_type=float),
            #         }
            #     ],
            # ),
            Node(
                package="measure_reflect_distance",
                executable="unet_inference_node",
                name="unet_inference_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": rgb_topic,
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
                        "rgb_topic": rgb_topic,
                        "mask_topic": raw_mask_topic,
                        "output_topic": processed_mask_topic,
                        "sync_queue_size": ParameterValue(
                            sync_queue_size, value_type=int
                        ),
                        "sync_slop": ParameterValue(sync_slop, value_type=float),
                        "binarize_threshold": ParameterValue(
                            binarize_threshold, value_type=int
                        ),
                        "binarize_max_value": ParameterValue(
                            binarize_max_value, value_type=int
                        ),
                        "morph_operation": morph_operation,
                        "morph_kernel_shape": morph_kernel_shape,
                        "morph_kernel_width": ParameterValue(
                            morph_kernel_width, value_type=int
                        ),
                        "morph_kernel_height": ParameterValue(
                            morph_kernel_height, value_type=int
                        ),
                        "morph_iterations": ParameterValue(
                            morph_iterations, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
