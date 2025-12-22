#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as T
import cv2
import numpy as np
import os
import time
from std_msgs.msg import Float32

from measure_reflect_distance.util.unet_model import UNet
 
 
class UNetInferenceNode(Node):
    def __init__(self):
        super().__init__("unet_inference_node")
        self.bridge = CvBridge()

        # --- Parameters ---
        # self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("image_topic", "/camera/unet/image_raw")
        self.declare_parameter("mask_topic", "/mask_image")
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("profile_enabled", False)
        self.declare_parameter("profile_interval", 30)
        self.declare_parameter("inference_time_topic", "/unet/inference_time_ms")
        username = os.environ.get("USER") or os.path.basename(os.path.expanduser("~"))
        default_model_path = os.path.join(
            "/media",
            username,
            "KIOXIA",
            "segmentation_model",
            "mirror",
            "saved_model",
            "original_dataset",
            "best.pt",
        )
        self.declare_parameter("model_path", default_model_path)

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = UNet(n_channels=3, n_classes=1, bilinear=True)
        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"UNet weight file not found at: {model_path}. "
                "Set the 'model_path' parameter to the correct location."
            )
        self.get_logger().info(f"Loading UNet weights from {model_path}")

        state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model.to(self.device)
        self.model.eval()

        # 画像をリサイズし、正規化
        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((512, 512)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        from rclpy.qos import qos_profile_sensor_data

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value

        profile_enabled = self.get_parameter("profile_enabled").value
        if isinstance(profile_enabled, str):
            profile_enabled = profile_enabled.strip().lower() in ("1", "true", "yes", "on")
        self.profile_enabled = bool(profile_enabled)
        profile_interval = self.get_parameter("profile_interval").value
        try:
            self.profile_interval = int(profile_interval)
        except (TypeError, ValueError):
            self.profile_interval = 30
        self.profile_interval = max(self.profile_interval, 1)
        self.inference_time_topic = (
            self.get_parameter("inference_time_topic").get_parameter_value().string_value
        )
        self._profile_frame_count = 0
        self._inference_time_pub = None
        if self.profile_enabled and self.inference_time_topic:
            self._inference_time_pub = self.create_publisher(
                Float32, self.inference_time_topic, 10
            )

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.publisher_ = self.create_publisher(Image, mask_topic, 10)

        self.max_fps = float(self.get_parameter("max_fps").get_parameter_value().double_value)
        if self.max_fps <= 0.0:
            self.min_interval_ns = 0
        else:
            self.min_interval_ns = int(1e9 / self.max_fps)
        self._last_processed_time = None

        # 入力画像と出力マスクのfpsを計算
        self.input_frame_count = 0
        self.output_frame_count = 0
        self.last_fps_time = self.get_clock().now()

        # 5秒ごとにlog_fps関数を呼び出すタイマーを作成
        self.fps_timer = self.create_timer(5.0, self.log_fps)

        self.get_logger().info(
            f"UNet Inference Node has been started (image_topic={image_topic}, "
            f"mask_topic={mask_topic}, max_fps={self.max_fps:.2f}, "
            f"profile_enabled={self.profile_enabled})"
        )

    def log_fps(self):
        current_time = self.get_clock().now()
        elapsed_seconds = (current_time - self.last_fps_time).nanoseconds / 1e9

        # ゼロ除算を回避
        if elapsed_seconds > 0:
            input_fps = self.input_frame_count / elapsed_seconds
            output_fps = self.output_frame_count / elapsed_seconds

            self.get_logger().info(f"FPS -> Input: {input_fps:.2f}, Output: {output_fps:.2f}")
        
        self.input_frame_count = 0
        self.output_frame_count = 0
        self.last_fps_time = current_time

    def image_callback(self, msg):
        self.input_frame_count += 1
        try:
            if self.min_interval_ns > 0:
                now = self.get_clock().now()
                if self._last_processed_time is not None:
                    elapsed_ns = (now - self._last_processed_time).nanoseconds
                    if elapsed_ns < self.min_interval_ns:
                        return
                self._last_processed_time = now

            # ROS Imageから OpenCVのImageに変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            H, W = cv_image.shape[:2]

            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 前処理
            input_tensor = self.preprocess(rgb_image).unsqueeze(0).to(self.device)

            # 推論
            if self.profile_enabled and self.device.type == "cuda":
                torch.cuda.synchronize()
            infer_start = time.perf_counter() if self.profile_enabled else None
            with torch.no_grad():
                output = self.model(input_tensor)
            if self.profile_enabled:
                if self.device.type == "cuda":
                    torch.cuda.synchronize()
                infer_ms = (time.perf_counter() - infer_start) * 1000.0
                if self._inference_time_pub is not None:
                    self._inference_time_pub.publish(Float32(data=float(infer_ms)))
                self._profile_frame_count += 1
                if self._profile_frame_count % self.profile_interval == 0:
                    self.get_logger().info(f"Inference time: {infer_ms:.2f} ms")
            
            # min_val = output.min().item()
            # max_val = output.max().item()
            # mean_val = output.mean().item()
            # self.get_logger().info(f"Model Output Stats -> Min: {min_val:.4f}, Max: {max_val:.4f}, Mean: {mean_val:.4f}")

            # 後処理
            output_mask = self.postprocess(output, (W, H))

            # マスクをROS Imageとしてパブリッシュ
            output_msg = self.bridge.cv2_to_imgmsg(output_mask, encoding='mono8')
            output_msg.header = msg.header  # 元の画像のヘッダーをコピー
            # self.get_logger().info("Publishing output mask")
            self.publisher_.publish(output_msg)

            self.output_frame_count += 1
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def preprocess(self, image):
        return self.transform(image)

    def postprocess(self, output_tensor, orig_size):
        prob = output_tensor.squeeze().cpu()

        mask = (prob > 0.5).numpy().astype(np.uint8)

        mask = mask * 255  # 0-1を0-255に変換
        mask = cv2.resize(mask, orig_size, interpolation=cv2.INTER_NEAREST)
        return mask

def main(args=None):
    rclpy.init(args=args)
    node = UNetInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
