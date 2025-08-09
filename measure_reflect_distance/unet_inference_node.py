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

from .unet_model import UNet
 
 
class UNetInferenceNode(Node):
    def __init__(self):
        super().__init__("unet_inference_node")
        self.bridge = CvBridge()

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = UNet(n_channels=3, n_classes=1, bilinear=True)
        model_path = os.path.expanduser('~/U-Net/U-Net_model/20250726/mirror/best.pt')

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

        self.subscription = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )

        self.publisher_ = self.create_publisher(Image, '/mask_image', 10)

        # 入力画像と出力マスクのfpsを計算
        self.input_frame_count = 0
        self.output_frame_count = 0
        self.last_fps_time = self.get_clock().now()

        # 5秒ごとにlog_fps関数を呼び出すタイマーを作成
        self.fps_timer = self.create_timer(5.0, self.log_fps)

        self.get_logger().info("UNet Inference Node has been started.")

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
            # ROS Imageから OpenCVのImageに変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            H, W = cv_image.shape[:2]

            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 前処理
            input_tensor = self.preprocess(rgb_image).unsqueeze(0).to(self.device)

            # 推論
            with torch.no_grad():
                output = self.model(input_tensor)
            
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