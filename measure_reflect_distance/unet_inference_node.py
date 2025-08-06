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
        self.model = UNet(n_channels=3, n_classes=1, bilinear=True)
        model_path = os.path.expanduser('~/U-Net/U-Net_model/20250726/mirror/best.pt')
        state_dict = torch.load(model_path, map_location=torch.device('cuda:0'), weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model.eval().cuda()

        self.subscription = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        # 画像をリサイズし、正規化
        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((512, 512)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.publisher_ = self.create_publisher(Image, '/mask_image', 10)
    
    def image_callback(self, msg):
        # ROS Imageから OpenCVのImageに変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W = cv_image.shape[:2]

        # 前処理
        input_tensor = self.preprocess(cv_image).unsqueeze(0).cuda()

        # 推論
        with torch.no_grad():
            output = self.model(input_tensor)
        
        # 後処理
        output_mask = self.postprocess(output, (W, H))

        # マスクをROS Imageとしてパブリッシュ
        output_msg = self.bridge.cv2_to_imgmsg(output_mask, encoding='mono8')
        output_msg.header = msg.header  # 元の画像のヘッダーをコピー
        self.get_logger().info("Publishing output mask")
        self.publisher_.publish(output_msg)
    
    def preprocess(self, image):
        return self.transform(image)

    def postprocess(self, output_tensor, orig_size):
        logits = output_tensor.squeeze(0).cpu()
        if logits.ndim == 2:
            prob = torch.sigmoid(logits)
            mask = (prob > 0.5).numpy().astype(np.uint8)
        else:
            mask = torch.argmax(logits, dim=0).numpy().astype(np.uint8)
        
        mask = (mask * 255).astype(np.uint8)  # 0-1を0-255に変換
        mask = cv2.resize(mask, orig_size, interpolation=cv2.INTER_NEAREST)
        return mask

def main(args=None):
    rclpy.init(args=args)
    node = UNetInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()