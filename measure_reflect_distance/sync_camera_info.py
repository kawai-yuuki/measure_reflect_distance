# sync_camera_info.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager  # pip/apt: camera_info_manager_py

class CameraInfoSync(Node):
    def __init__(self):
        super().__init__('camera_info_sync')

        # パラメータ
        self.declare_parameter('image_topic', '/camera/image_rgb')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_info_url', 'file:///path/to/calibration.yaml')
        self.declare_parameter('force_frame_id', '')  # 空ならImageのframe_idを使用

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        caminfo_url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        self._force_frame_id = self.get_parameter('force_frame_id').get_parameter_value().string_value

        # CameraInfo 読み込み
        self._cim = CameraInfoManager(self, url=caminfo_url)
        if not self._cim.loadCameraInfo():
            self.get_logger().warn(f'Failed to load camera info from {caminfo_url}')

        # Pub/Sub
        self._pub = self.create_publisher(CameraInfo, self._camera_info_topic, 10)
        self._sub = self.create_subscription(Image, image_topic, self.on_image, 10)

    def on_image(self, img: Image):
        info = self._cim.getCameraInfo()

        # 画像サイズに合わせる（画像をリサイズして出している場合は必須）
        info.width = img.width
        info.height = img.height

        # 重要：stamp/frame_id を画像に合わせる
        info.header.stamp = img.header.stamp
        info.header.frame_id = self._force_frame_id if self._force_frame_id else img.header.frame_id

        self._pub.publish(info)

def main():
    rclpy.init()
    node = CameraInfoSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
