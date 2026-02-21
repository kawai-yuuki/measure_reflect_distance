import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    from camera_info_manager import CameraInfoManager
except Exception:
    CameraInfoManager = None


class CameraInfoSync(Node):
    def __init__(self):
        super().__init__('camera_info_sync')

        self.declare_parameter('image_topic', '/camera/image_rgb')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('source_camera_info_topic', '')
        self.declare_parameter('camera_info_url', 'file:///path/to/calibration.yaml')
        self.declare_parameter('force_frame_id', '')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._camera_info_topic = (
            self.get_parameter('camera_info_topic').get_parameter_value().string_value
        )
        self._source_camera_info_topic = (
            self.get_parameter('source_camera_info_topic').get_parameter_value().string_value
        )
        caminfo_url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        self._force_frame_id = self.get_parameter('force_frame_id').get_parameter_value().string_value

        self._cim = None
        self._has_yaml_info = False
        if CameraInfoManager is not None:
            self._cim = CameraInfoManager(self, url=caminfo_url)
            self._has_yaml_info = self._cim.loadCameraInfo()
        self._latest_source_info = None
        self._warned_no_source = False

        self._pub = self.create_publisher(CameraInfo, self._camera_info_topic, 10)
        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self._source_sub = None
        if self._source_camera_info_topic:
            self._source_sub = self.create_subscription(
                CameraInfo,
                self._source_camera_info_topic,
                self.on_source_camera_info,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f'camera_info_sync started: image_topic={image_topic}, '
            f'camera_info_topic={self._camera_info_topic}, '
            f'source_camera_info_topic={self._source_camera_info_topic or "(none)"}'
        )
        if CameraInfoManager is None:
            self.get_logger().warn(
                'camera_info_manager module is not installed. '
                'sync_camera_info will run in source_camera_info_topic-only mode.'
            )
        if not self._source_camera_info_topic and not self._has_yaml_info:
            self.get_logger().warn(f'Failed to load camera info from {caminfo_url}')

    def on_source_camera_info(self, msg: CameraInfo):
        self._latest_source_info = msg

    def on_image(self, img: Image):
        if self._latest_source_info is not None:
            info = copy.deepcopy(self._latest_source_info)
        elif self._has_yaml_info and self._cim is not None:
            info = self._cim.getCameraInfo()
        else:
            if not self._warned_no_source:
                self.get_logger().warn(
                    'No source CameraInfo received yet and camera_info_url is unavailable.'
                )
                self._warned_no_source = True
            return

        info.width = img.width
        info.height = img.height
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
