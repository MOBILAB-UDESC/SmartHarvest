from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import (
    CameraInfo,
    Image,
)
from sh_interfaces.srv import DetectObjects


class TestDetectionClient(Node):
    def __init__(self):
        super().__init__('test_detection_client')
        self.client = self.create_client(DetectObjects, 'get_detections')
        self.bridge = CvBridge()

    def call_service(self, width=640, height=480):
        rgb_img = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

        depth_img = np.random.randint(500, 5000, (height, width), dtype=np.uint16)

        self.get_logger().info(f'Generated random RGB image: {rgb_img.shape}')
        self.get_logger().info(
            f'Generated random depth image: {
                depth_img.shape}, range: {
                depth_img.min()}-{
                depth_img.max()}'
        )

        request = DetectObjects.Request()
        request.rgb_image = self.bridge.cv2_to_imgmsg(rgb_img, 'bgr8')
        request.depth_image = self.bridge.cv2_to_imgmsg(depth_img, '16UC1')

        request.cam_info = CameraInfo()
        request.cam_info.header.frame_id = 'camera_optical_frame'
        request.cam_info.header.stamp = self.get_clock().now().to_msg()
        request.cam_info.width = width
        request.cam_info.height = height
        request.cam_info.k = [
            615.0, 0.0, 320.0,      # fx, 0, cx
            0.0, 615.0, 240.0,      # 0, fy, cy
            0.0, 0.0, 1.0           # 0, 0, 1
        ]

        self.get_logger().info('Waiting for detection service...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available!')
            return

        self.get_logger().info('Calling detection service...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Success: {response.success}')
            self.get_logger().info(f'Detected {response.detected_objects.num_objects} objects:')

            if response.detected_objects.num_objects > 0:
                for i, obj in enumerate(response.detected_objects.objects):
                    self.get_logger().info(
                        f'  [{i + 1}] Class {obj.class_id}: '
                        f'pos=({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})m, '
                        f'dist={obj.distance:.3f}m, '
                        f'conf={obj.confidence:.2%}'
                    )
            else:
                self.get_logger().info('  No objects detected (expected with random noise)')
        else:
            self.get_logger().error('Service call failed')


def main():
    rclpy.init()
    client = TestDetectionClient()

    client.call_service(width=1280, height=720)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
