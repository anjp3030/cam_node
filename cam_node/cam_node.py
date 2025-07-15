#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # 파라미터 선언 (기본값 설정)
        self.declare_parameter("device_path", "/dev/v4l/by-id/usb-HD_Camera_HD_Camera-video-index0")
        self.declare_parameter("camera_name", "uv570")

        # 파라미터 값 읽기
        device_path = self.get_parameter("device_path").get_parameter_value().string_value
        camera_name = self.get_parameter("camera_name").get_parameter_value().string_value

        self.get_logger().info(f"카메라 노드 시작 - 이름: {camera_name}, 디바이스: {device_path}")
        publish_topic = f"/{camera_name}/image_raw"

        self.publisher_ = self.create_publisher(Image, publish_topic, 10)
        self.bridge = CvBridge()

        # 카메라 장치 열기
        self.cap = cv2.VideoCapture(device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"{camera_name} 카메라를 열 수 없습니다! 경로: {device_path}")
            return

        # 타이머 설정: 주기는 1 / frame_rate (초)
        period = 1.0 / 30
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임을 읽지 못했습니다.")
            return

        # OpenCV 이미지를 ROS2 Image 메시지로 변환 (인코딩은 필요에 따라 조정)
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(image_msg)
        # self.get_logger().info("이미지 프레임을 publish 했습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    if not node.cap.isOpened():
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
