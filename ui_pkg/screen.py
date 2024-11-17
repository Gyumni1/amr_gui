import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from mss import mss  # mss 라이브러리를 사용한 화면 캡처

class ScreenCapturePublisher(Node):
    def __init__(self):
        super().__init__('screen_capture_publisher')
       
        # ROS2 퍼블리셔 설정
        self.publisher = self.create_publisher(Image, 'screen_image', 10)
        self.bridge = CvBridge()
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # mss를 사용해 화면 캡처 설정
        self.sct = mss()
        self.monitor = {
            "top": 0,  # 캡처할 영역의 y 좌표
            "left": 0,  # 캡처할 영역의 x 좌표
            "width": 1000,  # 캡처할 영역의 너비
            "height": 1000  # 캡처할 영역의 높이
        }
        self.monitor = self.sct.monitors[1]
        print(self.sct.monitors)

        # 타이머로 주기적인 캡처 설정
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 캡처

    def timer_callback(self):
        # 화면의 일부를 캡처
        screen_shot = self.sct.grab(self.monitor)
       
        # numpy 배열로 변환하고 BGR로 변환 (OpenCV 호환)
        img = np.array(screen_shot)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        print(img.shape)

        # OpenCV 이미지로 표시
        cv2.imshow("Captured Image", img)
        cv2.waitKey(10)  # 1ms 대기하여 이미지가 표시되도록 함
        # print(img.shape)

        # OpenCV 이미지 -> ROS2 이미지 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
       
        # ROS2 이미지 퍼블리시
        self.publisher.publish(ros_image)
        # self.get_logger().info('Publishing screen image')


def main(args=None):
    rclpy.init(args=args)
    screen_capture_publisher = ScreenCapturePublisher()
    rclpy.spin(screen_capture_publisher)
    screen_capture_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
