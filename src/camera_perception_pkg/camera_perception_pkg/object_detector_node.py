import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty  # 감지된 객체 정보를 문자열 리스트로 발행
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.camera_position = self.declare_parameter('camera_position', 'left').value
        
        self.bridge = CvBridge()
        self.detection_order = []  # 감지된 객체 순서를 저장할 리스트

        # HSV 색상 범위 정의 (실제 환경에 맞게 튜닝 필요)
        self.hsv_ranges = {
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'blue': (np.array([100, 150, 50]), np.array([140, 255, 255])),
            'red': (np.array([0, 110, 48]), np.array([14, 255, 255]))
        }
        
        self.create_subscription(Image, f'/camera/{self.camera_position}/image_raw', self.image_callback, 10)
        self.create_subscription(Empty, f'/object/{self.camera_position}/reset', self.reset_callback, 10)
        
        self.publisher_ = self.create_publisher(String, f'/object/{self.camera_position}/detection_order', 10)
        self.get_logger().info(f"Object detector for '{self.camera_position}' camera started.")

    def reset_callback(self, msg):
        """탐지 기록을 초기화하는 콜백 함수"""
        if self.detection_order:
            self.get_logger().info(f"[{self.camera_position}] Resetting detection order. Was: {self.detection_order}")
            self.detection_order = []

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_images = {'original': cv_image}
        for color, (lower, upper) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv_image, lower, upper)
            debug_images[f'{color}_mask'] = mask
            if cv2.countNonZero(mask) > 2000:  # 임계값 튜닝 필요
                if color not in self.detection_order:
                    self.get_logger().info(f"[{self.camera_position}] New object detected: {color}")
                    self.detection_order.append(color)

        # 감지된 순서를 문자열로 변환하여 발행 (예: "blue,yellow")
        order_str = ",".join(self.detection_order)
        pub_msg = String()
        pub_msg.data = order_str
        self.publisher_.publish(pub_msg)
        #for name, image in debug_images.items():
        #    cv2.imshow(f"[{self.camera_position}] {name}", image)
        #cv2.waitKey(1)
# main 함수는 cone_detector_node와 동일하게 작성
def main(args=None):
    rclpy.init(args=args)
    cone_detector = ObjectDetectorNode()
    rclpy.spin(cone_detector)
    cone_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()