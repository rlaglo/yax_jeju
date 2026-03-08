import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 토픽 이름

# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_obstacle_info'  # 물체 감지 여부를 퍼블리시할 토픽 이름
#----------------------------------------------


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(
            LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)
        self.publisher = self.create_publisher(
            Bool, PUB_TOPIC_NAME, self.qos_profile) 

        self.detection_checker = LPFL.StabilityDetector(consec_count=3)


    def lidar_callback(self, msg):
        start_angle_deg = -10  # 차량 정면 좌측 30도
        end_angle_deg = 10     # 차량 정면 우측 30도
        range_min_m = 0.5     # 최소 감지 거리 (15cm)
        range_max_m = 2.0      # 최대 감지 거리 (2m)

        detected = False

        try:
            # 2. 각도를 Radian으로 변환
            start_angle_rad = np.deg2rad(start_angle_deg)
            end_angle_rad = np.deg2rad(end_angle_deg)

            # 3. LaserScan 메시지 정보를 이용해 검사할 인덱스 범위 계산
            # (시작 각도 - 스캔 최소 각도) / 각도 해상도
            start_index = int((start_angle_rad - msg.angle_min) / msg.angle_increment)
            end_index = int((end_angle_rad - msg.angle_min) / msg.angle_increment)
            
            # 인덱스가 배열 범위를 벗어나지 않도록 보정
            start_index = max(0, start_index)
            end_index = min(len(msg.ranges) - 1, end_index)

            # 4. 해당 인덱스 범위 내에서 장애물 검사
            for i in range(start_index, end_index + 1):
                # inf, nan 같은 유효하지 않은 값은 무시
                if np.isfinite(msg.ranges[i]) and range_min_m <= msg.ranges[i] <= range_max_m:
                    detected = True
                    break  # 장애물 발견 시 즉시 반복 중단

        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")
            detected = False

        # 5. 연속 감지 확인 후 결과 발행
        detection_result = self.detection_checker.check_consecutive_detections(detected)
        detection_msg = Bool()
        detection_msg.data = detection_result
        self.publisher.publish(detection_msg)

        self.get_logger().info(f'Lidar Obstacle detected: {detection_result}')

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
