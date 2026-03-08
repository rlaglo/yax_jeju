import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
import numpy as np
import math

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand, Point2D
from .lib.pid_controller import PIDController

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_TRAFFIC_LIGHT_BBOX_TOPIC_NAME = "yolov8_traffic_light_detections"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"
#----------------------------------------------

TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.image_height = self.declare_parameter('image_height', 480).value  # 실제 카메라 높이로 바꾸세요
        self.stop_y_threshold = self.declare_parameter('stop_y_threshold', 80.0).value
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # --- 제어 파라미터 ---
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 50.0).value # 픽셀 단위
        self.wheel_base = self.declare_parameter('wheel_base', 0.4).value # 차량 축거 (미터 단위)
        self.smoothing_factor = self.declare_parameter('smoothing_factor', 0.15).value
        
        # --- PID 게인 값 (튜닝 필요) ---
        self.kp = self.declare_parameter('kp', 30.0).value # P 게인, 오차에 대한 반응성
        self.ki = self.declare_parameter('ki', 0.0).value  # I 게인, 누적 오차 보정
        self.kd = self.declare_parameter('kd', 2.0).value  # D 게인, 급격한 변화 억제 (진동 감소)
        
        # --- PID 컨트롤러 객체 생성 (주석 해제 및 수정) ---
        self.pid_controller = PIDController(self.kp, self.ki, self.kd, setpoint=0.0)

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_state = "None"
        self.traffic_light_detections = []
        self.lidar_data = None
        self.steering_command = 0.0
        self.left_speed_command = 0
        self.right_speed_command = 0
        
        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.traffic_light_bbox_sub = self.create_subscription(
            DetectionArray, SUB_TRAFFIC_LIGHT_BBOX_TOPIC_NAME, self.traffic_light_bbox_callback, self.qos_profile)    
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        self.goal_point_publisher = self.create_publisher(Point2D, 'goal_point', self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Motion Planner Node has been initialized.")

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_state = msg.data
    def traffic_light_bbox_callback(self, msg: DetectionArray):
        self.traffic_light_detections = msg.detections

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):
        target_steering = 0.0
        alpha = 0.0  # alpha를 0으로 초기화
        
        # 기본 주행 속도 설정
        left_speed = 100
        right_speed = 100

        # 경로 추종 로직
        if self.path_data is not None and len(self.path_data) >= 2:
            car_center_x = 269
            car_center_y = 440
            
            goal_point = None
            for point in reversed(self.path_data):
                dx = point[0] - car_center_x
                dy = car_center_y - point[1]
                distance = math.sqrt(dx**2 + dy**2)
                if distance >= self.lookahead_distance:
                    goal_point = point
                    break
            
            if goal_point is None:
                goal_point = self.path_data[0]

            goal_point_msg = Point2D()
            goal_point_msg.x = float(goal_point[0])
            goal_point_msg.y = float(goal_point[1])
            self.goal_point_publisher.publish(goal_point_msg)

            alpha = math.atan2(goal_point[0] - car_center_x, car_center_y - goal_point[1])
            
            self.pid_controller.setpoint = 0.0
            target_steering = self.pid_controller.update(-alpha)
            
            max_steer = 7.0
            target_steering = np.clip(target_steering, -max_steer, max_steer)
        else:
            # 경로가 없으면 조향각 0
            target_steering = 0.0
        is_stop_signal = self.traffic_light_state.lower() in ["red", "yellow"]
        num_boxes = len(self.traffic_light_detections) if self.traffic_light_detections else 0
        self.get_logger().warn(f"[TL] stop? {is_stop_signal}, #boxes={num_boxes}")
        should_stop = False
        dbg = []
        if is_stop_signal and num_boxes > 0:
            for det in self.traffic_light_detections:
                yc = float(det.bbox.center.position.y)
                h  = float(det.bbox.size.y)
                y_max = yc + 0.5*h
                # 정규화 좌표(0~1)로 보이면 픽셀로 환산
                y_max_px = y_max * self.image_height if 0.0 <= y_max <= 1.0 else y_max
                dbg.append(round(y_max_px, 1))
                if y_max_px >= self.stop_y_threshold:
                    should_stop = True
                    break
            self.get_logger().warn(f"[TL] y_max_px samples={dbg[:3]} (th={self.stop_y_threshold})")

        if should_stop:
            left_speed, right_speed = 0, 0
            target_steering = 0.0
        if self.lidar_data is not None and self.lidar_data.data is True:
            self.get_logger().warn("!!! LIDAR OBSTACLE DETECTED -> STOP !!!")
            target_steering = 0.0
            left_speed, right_speed = 0, 0

        # 저대역 통과 필터로 조향 명령을 부드럽게 만듦
        self.steering_command = (self.smoothing_factor * target_steering) + \
                                ((1.0 - self.smoothing_factor) * self.steering_command)

        # 최종 로그 출력
        self.get_logger().info(f"Alpha(deg): {np.rad2deg(alpha):.2f}, Target: {target_steering:.2f}, Final Cmd: {self.steering_command:.2f}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = int(self.steering_command)
        motion_command_msg.left_speed = left_speed
        motion_command_msg.right_speed = right_speed
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()