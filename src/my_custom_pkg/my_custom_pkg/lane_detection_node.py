#!/usr/bin/env python3
"""
lane_detection_node
===================
카메라 영상에서 차선을 감지하고 조향 오차를 계산하는 노드.

기존 camera_perception_pkg의 lane_info_extractor + path_planner + motion_planner
파이프라인을 하나로 통합한 간소화 버전.

Subscribed Topics:
  /camera/image_raw  (sensor_msgs/Image)  — 카메라 raw 영상
  또는
  yolov8_lane_info   (interfaces_pkg/LaneInfo) — YOLO 차선 정보 (선택)

Published Topics:
  /lane/steering  (std_msgs/Float64)  — 차선 기반 조향 (-7 ~ +7)
  /lane/speed     (std_msgs/Float64)  — 차선 주행 속도
  /lane/active    (std_msgs/Bool)     — 차선 감지 활성 여부

mission_controller_node가 이 토픽을 구독하여 최종 cmd를 결정합니다.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSDurabilityPolicy, QoSReliabilityPolicy,
)
from std_msgs.msg import Float64, Bool
from interfaces_pkg.msg import LaneInfo, PathPlanningResult

from .lib.pid_controller import PIDController


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # ═══════ 파라미터 ═══════
        self.sub_lane_topic = self.declare_parameter(
            'sub_lane_topic', 'yolov8_lane_info').value
        self.sub_path_topic = self.declare_parameter(
            'sub_path_topic', 'path_planning_result').value
        self.timer_period = float(
            self.declare_parameter('timer_period', 0.1).value)

        # 카메라/BEV 기준점 (픽셀)
        self.car_center_x = int(
            self.declare_parameter('car_center_x', 269).value)
        self.car_center_y = int(
            self.declare_parameter('car_center_y', 440).value)
        self.lookahead_distance = float(
            self.declare_parameter('lookahead_distance', 50.0).value)

        # PID
        self.kp = float(self.declare_parameter('kp', 30.0).value)
        self.ki = float(self.declare_parameter('ki', 0.0).value)
        self.kd = float(self.declare_parameter('kd', 2.0).value)
        self.pid = PIDController(self.kp, self.ki, self.kd, setpoint=0.0)

        # 스무딩
        self.smoothing_factor = float(
            self.declare_parameter('smoothing_factor', 0.15).value)
        self.max_steer = float(
            self.declare_parameter('max_steer', 7.0).value)

        # 속도
        self.default_speed = float(
            self.declare_parameter('default_speed', 100.0).value)

        # 내부 상태
        self.path_data = None
        self.lane_info = None
        self.steering_cmd = 0.0
        self.last_lane_time = None
        self.lane_timeout_sec = float(
            self.declare_parameter('lane_timeout_sec', 1.0).value)

        # ═══════ QoS ═══════
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_best = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)

        # ═══════ 퍼블리셔 ═══════
        self.pub_steering = self.create_publisher(
            Float64, '/lane/steering', qos_rel)
        self.pub_speed = self.create_publisher(
            Float64, '/lane/speed', qos_rel)
        self.pub_active = self.create_publisher(
            Bool, '/lane/active', qos_rel)

        # ═══════ 서브스크라이버 ═══════
        # path_planning_result 토픽 (경로점 배열)
        self.create_subscription(
            PathPlanningResult, self.sub_path_topic,
            self._path_cb, qos_best)

        # lane_info 토픽 (차선 slope + target_points)
        self.create_subscription(
            LaneInfo, self.sub_lane_topic,
            self._lane_cb, qos_best)

        # ═══════ 타이머 ═══════
        self.timer = self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info('LaneDetectionNode 시작')

    # ────────── 콜백 ──────────
    def _path_cb(self, msg: PathPlanningResult):
        """경로 계획 결과 수신"""
        if len(msg.x_points) >= 2:
            self.path_data = list(zip(msg.x_points, msg.y_points))
            self.last_lane_time = self.get_clock().now()
        else:
            self.path_data = None

    def _lane_cb(self, msg: LaneInfo):
        """차선 정보 직접 수신 (path_planner 없이 사용할 때)"""
        self.lane_info = msg
        self.last_lane_time = self.get_clock().now()

    # ────────── 차선 활성 여부 ──────────
    def _is_lane_active(self) -> bool:
        if self.last_lane_time is None:
            return False
        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        return elapsed < self.lane_timeout_sec

    # ────────── 제어 루프 ──────────
    def _control_loop(self):
        active = self._is_lane_active()

        # Bool 퍼블리시 (항상)
        b = Bool()
        b.data = active
        self.pub_active.publish(b)

        if not active:
            # 차선 미감지 — steering/speed = 0
            self._pub_values(0.0, 0.0)
            return

        target_steering = 0.0
        speed = self.default_speed

        # path_planning_result 기반 조향 계산 (우선)
        if self.path_data is not None and len(self.path_data) >= 2:
            goal = self._find_lookahead_point()

            alpha = math.atan2(
                goal[0] - self.car_center_x,
                self.car_center_y - goal[1])

            self.pid.setpoint = 0.0
            target_steering = self.pid.update(-alpha, self.timer_period)
            target_steering = max(-self.max_steer,
                                  min(self.max_steer, target_steering))

        # 저역통과 필터
        self.steering_cmd = (
            self.smoothing_factor * target_steering
            + (1.0 - self.smoothing_factor) * self.steering_cmd)

        self._pub_values(self.steering_cmd, speed)

        self.get_logger().info(
            f'[Lane] steer={self.steering_cmd:.2f} spd={speed:.0f}',
            throttle_duration_sec=0.5)

    def _find_lookahead_point(self):
        """lookahead_distance 이상 떨어진 경로점 탐색"""
        for pt in reversed(self.path_data):
            dx = pt[0] - self.car_center_x
            dy = self.car_center_y - pt[1]
            if math.hypot(dx, dy) >= self.lookahead_distance:
                return pt
        return self.path_data[0]

    def _pub_values(self, steering: float, speed: float):
        s = Float64(); s.data = float(steering)
        self.pub_steering.publish(s)
        v = Float64(); v.data = float(speed)
        self.pub_speed.publish(v)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
