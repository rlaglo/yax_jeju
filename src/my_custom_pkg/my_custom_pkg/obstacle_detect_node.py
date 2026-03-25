#!/usr/bin/env python3
"""
obstacle_detect_node
====================
LiDAR 기반 전방 장애물 감지 노드.

전방 ±N도 범위 내에 threshold 이하 거리의 물체가 있으면
장애물 감지 → 정지 명령을 발행합니다.

Subscribed Topics:
  /scan  (sensor_msgs/LaserScan) — LiDAR 원본 또는 전처리 스캔

Published Topics:
  /obstacle/detected  (std_msgs/Bool)     — 장애물 감지 여부
  /obstacle/distance  (std_msgs/Float64)  — 전방 최소 거리 (m)
  /obstacle/steering  (std_msgs/Float64)  — 장애물 회피 조향 (현재: 0, 향후 확장)
  /obstacle/speed     (std_msgs/Float64)  — 장애물 감지 시 0, 미감지 시 -1 (제어 없음)

mission_controller_node가 /obstacle/detected = True이면
모든 다른 소스보다 우선하여 정지합니다.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSDurabilityPolicy, QoSReliabilityPolicy,
)
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool


class ObstacleDetectNode(Node):
    def __init__(self):
        super().__init__('obstacle_detect_node')

        # ═══════ 파라미터 ═══════
        self.sub_lidar_topic = self.declare_parameter(
            'sub_lidar_topic', '/scan').value
        self.timer_period = float(
            self.declare_parameter('timer_period', 0.1).value)

        # 전방 감지 각도 범위 (deg, 0=정면)
        self.front_half_angle_deg = float(
            self.declare_parameter('front_half_angle_deg', 30.0).value)

        # 장애물 판정 거리 (m)
        self.stop_distance_m = float(
            self.declare_parameter('stop_distance_m', 2.0).value)

        # 안정적 감지를 위한 연속 감지 횟수
        self.consec_detect_threshold = int(
            self.declare_parameter('consec_detect_threshold', 3).value)
        self.consec_clear_threshold = int(
            self.declare_parameter('consec_clear_threshold', 3).value)

        # 내부 상태
        self.latest_scan: LaserScan = None
        self.obstacle_detected = False
        self._detect_cnt = 0
        self._clear_cnt = 0
        self.min_distance = float('inf')

        # ═══════ QoS ═══════
        qos_best = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)

        # ═══════ 퍼블리셔 ═══════
        self.pub_detected = self.create_publisher(
            Bool, '/obstacle/detected', qos_rel)
        self.pub_distance = self.create_publisher(
            Float64, '/obstacle/distance', qos_rel)
        self.pub_steering = self.create_publisher(
            Float64, '/obstacle/steering', qos_rel)
        self.pub_speed = self.create_publisher(
            Float64, '/obstacle/speed', qos_rel)

        # ═══════ 서브스크라이버 ═══════
        self.create_subscription(
            LaserScan, self.sub_lidar_topic,
            self._scan_cb, qos_best)

        # ═══════ 타이머 ═══════
        self.timer = self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info(
            f'ObstacleDetectNode 시작 '
            f'(전방 ±{self.front_half_angle_deg}°, '
            f'정지거리 {self.stop_distance_m}m)')

    # ────────── LiDAR 콜백 ──────────
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ────────── 전방 장애물 검사 ──────────
    def _check_front_obstacle(self, msg: LaserScan) -> tuple:
        """
        전방 영역의 최소 거리 반환.
        Returns: (is_close: bool, min_dist: float)
        """
        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return False, float('inf')

        half = math.radians(self.front_half_angle_deg)
        i0 = int((-half - msg.angle_min) / msg.angle_increment)
        i1 = int((half - msg.angle_min) / msg.angle_increment)
        i0 = max(0, min(n - 1, min(i0, i1)))
        i1 = max(0, min(n - 1, max(i0, i1)))

        front = ranges[i0:i1 + 1]
        valid = (np.isfinite(front)
                 & (front >= max(1e-3, msg.range_min))
                 & (front <= msg.range_max))

        if not np.any(valid):
            return False, float('inf')

        min_d = float(front[valid].min())
        return min_d < self.stop_distance_m, min_d

    # ────────── 제어 루프 ──────────
    def _control_loop(self):
        if self.latest_scan is None:
            self._pub_values(False, float('inf'))
            return

        is_close, min_d = self._check_front_obstacle(self.latest_scan)
        self.min_distance = min_d

        # 히스테리시스
        if is_close:
            self._detect_cnt += 1
            self._clear_cnt = 0
            if self._detect_cnt >= self.consec_detect_threshold:
                self.obstacle_detected = True
        else:
            self._clear_cnt += 1
            self._detect_cnt = 0
            if self._clear_cnt >= self.consec_clear_threshold:
                self.obstacle_detected = False

        self._pub_values(self.obstacle_detected, min_d)

        if self.obstacle_detected:
            self.get_logger().warn(
                f'[Obstacle] 감지! 최소거리={min_d:.2f}m < {self.stop_distance_m}m',
                throttle_duration_sec=0.5)

    # ────────── 퍼블리시 헬퍼 ──────────
    def _pub_values(self, detected: bool, distance: float):
        b = Bool(); b.data = detected
        self.pub_detected.publish(b)

        d = Float64(); d.data = distance
        self.pub_distance.publish(d)

        # 장애물 감지 시: 조향 0, 속도 0 (정지)
        # 미감지 시: 속도 -1 (= "제어권 없음" 표시)
        s = Float64(); s.data = 0.0
        self.pub_steering.publish(s)

        v = Float64()
        v.data = 0.0 if detected else -1.0
        self.pub_speed.publish(v)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
