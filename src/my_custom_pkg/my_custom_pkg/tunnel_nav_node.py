#!/usr/bin/env python3
"""
tunnel_nav_node
===============
LiDAR 기반 터널/좁은 통로 주행 노드.

터널 내부에서는 GPS 신호가 불안정하므로, LiDAR 스캔 데이터를 활용하여
좌/우 벽 거리를 측정하고 중앙을 유지하도록 조향합니다.

Subscribed Topics:
  /scan  (sensor_msgs/LaserScan) — LiDAR 스캔 데이터

Published Topics:
  /tunnel/steering  (std_msgs/Float64)  — 터널 내 조향 (-7 ~ +7)
  /tunnel/speed     (std_msgs/Float64)  — 터널 내 속도
  /tunnel/active    (std_msgs/Bool)     — 터널 감지 여부

터널 감지 로직:
  좌/우 일정 각도 범위의 평균 거리가 모두 threshold 이하이면
  "터널 내부"로 판단하여 active = True.
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


class TunnelNavNode(Node):
    def __init__(self):
        super().__init__('tunnel_nav_node')

        # ═══════ 파라미터 ═══════
        self.sub_lidar_topic = self.declare_parameter(
            'sub_lidar_topic', '/scan').value
        self.timer_period = float(
            self.declare_parameter('timer_period', 0.1).value)

        # 터널 감지 기준
        self.tunnel_wall_dist_m = float(
            self.declare_parameter('tunnel_wall_dist_m', 3.0).value)
        self.tunnel_min_points = int(
            self.declare_parameter('tunnel_min_points', 5).value)

        # 좌/우 벽 스캔 각도 (deg)
        self.left_angle_min_deg = float(
            self.declare_parameter('left_angle_min_deg', 60.0).value)
        self.left_angle_max_deg = float(
            self.declare_parameter('left_angle_max_deg', 120.0).value)
        self.right_angle_min_deg = float(
            self.declare_parameter('right_angle_min_deg', -120.0).value)
        self.right_angle_max_deg = float(
            self.declare_parameter('right_angle_max_deg', -60.0).value)

        # 조향 게인
        self.k_tunnel = float(
            self.declare_parameter('k_tunnel', 2.0).value)
        self.max_steering = float(
            self.declare_parameter('max_steering', 7.0).value)
        self.tunnel_speed = float(
            self.declare_parameter('tunnel_speed', 150.0).value)

        # 터널 상태 유지를 위한 연속 감지 카운터
        self.entry_count_threshold = int(
            self.declare_parameter('entry_count_threshold', 5).value)
        self.exit_count_threshold = int(
            self.declare_parameter('exit_count_threshold', 10).value)

        # 내부 상태
        self.latest_scan: LaserScan = None
        self.is_tunnel = False
        self._tunnel_enter_cnt = 0
        self._tunnel_exit_cnt = 0

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
        self.pub_steering = self.create_publisher(
            Float64, '/tunnel/steering', qos_rel)
        self.pub_speed = self.create_publisher(
            Float64, '/tunnel/speed', qos_rel)
        self.pub_active = self.create_publisher(
            Bool, '/tunnel/active', qos_rel)

        # ═══════ 서브스크라이버 ═══════
        self.create_subscription(
            LaserScan, self.sub_lidar_topic,
            self._scan_cb, qos_best)

        # ═══════ 타이머 ═══════
        self.timer = self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info('TunnelNavNode 시작')

    # ────────── LiDAR 콜백 ──────────
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ────────── 각도 범위 내 거리 추출 ──────────
    def _get_ranges_in_angle(self, msg: LaserScan,
                              angle_min_deg: float,
                              angle_max_deg: float) -> np.ndarray:
        """지정 각도 범위 내의 유효 거리값 배열 반환"""
        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return np.array([])

        ang_min = math.radians(angle_min_deg)
        ang_max = math.radians(angle_max_deg)

        i0 = int((ang_min - msg.angle_min) / msg.angle_increment)
        i1 = int((ang_max - msg.angle_min) / msg.angle_increment)
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i0 > i1:
            i0, i1 = i1, i0

        sector = ranges[i0:i1 + 1]
        valid = np.isfinite(sector) & (sector > msg.range_min) & (sector < msg.range_max)
        return sector[valid]

    # ────────── 터널 감지 ──────────
    def _detect_tunnel(self, msg: LaserScan) -> bool:
        """좌/우 벽이 동시에 가까이 있으면 터널로 판단"""
        left = self._get_ranges_in_angle(
            msg, self.left_angle_min_deg, self.left_angle_max_deg)
        right = self._get_ranges_in_angle(
            msg, self.right_angle_min_deg, self.right_angle_max_deg)

        left_wall = (len(left) >= self.tunnel_min_points
                     and np.mean(left) < self.tunnel_wall_dist_m)
        right_wall = (len(right) >= self.tunnel_min_points
                      and np.mean(right) < self.tunnel_wall_dist_m)
        return left_wall and right_wall

    # ────────── 제어 루프 ──────────
    def _control_loop(self):
        if self.latest_scan is None:
            self._pub_values(0.0, 0.0, False)
            return

        msg = self.latest_scan
        detected = self._detect_tunnel(msg)

        # 히스테리시스: 연속 N회 이상 감지/미감지 시 상태 전환
        if detected:
            self._tunnel_enter_cnt += 1
            self._tunnel_exit_cnt = 0
            if self._tunnel_enter_cnt >= self.entry_count_threshold:
                self.is_tunnel = True
        else:
            self._tunnel_exit_cnt += 1
            self._tunnel_enter_cnt = 0
            if self._tunnel_exit_cnt >= self.exit_count_threshold:
                self.is_tunnel = False

        if not self.is_tunnel:
            self._pub_values(0.0, 0.0, False)
            return

        # ── 터널 내부 조향: 좌/우 벽 거리 차이 → 중앙 유지 ──
        left = self._get_ranges_in_angle(
            msg, self.left_angle_min_deg, self.left_angle_max_deg)
        right = self._get_ranges_in_angle(
            msg, self.right_angle_min_deg, self.right_angle_max_deg)

        left_dist = float(np.mean(left)) if len(left) > 0 else self.tunnel_wall_dist_m
        right_dist = float(np.mean(right)) if len(right) > 0 else self.tunnel_wall_dist_m

        # 양수 = 오른쪽이 더 가까움 → 왼쪽으로 조향
        error = right_dist - left_dist
        steering = self.k_tunnel * error
        steering = max(-self.max_steering, min(self.max_steering, steering))

        self._pub_values(steering, self.tunnel_speed, True)
        self.get_logger().info(
            f'[Tunnel] L={left_dist:.2f}m R={right_dist:.2f}m '
            f'err={error:.2f} steer={steering:.2f}',
            throttle_duration_sec=0.5)

    # ────────── 퍼블리시 헬퍼 ──────────
    def _pub_values(self, steering: float, speed: float, active: bool):
        s = Float64(); s.data = float(steering)
        self.pub_steering.publish(s)
        v = Float64(); v.data = float(speed)
        self.pub_speed.publish(v)
        b = Bool(); b.data = active
        self.pub_active.publish(b)


def main(args=None):
    rclpy.init(args=args)
    node = TunnelNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
