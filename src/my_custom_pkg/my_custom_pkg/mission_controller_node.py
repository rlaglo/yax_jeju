#!/usr/bin/env python3
"""
mission_controller_node
=======================
모든 하위 노드(obstacle, lane, tunnel, waypoint)의 토픽을 구독하고
**우선순위**에 따라 최종 cmd 명령을 결정하여 시리얼 노드로 전송합니다.

우선순위 (높은 순):
  1. 장애물 감지 (obstacle)  → 즉시 정지
  2. 차선 추종 (lane)        → 카메라 차선 기반 조향
  3. 터널 주행 (tunnel)      → LiDAR 벽면 기반 중앙유지
  4. GPS 웨이포인트 (waypoint) → GPS/UTM 기반 조향 (기본)

Subscribed Topics:
  /obstacle/detected  (Bool)     — 장애물 감지 여부
  /obstacle/steering  (Float64)  — (예비) 회피 조향
  /obstacle/speed     (Float64)  — 0=정지, -1=제어없음

  /lane/active        (Bool)     — 차선 감지 활성 여부
  /lane/steering      (Float64)  — 차선 조향
  /lane/speed         (Float64)  — 차선 속도

  /tunnel/active      (Bool)     — 터널 내부 여부
  /tunnel/steering    (Float64)  — 터널 조향
  /tunnel/speed       (Float64)  — 터널 속도

  /waypoint/steering  (Float64)  — GPS 조향
  /waypoint/speed     (Float64)  — GPS 속도
  /waypoint/status    (String)   — GPS 상태
  /waypoint/idx       (Int32)    — 현재 WP 인덱스

Published Topics:
  topic_control_signal (interfaces_pkg/MotionCommand) — 최종 명령
    → serial_sender_node가 이를 받아 아두이노로 전송
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSDurabilityPolicy, QoSReliabilityPolicy,
)
from std_msgs.msg import String, Bool, Int32, Float64
from interfaces_pkg.msg import MotionCommand


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        # ═══════ 파라미터 ═══════
        self.pub_topic = self.declare_parameter(
            'pub_topic', 'topic_control_signal').value
        self.timer_period = float(
            self.declare_parameter('timer_period', 0.1).value)
        self.max_steering = int(
            self.declare_parameter('max_steering', 7).value)
        self.max_abs_pwm = int(
            self.declare_parameter('max_abs_pwm', 255).value)

        # ═══════ 각 소스의 최신 데이터 ═══════
        # Obstacle
        self.obstacle_detected = False
        self.obstacle_steering = 0.0
        self.obstacle_speed = -1.0  # -1 = 제어없음

        # Lane
        self.lane_active = False
        self.lane_steering = 0.0
        self.lane_speed = 0.0

        # Tunnel
        self.tunnel_active = False
        self.tunnel_steering = 0.0
        self.tunnel_speed = 0.0

        # Waypoint (GPS)
        self.wp_steering = 0.0
        self.wp_speed = 0.0
        self.wp_status = 'WAITING_GPS'
        self.wp_idx = 0

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
        self.pub_cmd = self.create_publisher(
            MotionCommand, self.pub_topic, qos_rel)
        # 디버그: 어떤 소스가 선택되었는지 표시
        self.pub_active_source = self.create_publisher(
            String, '/mission/active_source', qos_rel)

        # ═══════ 서브스크라이버 — Obstacle ═══════
        self.create_subscription(
            Bool, '/obstacle/detected',
            self._obstacle_detected_cb, qos_rel)
        self.create_subscription(
            Float64, '/obstacle/steering',
            self._obstacle_steering_cb, qos_rel)
        self.create_subscription(
            Float64, '/obstacle/speed',
            self._obstacle_speed_cb, qos_rel)

        # ═══════ 서브스크라이버 — Lane ═══════
        self.create_subscription(
            Bool, '/lane/active',
            self._lane_active_cb, qos_rel)
        self.create_subscription(
            Float64, '/lane/steering',
            self._lane_steering_cb, qos_rel)
        self.create_subscription(
            Float64, '/lane/speed',
            self._lane_speed_cb, qos_rel)

        # ═══════ 서브스크라이버 — Tunnel ═══════
        self.create_subscription(
            Bool, '/tunnel/active',
            self._tunnel_active_cb, qos_rel)
        self.create_subscription(
            Float64, '/tunnel/steering',
            self._tunnel_steering_cb, qos_rel)
        self.create_subscription(
            Float64, '/tunnel/speed',
            self._tunnel_speed_cb, qos_rel)

        # ═══════ 서브스크라이버 — Waypoint ═══════
        self.create_subscription(
            Float64, '/waypoint/steering',
            self._wp_steering_cb, qos_rel)
        self.create_subscription(
            Float64, '/waypoint/speed',
            self._wp_speed_cb, qos_rel)
        self.create_subscription(
            String, '/waypoint/status',
            self._wp_status_cb, qos_rel)
        self.create_subscription(
            Int32, '/waypoint/idx',
            self._wp_idx_cb, qos_rel)

        # ═══════ 타이머 ═══════
        self.timer = self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info(
            f'MissionController 시작 → publish: {self.pub_topic}')

    # ────────── Obstacle 콜백 ──────────
    def _obstacle_detected_cb(self, msg: Bool):
        self.obstacle_detected = msg.data

    def _obstacle_steering_cb(self, msg: Float64):
        self.obstacle_steering = msg.data

    def _obstacle_speed_cb(self, msg: Float64):
        self.obstacle_speed = msg.data

    # ────────── Lane 콜백 ──────────
    def _lane_active_cb(self, msg: Bool):
        self.lane_active = msg.data

    def _lane_steering_cb(self, msg: Float64):
        self.lane_steering = msg.data

    def _lane_speed_cb(self, msg: Float64):
        self.lane_speed = msg.data

    # ────────── Tunnel 콜백 ──────────
    def _tunnel_active_cb(self, msg: Bool):
        self.tunnel_active = msg.data

    def _tunnel_steering_cb(self, msg: Float64):
        self.tunnel_steering = msg.data

    def _tunnel_speed_cb(self, msg: Float64):
        self.tunnel_speed = msg.data

    # ────────── Waypoint 콜백 ──────────
    def _wp_steering_cb(self, msg: Float64):
        self.wp_steering = msg.data

    def _wp_speed_cb(self, msg: Float64):
        self.wp_speed = msg.data

    def _wp_status_cb(self, msg: String):
        self.wp_status = msg.data

    def _wp_idx_cb(self, msg: Int32):
        self.wp_idx = msg.data

    # ═══════════════════════════════════
    #         메인 제어 루프
    #  우선순위: obstacle > lane > tunnel > waypoint
    # ═══════════════════════════════════
    def _control_loop(self):
        steering = 0.0
        speed = 0.0
        source = 'NONE'

        # ── 우선순위 1: 장애물 감지 → 즉시 정지 ──
        if self.obstacle_detected:
            steering = self.obstacle_steering  # 현재 0 (향후 회피 조향 확장 가능)
            speed = 0.0                         # 무조건 정지
            source = 'OBSTACLE'

        # ── 우선순위 2: 차선 감지 → 차선 추종 ──
        elif self.lane_active:
            steering = self.lane_steering
            speed = self.lane_speed
            source = 'LANE'

        # ── 우선순위 3: 터널 감지 → 터널 벽 기반 중앙유지 ──
        elif self.tunnel_active:
            steering = self.tunnel_steering
            speed = self.tunnel_speed
            source = 'TUNNEL'

        # ── 우선순위 4: GPS 웨이포인트 → 기본 주행 ──
        else:
            steering = self.wp_steering
            speed = self.wp_speed
            source = f'GPS(WP{self.wp_idx})'

            # 웨이포인트 상태에 따른 예외 처리
            if self.wp_status in ('FINISHED', 'GPS_TIMEOUT'):
                steering = 0.0
                speed = 0.0
                source = f'GPS_{self.wp_status}'

        # ── 최종 값 클램핑 ──
        final_steering = int(max(-self.max_steering,
                                 min(self.max_steering, steering)))
        final_speed = int(max(-self.max_abs_pwm,
                              min(self.max_abs_pwm, speed)))

        # ── MotionCommand 퍼블리시 ──
        cmd = MotionCommand()
        cmd.steering = final_steering
        cmd.left_speed = final_speed
        cmd.right_speed = final_speed
        self.pub_cmd.publish(cmd)

        # ── 활성 소스 퍼블리시 ──
        src_msg = String()
        src_msg.data = source
        self.pub_active_source.publish(src_msg)

        self.get_logger().info(
            f'[{source}] steer={final_steering} '
            f'L={final_speed} R={final_speed}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 정지
        cmd = MotionCommand()
        cmd.steering = 0
        cmd.left_speed = 0
        cmd.right_speed = 0
        node.pub_cmd.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
