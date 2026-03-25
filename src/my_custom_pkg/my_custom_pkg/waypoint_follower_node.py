#!/usr/bin/env python3
"""
waypoint_follower_node
======================
GPS 기반 웨이포인트 추종 노드.

Subscribed Topics:
  /ublox_gps_node/fix    (sensor_msgs/NavSatFix)  — GPS 위치
  /ublox_gps_node/navpvt (ublox_msgs/NavPVT)      — GPS 헤딩 (선택)

Published Topics:
  /waypoint/steering  (std_msgs/Float64)  — 웨이포인트 기반 조향 명령 (-7 ~ +7)
  /waypoint/speed     (std_msgs/Float64)  — 웨이포인트 기반 속도 명령
  /waypoint/idx       (std_msgs/Int32)    — 현재 추종 중인 웨이포인트 인덱스
  /waypoint/status    (std_msgs/String)   — 상태 (TRACKING / PAUSED / FINISHED / WAITING_GPS)

mission_controller_node가 이 토픽을 구독하여 최종 cmd를 결정합니다.
"""
import math
import csv
import ast
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSDurabilityPolicy, QoSReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32, Float64

# pygeodesy (UTM)
try:
    from pygeodesy.utm import toUtm8, Utm
    _HAS_PYGEODESY = True
except ImportError:
    _HAS_PYGEODESY = False

# ublox NavPVT (GPS heading)
try:
    from ublox_msgs.msg import NavPVT
    _HAS_NAVPVT = True
except Exception:
    _HAS_NAVPVT = False


def wrap_pi(a: float) -> float:
    """각도를 [-π, π) 범위로 정규화"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # ═══════ 파라미터 ═══════
        self.sub_fix_topic = self.declare_parameter(
            'sub_fix_topic', '/ublox_gps_node/fix').value
        self.sub_navpvt_topic = self.declare_parameter(
            'sub_navpvt_topic', '/ublox_gps_node/navpvt').value
        self.timer_period = float(
            self.declare_parameter('timer_period', 0.1).value)
        self.arrive_dist_m = float(
            self.declare_parameter('arrive_dist_m', 2.5).value)

        # 속도
        self.speed_forward = float(
            self.declare_parameter('speed_forward', 200.0).value)
        self.speed_slow = float(
            self.declare_parameter('speed_slow', 120.0).value)
        self.slow_dist_m = float(
            self.declare_parameter('slow_dist_m', 5.0).value)

        # 스티어링 제한
        self.max_steering = int(
            self.declare_parameter('max_steering', 7).value)

        # GPS→차량 기준점 오프셋
        self.ref_fwd = float(
            self.declare_parameter('ref_from_gps_forward_m', 0.5).value)
        self.ref_lft = float(
            self.declare_parameter('ref_from_gps_left_m', 0.0).value)
        self.min_speed_ms = float(
            self.declare_parameter('min_speed_ms', 0.5).value)

        # PID
        self.k_p = float(self.declare_parameter('k_p', -0.4).value)
        self.k_i = float(self.declare_parameter('k_i', -0.01).value)
        self.k_d = float(self.declare_parameter('k_d', -0.12).value)
        self.integral_max = float(
            self.declare_parameter('integral_max', 1.0).value)
        self.integral_min = float(
            self.declare_parameter('integral_min', -1.0).value)
        self.max_angular = float(
            self.declare_parameter('max_angular', 0.5).value)

        # 저역통과 / 레이트 리미터
        self.steering_alpha = float(
            self.declare_parameter('steering_alpha', 0.55).value)
        self.steering_step_hyst = int(
            self.declare_parameter('steering_step_hyst', 2).value)
        self.steering_rate_limit = int(
            self.declare_parameter('steering_rate_limit', 2).value)

        # 정지/일시정지/후진 구간
        self.stop_wp_indices: List[int] = self._parse_index_list(
            self.declare_parameter('stop_wp_indices', '[]').value)
        self.pause_wp_indices: List[int] = self._parse_index_list(
            self.declare_parameter('pause_wp_indices', '[]').value)
        self.pause_seconds = float(
            self.declare_parameter('pause_seconds', 0.0).value)
        self.reverse_ranges: List[Tuple[int, int]] = self._parse_ranges(
            self.declare_parameter('reverse_ranges', '').value)

        # CSV
        self.csv_path = self.declare_parameter(
            'waypoint_csv', 'waypoints.csv').value

        # ═══════ 내부 상태 ═══════
        self.integral_error = 0.0
        self.previous_error = 0.0
        self._prev_steering_f = 0.0
        self._prev_steering_i = 0

        self.origin_e0n0: Optional[Tuple[float, float]] = None
        self.curr_xy: Optional[Tuple[float, float]] = None
        self.prev_xy: Optional[Tuple[float, float]] = None
        self.curr_yaw: Optional[float] = None
        self.heading_ok = False
        self.last_fix_time = None
        self.goal_xy_local: Optional[Tuple[float, float]] = None

        self.waypoints = self._load_csv(self.csv_path)
        self.current_wp_idx = 0
        self.did_autostart = False
        self.paused_until = None
        self.stopped_forever = False
        self.is_entering_reverse = False

        # ═══════ QoS ═══════
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_best = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        # ═══════ 퍼블리셔 ═══════
        self.pub_steering = self.create_publisher(
            Float64, '/waypoint/steering', qos_rel)
        self.pub_speed = self.create_publisher(
            Float64, '/waypoint/speed', qos_rel)
        self.pub_idx = self.create_publisher(
            Int32, '/waypoint/idx', qos_rel)
        self.pub_status = self.create_publisher(
            String, '/waypoint/status', qos_rel)

        # ═══════ 서브스크라이버 ═══════
        self.create_subscription(
            NavSatFix, self.sub_fix_topic, self._on_fix, qos_rel)
        if _HAS_NAVPVT:
            self.create_subscription(
                NavPVT, self.sub_navpvt_topic, self._on_navpvt, qos_best)
        else:
            self.get_logger().warn(
                'ublox_msgs/NavPVT 미발견 — 위치변화량 yaw 추정만 사용')

        # ═══════ 타이머 ═══════
        self.timer = self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info(
            f'WaypointFollower 시작 ({len(self.waypoints)}개 WP)')

    # ────────────── CSV / 파싱 ──────────────
    def _load_csv(self, path: str) -> List[Tuple[float, float]]:
        wps = []
        try:
            with open(path, newline='') as f:
                for row in csv.reader(f):
                    if len(row) >= 2:
                        try:
                            wps.append((float(row[0]), float(row[1])))
                        except ValueError:
                            continue
            self.get_logger().info(f'WP {len(wps)}개 로드: {path}')
        except Exception as e:
            self.get_logger().error(f'CSV 로드 실패: {e}')
        return wps

    @staticmethod
    def _parse_index_list(s) -> List[int]:
        if isinstance(s, list):
            return [int(x) for x in s]
        try:
            v = ast.literal_eval(s) if isinstance(s, str) else s
            if isinstance(v, list):
                return [int(x) for x in v]
        except Exception:
            pass
        try:
            if isinstance(s, str) and s.strip():
                return [int(x.strip()) for x in s.split(',')]
        except Exception:
            pass
        return []

    @staticmethod
    def _parse_ranges(s) -> List[Tuple[int, int]]:
        ranges = []
        if not s:
            return ranges
        try:
            v = ast.literal_eval(s)
            if isinstance(v, list):
                for item in v:
                    if isinstance(item, (list, tuple)) and len(item) == 2:
                        a, b = int(item[0]), int(item[1])
                        ranges.append((min(a, b), max(a, b)))
                return ranges
        except Exception:
            pass
        try:
            for p in [p.strip() for p in s.split(',')]:
                if '-' in p:
                    a, b = p.split('-', 1)
                    a, b = int(a.strip()), int(b.strip())
                    ranges.append((min(a, b), max(a, b)))
        except Exception:
            pass
        return ranges

    def _in_range(self, idx, ranges):
        return any(a <= idx <= b for a, b in ranges)

    # ────────────── UTM 변환 ──────────────
    def _ll2local(self, lat, lon):
        if self.origin_e0n0 is None:
            return None
        g = toUtm8(latlon=lat, lon=lon)
        e0, n0 = self.origin_e0n0
        return (g.easting - e0, g.northing - n0)

    # ────────────── GPS 콜백 ──────────────
    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        if not _HAS_PYGEODESY:
            self.get_logger().error('pygeodesy 미설치!', once=True)
            return

        u = toUtm8(latlon=msg.latitude, lon=msg.longitude)
        e, n = u.easting, u.northing
        if self.origin_e0n0 is None:
            self.origin_e0n0 = (e, n)

        e0, n0 = self.origin_e0n0
        self.prev_xy = self.curr_xy
        self.curr_xy = (e - e0, n - n0)
        self.last_fix_time = self.get_clock().now()

        if not self.did_autostart:
            self._autostart_nearest()
            self.did_autostart = True
        else:
            self._update_goal()

        if (not self.heading_ok) and self.prev_xy is not None:
            dx = self.curr_xy[0] - self.prev_xy[0]
            dy = self.curr_xy[1] - self.prev_xy[1]
            if abs(dx) + abs(dy) > 0.02:
                self.curr_yaw = math.atan2(dy, dx)

    def _on_navpvt(self, m):
        try:
            heading_deg = (m.heading * 1e-5) if hasattr(m, 'heading') else None
            g_speed = (m.g_speed / 1000.0) if hasattr(m, 'g_speed') else 0.0
        except Exception:
            return
        self.heading_ok = heading_deg is not None and g_speed >= self.min_speed_ms
        if self.heading_ok:
            yaw_deg = 90.0 - heading_deg
            while yaw_deg > 180.0:
                yaw_deg -= 360.0
            while yaw_deg < -180.0:
                yaw_deg += 360.0
            self.curr_yaw = math.radians(yaw_deg)

    # ────────────── 웨이포인트 관리 ──────────────
    def _autostart_nearest(self):
        if not self.waypoints or self.curr_xy is None:
            return
        cx, cy = self.curr_xy
        best_i, best_d = 0, float('inf')
        for i, (lat, lon) in enumerate(self.waypoints):
            xy = self._ll2local(lat, lon)
            if xy is None:
                continue
            d = math.hypot(xy[0] - cx, xy[1] - cy)
            if d < best_d:
                best_d, best_i = d, i
        self.current_wp_idx = best_i
        self._update_goal()
        self.get_logger().info(
            f'[AutoStart] WP{best_i} (거리 {best_d:.1f}m)')

    def _update_goal(self):
        if self.current_wp_idx < len(self.waypoints):
            xy = self._ll2local(*self.waypoints[self.current_wp_idx])
            if xy is not None:
                self.goal_xy_local = xy

    def _advance(self) -> bool:
        self.current_wp_idx += 1
        self.integral_error = 0.0
        self.previous_error = 0.0
        if self.current_wp_idx >= len(self.waypoints):
            self.stopped_forever = True
            return False
        self._update_goal()
        return True

    # ────────────── 퍼블리시 헬퍼 ──────────────
    def _pub(self, steering: float, speed: float, status: str):
        s = Float64(); s.data = float(steering)
        self.pub_steering.publish(s)
        v = Float64(); v.data = float(speed)
        self.pub_speed.publish(v)
        idx = Int32(); idx.data = self.current_wp_idx
        self.pub_idx.publish(idx)
        st = String(); st.data = status
        self.pub_status.publish(st)

    # ────────────── 메인 제어 루프 ──────────────
    def _control_loop(self):
        # 종료
        if self.stopped_forever:
            self._pub(0.0, 0.0, 'FINISHED')
            return

        # 일시정지
        if self.paused_until is not None:
            if self.get_clock().now() < self.paused_until:
                self._pub(0.0, 10.0, 'PAUSED')
                return
            self.paused_until = None

        # GPS 대기
        if self.goal_xy_local is None or self.curr_xy is None:
            self._pub(0.0, 0.0, 'WAITING_GPS')
            return

        # GPS timeout
        if (self.last_fix_time is not None
                and (self.get_clock().now() - self.last_fix_time)
                > Duration(seconds=2.0)):
            self.get_logger().warn('GPS timeout', throttle_duration_sec=2)
            self._pub(0.0, 0.0, 'GPS_TIMEOUT')
            return

        # 기준점 보정
        cx, cy = self.curr_xy
        if self.curr_yaw is not None:
            ux, uy = math.cos(self.curr_yaw), math.sin(self.curr_yaw)
            cx -= self.ref_fwd * ux + self.ref_lft * (-uy)
            cy -= self.ref_fwd * uy + self.ref_lft * ux

        gx, gy = self.goal_xy_local
        dx, dy = gx - cx, gy - cy
        dist = math.hypot(dx, dy)

        # ── 도착 판정 ──
        if dist < self.arrive_dist_m:
            self.get_logger().info(f'WP{self.current_wp_idx} 도착')
            if self.current_wp_idx in self.stop_wp_indices:
                self.stopped_forever = True
                self._pub(0.0, 0.0, 'FINISHED')
                return
            if (self.current_wp_idx in self.pause_wp_indices
                    and self.pause_seconds > 0):
                self.paused_until = (self.get_clock().now()
                                     + Duration(seconds=self.pause_seconds))
                self._advance()
                self._pub(0.0, 0.0, 'PAUSED')
                return
            if not self._advance():
                self._pub(0.0, 0.0, 'FINISHED')
                return
            return

        # ── 방위각 ──
        desired_yaw = math.atan2(dy, dx)
        reverse = self._in_range(self.current_wp_idx, self.reverse_ranges)

        if reverse and not self.is_entering_reverse:
            self.paused_until = (self.get_clock().now()
                                 + Duration(seconds=1.0))
            self.is_entering_reverse = True
            self._pub(0.0, 0.0, 'PAUSED')
            return
        if not reverse:
            self.is_entering_reverse = False
        if reverse:
            desired_yaw = wrap_pi(desired_yaw + math.pi)

        # ── PID 조향 ──
        if self.curr_yaw is None:
            steering = 0.0
        else:
            dt = self.timer_period
            err = wrap_pi(desired_yaw - self.curr_yaw)

            p = self.k_p * err
            self.integral_error += err * dt
            self.integral_error = max(self.integral_min,
                                      min(self.integral_max,
                                          self.integral_error))
            i = self.k_i * self.integral_error
            d = self.k_d * ((err - self.previous_error) / dt)
            self.previous_error = err

            w = max(-self.max_angular,
                    min(self.max_angular, p + i + d))
            sf = (w / self.max_angular) * self.max_steering
            sf = max(-self.max_steering, min(self.max_steering, sf))

            # 저역통과
            sl = ((1.0 - self.steering_alpha) * self._prev_steering_f
                  + self.steering_alpha * sf)
            self._prev_steering_f = sl
            si = int(round(sl))
            si = max(-self.max_steering, min(self.max_steering, si))

            # 히스테리시스
            if abs(si - self._prev_steering_i) < self.steering_step_hyst:
                si = self._prev_steering_i
            # 레이트 리미트
            delta = si - self._prev_steering_i
            if delta > self.steering_rate_limit:
                si = self._prev_steering_i + self.steering_rate_limit
            elif delta < -self.steering_rate_limit:
                si = self._prev_steering_i - self.steering_rate_limit
            self._prev_steering_i = si
            steering = float(si)

        # ── 속도 ──
        if reverse:
            speed = -80.0
        elif dist < self.slow_dist_m:
            speed = self.speed_slow
        else:
            speed = self.speed_forward

        self._pub(steering, speed, 'TRACKING')
        self.get_logger().info(
            f'[WP{self.current_wp_idx}] d={dist:.1f}m '
            f'steer={steering:.0f} spd={speed:.0f} '
            f'{"REV" if reverse else "FWD"}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub(0.0, 0.0, 'FINISHED')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
