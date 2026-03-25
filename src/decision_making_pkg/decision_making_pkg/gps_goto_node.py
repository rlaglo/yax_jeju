#!/usr/bin/env python3
import math
from typing import Optional, Tuple, List, Tuple as Tup
import csv
import ast
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Empty
from interfaces_pkg.msg import MotionCommand

# u-blox NAV-PVT 메시지 사용
try:
    from sensor_msgs.msg import NavPVT
    _HAS_NAVPVT = True
except Exception:
    _HAS_NAVPVT = False

# (권장) pygeodesy 설치되어 있다면 UTM 사용
try:
    from pygeodesy.utm import toUtm8, Utm
    _HAS_PYGEODESY = True
except Exception:
    _HAS_PYGEODESY = False


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GPSGotoNode(Node):
    """
    NAV-PVT의 heading으로 조향을 제어하고,
    좌/우 바퀴 PWM을 출력.
    - 특정 웨이포인트에서 정지
    - 특정 웨이포인트 구간은 후진만
    목표점에 도착하면 다음 웨이포인트로 이동.
    """

    def __init__(self):
        super().__init__('gps_goto_node')

        # ---------- Parameters ----------
        self.pub_topic     = self.declare_parameter('pub_topic', 'topic_control_signal').value
        self.sub_fix_topic = self.declare_parameter('sub_fix_topic', '/ublox_gps_node/fix').value
        self.sub_navpvt_topic = self.declare_parameter('sub_navpvt_topic', '/ublox_gps_node/navpvt').value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', 'yolov8_traffic_light_info').value
        self.sub_lidar_topic = self.declare_parameter('sub_lidar_topic', 'scan').value
        self.min_speed_ms     = float(self.declare_parameter('min_speed_ms', 0.5).value)
        self.timer_period   = float(self.declare_parameter('timer', 0.1).value)     # s
        self.arrive_dist_m  = float(self.declare_parameter('arrive_dist_m', 3.0).value)
        self.max_angular    = float(self.declare_parameter('max_angular', 0.5).value)
        self.k_w            = float(self.declare_parameter('k_w', -0.4).value)      # w 게인 (조향 방향 수정)
        self.ref_from_gps_forward_m = float(self.declare_parameter('ref_from_gps_forward_m', 0.5).value)  # GPS가 중심보다 0.6 m 앞에 있다면 +0.6
        self.ref_from_gps_left_m    = float(self.declare_parameter('ref_from_gps_left_m',    0.0).value)  # 중심보다 좌로 치우친 경우(+), 보통 0
        self.k_p = float(self.declare_parameter('k_p', -0.4).value) # P 게인 (기존 k_w)
        self.k_i = float(self.declare_parameter('k_i', -0.01).value) # I 게인 (튜닝 필요)
        self.k_d = float(self.declare_parameter('k_d', -0.12).value) # D 게인 (튜닝 필요)

        # I항의 과도한 누적을 방지 (Integral Windup 방지)
        self.integral_max = float(self.declare_parameter('integral_max', 1.0).value)
        self.integral_min = float(self.declare_parameter('integral_min', -1.0).value)

        # PID 상태 변수
        self.integral_error = 0.0
        self.previous_error = 0.0
        # 목표점 (단일 모드도 유지)
        self.goal_lat = float(self.declare_parameter('goal_lat', 37.56163657).value)
        self.goal_lon = float(self.declare_parameter('goal_lon', 126.93716571).value)

        self.steering_alpha = float(self.declare_parameter('steering_alpha', 0.55).value)    # 0.2~0.5
        self.steering_step_hyst = int(self.declare_parameter('steering_step_hyst', 2).value) # 최소 스텝 변화
        self.steering_rate_limit = int(self.declare_parameter('steering_rate_limit', 2).value) # 틱당 최대 변화(스텝)
        self._prev_steering_f = 0.0
        self._prev_steering_i = 0

        # ✨ 속도/스티어 제한 (음수 허용)
        self.max_abs_pwm   = int(self.declare_parameter('max_abs_pwm', 255).value)
        self.max_steering  = int(self.declare_parameter('max_steering', 7).value)
        self.speed_forward_pwm = int(self.declare_parameter('speed_forward_pwm', 250).value)  # 0~+255
        self.speed_reverse_pwm = int(self.declare_parameter('speed_reverse_pwm', -200).value) # 0~-255
        self.speed_lidar_zone_pwm = int(self.declare_parameter('speed_lidar_zone_pwm', self.speed_forward_pwm).value)
        self.speed_prepare_zone_pwm = int(self.declare_parameter('speed_prepare_zone_pwm', self.speed_forward_pwm).value)
        self.traffic_light_left_confidence_count = int(self.declare_parameter('traffic_light_left_confidence_count', 10).value)
        self.max_pwm_change = int(self.declare_parameter('max_pwm_change', 15).value) # 타이머 주기(0.1초)당 최대 15씩 변경 (튜닝 필요!)
        self._current_pwm = 0
        self.initial_drive_pwm = int(self.declare_parameter('initial_drive_pwm', 100).value)
        self.initial_drive_duration_sec = float(self.declare_parameter('initial_drive_duration_sec', 0.5).value) # 초 단위
        
        # 저속 출발 상태를 관리하는 변수들
        self._is_in_initial_drive = False
        self._initial_drive_end_time = None
        # ✨ 기본 주행 속도 (기존 const_pwm 대체; 하위호환 위해 있으면 우선 적용)
        if self.has_parameter('const_pwm'):
            cp = int(self.get_parameter('const_pwm').value)
            # 음수 허용 + 절대값 clamp
            cp = max(-self.max_abs_pwm, min(self.max_abs_pwm, cp))
            self.speed_forward_pwm = cp if cp >= 0 else self.speed_forward_pwm
            self.speed_reverse_pwm = cp if cp < 0 else self.speed_reverse_pwm

        # CSV 경로
        self.csv_path = self.declare_parameter('waypoint_csv', 'waypoints.csv').value
        self.waypoints = self.load_waypoints_from_csv(self.csv_path)
        self.current_wp_idx = 0
        self.original_waypoints = list(self.waypoints)
        self.original_wp_idx = 0
        self.mission_reset_sent = {'t_parking': False, 'p_parking': False} 

        # ✨ 정지/일시정지/후진 구간 파라미터
        self.stop_wp_indices: List[int] = self._parse_index_list(
            self.declare_parameter('stop_wp_indices', '[]').value
        )
        self.pause_wp_indices: List[int] = self._parse_index_list(
            self.declare_parameter('pause_wp_indices', '[]').value
        )
        self.pause_seconds: float = float(self.declare_parameter('pause_seconds', 0.0).value)

        # 예: "3-6,10-12" 또는 "[(3,6),(10,12)]"
        self.reverse_ranges: List[Tup[int, int]] = self._parse_ranges(
            self.declare_parameter('reverse_ranges', '').value
        )
        self.lidar_activation_ranges: List[Tup[int, int]] = self._parse_ranges(
            self.declare_parameter('lidar_activation_ranges', '').value
        )
        self.traffic_light_green_go_ranges: List[Tup[int, int]] = self._parse_ranges(
            self.declare_parameter('traffic_light_green_go_ranges', '').value
        )
        self.traffic_light_left_go_ranges: List[Tup[int, int]] = self._parse_ranges(
            self.declare_parameter('traffic_light_left_go_ranges', '').value
        )

        self.t_parking_prepare_ranges = self._parse_ranges(self.declare_parameter('t_parking_prepare_ranges', '').value)
        self.parallel_parking_prepare_ranges = self._parse_ranges(self.declare_parameter('parallel_parking_prepare_ranges', '').value)

        # ---------- T자 주차 미션 파라미터 선언 ----------
        self.t_parking_mission_ranges = self._parse_ranges(self.declare_parameter('t_parking_mission_ranges', '').value)
        self.t_parking_right_csv = self.declare_parameter('t_parking_right_csv', '').value
        self.t_parking_left_csv = self.declare_parameter('t_parking_left_csv', '').value

        # ---------- 평행 주차 미션 파라미터 선언 ----------
        self.parallel_parking_mission_ranges = self._parse_ranges(self.declare_parameter('parallel_parking_mission_ranges', '').value)
        self.p_parking_left_csv = self.declare_parameter('p_parking_left_csv', '').value
        self.p_parking_right_csv = self.declare_parameter('p_parking_right_csv', '').value

        # ---------- 객체 탐지 토픽 파라미터 선언 ----------
        self.sub_object_left_topic = self.declare_parameter('sub_object_left_topic', '/object/left/detection_order').value
        self.sub_object_right_topic = self.declare_parameter('sub_object_right_topic', '/object/right/detection_order').value
        # 상태
        self.paused_until = None
        self.stopped_forever = False
        self.did_autostart = False
        self.traffic_light_state = "None"
        self.left_signal_counter = 0
        self.is_crossing_intersection = False
        self.latest_scan = None # 최신 라이다 데이터를 저장할 변수
        self.is_entering_reverse = False
        self.left_detection_order = []
        self.right_detection_order = []
        self.is_in_replay_mode = False
        self.current_mission_name = "None"
        self.replay_data = []  # List to store (steering, left_speed, right_speed) tuples
        self.replay_idx = 0
        self.replay_timer_period = 0.02 # driving_log.csv 재생 주기 (초)

        qos_rel_1 = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_best_effort_1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        qos_best_10 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        self.pub_mc  = self.create_publisher(MotionCommand, self.pub_topic, qos_rel_1)
        self.pub_reset_left = self.create_publisher(Empty, '/object/left/reset', 10)
        self.pub_reset_right = self.create_publisher(Empty, '/object/right/reset', 10)
        self.sub_fix = self.create_subscription(NavSatFix, self.sub_fix_topic, self.on_fix, qos_rel_1)
        self.sub_lidar = self.create_subscription(
            LaserScan, self.sub_lidar_topic, self.on_lidar, qos_best_effort_1)
        self.sub_traffic_light = self.create_subscription(
            String, self.sub_traffic_light_topic, self.on_traffic_light, qos_best_effort_1)
        
        self.sub_left_objects = self.create_subscription(String, self.sub_object_left_topic, self.on_left_detection, 10)
        self.sub_right_objects = self.create_subscription(String, self.sub_object_right_topic, self.on_right_detection, 10)

        if _HAS_NAVPVT:
            self.sub_navpvt = self.create_subscription(NavPVT, self.sub_navpvt_topic, self.on_navpvt, qos_best_10)
        else:
            self.sub_navpvt = None
            self.get_logger().warn("ublox_msgs/NavPVT 를 찾지 못했습니다.")

        self.curr_xy: Optional[Tuple[float, float]] = None
        self.prev_xy: Optional[Tuple[float, float]] = None
        self.curr_yaw: Optional[float] = None
        self.heading_ok: bool = False
        self.last_fix_time = None
        self.goal_xy_local: Optional[Tuple[float, float]] = None
        self.origin_e0n0: Optional[Tuple[float, float]] = None
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info(f"GPSGoto Node Started")

    # -------------------- Parsing helpers --------------------
    def _parse_index_list(self, s: str) -> List[int]:
        """
        문자열로 들어온 인덱스 리스트 파라미터를 안전하게 파싱.
        예: "[]", "[1,3,7]" 둘 다 허용.
        """
        try:
            v = ast.literal_eval(s) if isinstance(s, str) else s
            if isinstance(v, list):
                return [int(x) for x in v]
        except Exception:
            pass
        try:
            # "1,3,7" 형태도 허용
            if isinstance(s, str) and s.strip():
                return [int(x.strip()) for x in s.split(',')]
        except Exception:
            pass
        return []
    # 클래스 내부에 추가

    def _latlon_to_local_xy(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        """origin_e0n0가 설정된 상태에서 (lat,lon)을 로컬 (x,y)로 변환"""
        if self.origin_e0n0 is None:
            return None
        g = toUtm8(latlon=lat, lon=lon)
        ge, gn = g.eastingnorthing
        e0, n0 = self.origin_e0n0
        return (ge - e0, gn - n0)

    def _autostart_select_nearest_wp(self):
        """현재 위치(self.curr_xy) 기준으로 가장 가까운 웨이포인트 인덱스를 1회 선택"""
        if not self.waypoints or self.curr_xy is None or self.origin_e0n0 is None:
            return
        cx, cy = self.curr_xy
        best_i, best_d = None, float('inf')
        for i, (lat, lon) in enumerate(self.waypoints):
            xy = self._latlon_to_local_xy(lat, lon)
            if xy is None:
                continue
            gx, gy = xy
            d = math.hypot(gx - cx, gy - cy)
            if d < best_d:
                best_d, best_i = d, i
        if best_i is not None:
            self.current_wp_idx = best_i
            # 바로 goal_xy_local 갱신
            gxy = self._latlon_to_local_xy(*self.waypoints[self.current_wp_idx])
            if gxy is not None:
                self.goal_xy_local = gxy
            self.get_logger().info(f"[AutoStart] Nearest WP = {best_i} (dist {best_d:.1f} m)")

    def _parse_ranges(self, s: str) -> List[Tup[int, int]]:
        """
        후진 구간 문자열 파싱.
        허용형태:
        - "3-6,10-12"
        - "[(3,6),(10,12)]"
        - 빈 문자열
        """
        ranges: List[Tup[int, int]] = []
        if not s:
            return ranges
        # 시도 1: 리스트 리터럴
        try:
            v = ast.literal_eval(s)
            if isinstance(v, list):
                for item in v:
                    if (isinstance(item, (list, tuple)) and len(item) == 2):
                        a, b = int(item[0]), int(item[1])
                        if a <= b:
                            ranges.append((a, b))
                        else:
                            ranges.append((b, a))
                return ranges
        except Exception:
            pass
        # 시도 2: "3-6,10-12"
        try:
            parts = [p.strip() for p in s.split(',')]
            for p in parts:
                if '-' in p:
                    a, b = p.split('-', 1)
                    a, b = int(a.strip()), int(b.strip())
                    if a <= b:
                        ranges.append((a, b))
                    else:
                        ranges.append((b, a))
        except Exception:
            pass
        return ranges

    def _in_reverse_range(self, idx: int) -> bool:
        return any(a <= idx <= b for (a, b) in self.reverse_ranges)
    
    def _in_lidar_activation_range(self, idx: int) -> bool:
        """현재 웨이포인트 인덱스가 라이다 활성화 구간에 있는지 확인"""
        return any(start <= idx <= end for (start, end) in self.lidar_activation_ranges)
    
    def _in_traffic_light_green_go_range(self, idx: int) -> bool:
        """현재 웨이포인트가 'Green 신호 출발' 구간에 있는지 확인"""
        return any(start <= idx <= end for (start, end) in self.traffic_light_green_go_ranges)

    def _in_traffic_light_left_go_range(self, idx: int) -> bool:
        """현재 웨이포인트가 'Left 신호 출발' 구간에 있는지 확인"""
        return any(start <= idx <= end for (start, end) in self.traffic_light_left_go_ranges)
    
    # -------------------- Waypoints & Callbacks --------------------
    def load_waypoints_from_csv(self, path: str):
        waypoints = []
        try:
            with open(path, newline='') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    lat = float(row[0])
                    lon = float(row[1])
                    waypoints.append((lat, lon))
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV {path}: {e}")
        return waypoints
    def load_replay_data_from_csv(self, path: str) -> List[dict]:
        data = []
        try:
            with open(path, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for row in csv_reader:
                    data.append({'steering': int(row['steering']), 'left_speed': int(row['left_speed']), 'right_speed': int(row['right_speed'])})
            self.get_logger().info(f"Loaded {len(data)} commands from replay file: {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load replay CSV {path}: {e}")
        return data

    def start_replay_mission(self, csv_path: str, mission_name: str):
        """리플레이 미션을 시작"""
        self.get_logger().info(f"Starting '{mission_name}' mission using {csv_path}")
        self.stop_robot() # 안전을 위해 일단 정지
        
        self.replay_data = self.load_replay_data_from_csv(csv_path)
        if not self.replay_data:
            self.get_logger().error("Replay data is empty. Aborting mission.")
            return

        self.is_in_replay_mode = True
        self.current_mission_name = mission_name
        self.original_wp_idx_before_mission = self.current_wp_idx
        self.replay_idx = 0
        
        # 기존 타이머를 잠시 끄고, 리플레이 전용 고속 타이머를 설정
        self.timer.cancel()
        self.replay_timer = self.create_timer(self.replay_timer_period, self.execute_replay_step)

    def execute_replay_step(self):
        """리플레이 데이터 한 스텝을 실행"""
        if not self.is_in_replay_mode or self.replay_idx >= len(self.replay_data):
            self.finish_replay_mission()
            return

        command = self.replay_data[self.replay_idx]
        self.publish_motion_command(
            command['steering'], command['left_speed'], command['right_speed']
        )
        self.replay_idx += 1
        
        if self.replay_idx >= len(self.replay_data):
            self.finish_replay_mission()

    def finish_replay_mission(self):
        """리플레이 미션을 종료하고 원래 주행으로 복귀"""
        if not self.is_in_replay_mode: return
        
        self.get_logger().info(f"Mission '{self.current_mission_name}' complete. Returning to main route.")
        self.stop_robot()
        
        # 리플레이 타이머를 끄고 원래 제어 루프 타이머를 재시작
        if hasattr(self, 'replay_timer'):
            self.replay_timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # 상태 변수 초기화
        self.is_in_replay_mode = False
        self.replay_data = []
        self.replay_idx = 0

        # 미션 구간의 마지막 웨이포인트 다음부터 주행을 재개
        resumed = False
        if self.current_mission_name == "t_parking":
            try:
                # 현재 인덱스가 포함된 T주차 구간을 찾아 그 끝 다음으로 점프
                mission_range = next(r for r in self.t_parking_mission_ranges if r[0] <= self.original_wp_idx_before_mission <= r[1])
                self.current_wp_idx = mission_range[1] + 1
                resumed = True
            except StopIteration: pass
        elif self.current_mission_name == "p_parking":
            try:
                mission_range = next(r for r in self.parallel_parking_mission_ranges if r[0] <= self.original_wp_idx_before_mission <= r[1])
                self.current_wp_idx = mission_range[1] + 1
                resumed = True
            except StopIteration: pass

        if not resumed:
            self.get_logger().warn("Could not determine next waypoint after mission. Finding nearest.")
            self._autostart_select_nearest_wp()
        
        self.current_mission_name = "None"
        self._update_goal_xy_local()
        # 잠시 멈췄다가 출발
        self.paused_until = self.get_clock().now() + Duration(seconds=1.5)
        
    def on_fix(self, msg: NavSatFix):
        if msg.status.status < 0: return
        if not _HAS_PYGEODESY:
            self.get_logger().error("pygeodesy가 필요합니다.")
            return

        u: Utm = toUtm8(latlon=msg.latitude, lon=msg.longitude)
        e, n = u.eastingnorthing

        if self.origin_e0n0 is None:
            self.origin_e0n0 = (e, n)

        e0, n0 = self.origin_e0n0
        self.prev_xy = self.curr_xy
        self.curr_xy = (e - e0, n - n0)
        self.last_fix_time = self.get_clock().now()

        # 🔹 첫 유효 fix 때 한 번만 가장 가까운 WP로 점프
        if not self.did_autostart:
            self._autostart_select_nearest_wp()
            self.did_autostart = True
        else:
            # 기존 로직 유지: 현재 인덱스 기준 목표점 갱신
            if self.current_wp_idx < len(self.waypoints):
                g: Utm = toUtm8(latlon=self.waypoints[self.current_wp_idx][0],
                                lon=self.waypoints[self.current_wp_idx][1])
                ge, gn = g.eastingnorthing
                self.goal_xy_local = (ge - e0, gn - n0)

        if (not self.heading_ok) and (self.prev_xy is not None):
            dx = self.curr_xy[0] - self.prev_xy[0]
            dy = self.curr_xy[1] - self.prev_xy[1]
            if abs(dx) + abs(dy) > 0.02:
                self.curr_yaw = math.atan2(dy, dx)

    def _in_range_check(self, idx: int, ranges_list: List[Tup[int, int]]) -> bool:
        return any(a <= idx <= b for (a, b) in ranges_list)

    def on_navpvt(self, m: 'NavPVT'):
        try:
            heading_deg = (m.heading * 1e-5) if hasattr(m, 'heading') else None
            g_speed_ms = (m.g_speed / 1000.0) if hasattr(m, 'g_speed') else 0.0
        except Exception:
            return

        self.heading_ok = (heading_deg is not None and g_speed_ms >= self.min_speed_ms)

        if self.heading_ok:
            yaw_deg = 90.0 - heading_deg
            self.get_logger().info(f"[on_navpvt] GPS heading_deg={heading_deg:.1f} -> Converted yaw_deg={yaw_deg:.1f}")
            while yaw_deg > 180.0:
                yaw_deg -= 360.0
            while yaw_deg < -180.0:
                yaw_deg += 360.0
            self.curr_yaw = math.radians(yaw_deg)

    def on_lidar(self, msg: LaserScan):
        """LIDAR 장애물 감지 콜백"""
        #self.get_logger().info("lidar!")
        self.latest_scan = msg

    def on_traffic_light(self, msg: String):
        """
        신호등 상태를 업데이트하는 콜백.
        'Left' 신호는 일정 횟수 이상 연속으로 들어와야 확정.
        """
        received_signal = msg.data

        if received_signal == "Left":
            self.left_signal_counter += 1
            # self.get_logger().info(f"Left signal count: {self.left_signal_counter}") # 디버깅용
        else:
            # Left가 아닌 다른 신호(Red, Green, None 등)가 들어오면 카운터 리셋
            self.left_signal_counter = 0
            # Red나 Green은 즉시 상태 업데이트
            self.traffic_light_state = received_signal

        # 카운터가 설정된 기준값을 넘었을 때만, 최종 상태를 'Left'로 확정
        if self.left_signal_counter >= self.traffic_light_left_confidence_count:
            if self.traffic_light_state != "Left":
                self.get_logger().info(f"Left Signal CONFIRMED with count {self.left_signal_counter}.")
            self.traffic_light_state = "Left"
        # 아직 기준값에 도달하지 못했다면, 이전 상태를 유지 (Red가 잠깐 들어와도 무시)
        elif received_signal != "Left" and self.traffic_light_state == "Left":
            self.get_logger().info(f"Left Signal LOST. Resetting state.")
            self.traffic_light_state = received_signal # Left가 끊겼으므로 새로운 신호로 업데이트

    def _check_for_obstacle(self) -> bool:
        """
        전방(±front_half_angle_deg) 영역에서 유효 리턴의 최소거리가 stop_distance_m 미만이면 True.
        """
        if self.latest_scan is None:
            return False

        msg = self.latest_scan
        ranges = np.asarray(msg.ranges, dtype=float)

        # 1) 센서 유효범위 기반 필터 (0, NaN, inf, range_min 미만/ range_max 초과 제거)
        valid = np.isfinite(ranges)
        valid &= (ranges >= max(1e-3, msg.range_min))
        valid &= (ranges <= msg.range_max)

        if not np.any(valid):
            return False  # 유효 리턴이 하나도 없으면 장애물 없음으로 간주

        # 2) 전방 각도 인덱스 산출 (+ 클램프)
        half = math.radians(getattr(self, "front_half_angle_deg", 30.0))
        start_idx = int(( -half - msg.angle_min) / msg.angle_increment)
        end_idx   = int(( +half - msg.angle_min) / msg.angle_increment)

        n = len(ranges)
        i0 = max(0, min(start_idx, end_idx))
        i1 = min(n - 1, max(start_idx, end_idx))

        if i1 < i0:  # 방어적 (이론상 위 클램프로 안 생김)
            return False

        # 3) 전방 영역의 유효 리턴만 추출
        front = ranges[i0:i1+1]
        front_valid = valid[i0:i1+1]
        front_vals = front[front_valid]
        if front_vals.size == 0:
            return False

        # 4) 최소거리 판정
        min_d = float(front_vals.min())
        threshold = getattr(self, "stop_distance_m", 2.5)
        if min_d < threshold:
            self.get_logger().warn(f"Obstacle DETECTED at {min_d:.2f} m < {threshold:.2f} m! Stopping.")
            return True
        return False

    # ---------- 미션 콜백 함수 ----------
    def on_left_detection(self, msg: String):
        self.left_detection_order = msg.data.split(',') if msg.data else []

    def on_right_detection(self, msg: String):
        self.right_detection_order = msg.data.split(',') if msg.data else []
        
    # -------------------- Control Loop --------------------
    def control_loop(self):
        is_in_t_prepare_zone = self._in_range_check(self.current_wp_idx, self.t_parking_prepare_ranges)
        is_in_p_prepare_zone = self._in_range_check(self.current_wp_idx, self.parallel_parking_prepare_ranges)

        if is_in_t_prepare_zone and not self.mission_reset_sent['t_parking']:
            self.get_logger().info("Approaching T-Parking zone. Resetting left camera detection.")
            self.pub_reset_left.publish(Empty())
            self.mission_reset_sent['t_parking'] = True
        
        # T자 주차 미션/준비 구간을 완전히 벗어나면, 다음 랩을 위해 리셋 플래그 초기화
        elif not is_in_t_prepare_zone and not self._in_range_check(self.current_wp_idx, self.t_parking_mission_ranges):
            self.mission_reset_sent['t_parking'] = False

        # 평행 주차 준비 구간에 처음 진입하는 순간, 리셋 신호 전송
        if is_in_p_prepare_zone and not self.mission_reset_sent['p_parking']:
            self.get_logger().info("Approaching Parallel Parking zone. Resetting right camera detection.")
            self.pub_reset_right.publish(Empty())
            self.mission_reset_sent['p_parking'] = True

        # 평행 주차 미션/준비 구간을 완전히 벗어나면, 다음 랩을 위해 리셋 플래그 초기화
        elif not is_in_p_prepare_zone and not self._in_range_check(self.current_wp_idx, self.parallel_parking_mission_ranges):
            self.mission_reset_sent['p_parking'] = False

        if self.is_in_replay_mode:
            return
        if self._in_range_check(self.current_wp_idx, self.t_parking_mission_ranges):
            self.get_logger().info(f"Entering T-Parking Zone. Detections: {self.left_detection_order}")
            csv_path_to_load = None
            if 'cone' in self.left_detection_order:
                self.get_logger().info("Decision: Left blocked (cone first). Selecting RIGHT-TURN replay.")
                csv_path_to_load = self.t_parking_left_csv
            else:
                self.get_logger().info("Decision: Left clear (yellow first). Selecting LEFT-TURN replay.")
                csv_path_to_load = self.t_parking_right_csv

            if csv_path_to_load:
                self.start_replay_mission(csv_path_to_load, "t_parking")
            else:
                self.get_logger().warn("No valid CSV path for T-Parking, skipping mission.")
            return
        
        if self._in_range_check(self.current_wp_idx, self.parallel_parking_mission_ranges):
            self.get_logger().info(f"Entering P-Parking Zone. Detections: {self.right_detection_order}")
            csv_path_to_load = None 
            if 'cone' in self.right_detection_order:
                self.get_logger().info("Decision: Right blocked (cone first). Selecting LEFT-TURN replay.")
                csv_path_to_load = self.p_parking_left_csv
            else:
                self.get_logger().warn("No decisive detection on RIGHT. Defaulting to RIGHT-TURN.")
                csv_path_to_load = self.p_parking_right_csv
            if csv_path_to_load:
                self.start_replay_mission(csv_path_to_load, "p_parking")
            else:
                 self.get_logger().warn("No valid CSV path for P-Parking, skipping mission.")
            return

        if self._in_lidar_activation_range(self.current_wp_idx):
            if self._check_for_obstacle():
                self.stop_robot()
                return
            
        is_in_tl_zone = self._in_traffic_light_green_go_range(self.current_wp_idx) or \
                        self._in_traffic_light_left_go_range(self.current_wp_idx)

        if self.is_crossing_intersection and not is_in_tl_zone:
            self.get_logger().info("Intersection cleared. Resetting crossing state.")
            self.is_crossing_intersection = False
        
        if not self.is_crossing_intersection and is_in_tl_zone:
            if self._in_traffic_light_green_go_range(self.current_wp_idx):
                if self.traffic_light_state != "Green":
                    self.get_logger().warn(f"TL (Green-Go): Waiting for Green. Current: {self.traffic_light_state}. Stopping.")
                    self.stop_robot()
                    return
                else:
                    self.get_logger().info("TL (Green-Go): Green Signal DETECTED. Start crossing.")
                    self.is_crossing_intersection = True
            elif self._in_traffic_light_left_go_range(self.current_wp_idx):
                if self.traffic_light_state != "Left":
                    self.get_logger().warn(f"TL (Left-Go): Waiting for Left. Current: {self.traffic_light_state}. Stopping.")
                    self.stop_robot()
                    return
                else:
                    self.get_logger().info("TL (Left-Go): Left Signal DETECTED. Start crossing.")
                    self.is_crossing_intersection = True

        # 정지 상태 처리
        if self.stopped_forever:
            self.stop_robot()
            return

        if self.paused_until is not None:
            if self.get_clock().now() < self.paused_until:
                # 멈추는 대신, 뒤로 밀리는 것을 방지할 약한 전진 PWM을 줌
                # 30~50 사이의 값을 테스트하며 최적값을 찾으세요.
                hill_hold_pwm = 10 
                self.get_logger().info(f"Pausing on hill, applying hold power: {hill_hold_pwm}", throttle_duration_sec=1)
                self.publish_motion_command(0, hill_hold_pwm, hill_hold_pwm)
                return 
            else:
                self.paused_until = None  # 일시정지 해제

        if self.goal_xy_local is None or self.curr_xy is None:
            return

        if self.last_fix_time is not None and (self.get_clock().now() - self.last_fix_time) > Duration(seconds=2.0):
            self.get_logger().warn("GPS timeout >2s, stopping")
            self.stop_robot()
            return

        # 현재 WP와 거리
        # --- 제어 기준점 보정: GPS 위치를 차량 중심(또는 원하는 기준점)으로 이동 ---
        cx_raw, cy_raw = self.curr_xy  # GPS 실제 위치
        cx, cy = cx_raw, cy_raw        # 기본값
        if self.curr_yaw is not None:
            ux = math.cos(self.curr_yaw)   # 차량 전방 단위벡터 x성분
            uy = math.sin(self.curr_yaw)   # 차량 전방 단위벡터 y성분
            # 차량 좌측 단위벡터 = (-uy, +ux)
            fwd = self.ref_from_gps_forward_m
            lft = self.ref_from_gps_left_m
            # GPS -> 기준점 오프셋의 월드좌표 성분
            dx = fwd*ux + lft*(-uy)
            dy = fwd*uy + lft*(+ux)
            # 기준점 좌표 = GPS 좌표 - 오프셋
            cx = cx_raw - dx
            cy = cy_raw - dy

        gx, gy = self.goal_xy_local
        dx = gx - cx
        dy = gy - cy
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f"[Chk] idx={self.current_wp_idx}, dist={dist:.2f}m, "
            f"goal=({gx:.1f},{gy:.1f}), pos=({cx:.1f},{cy:.1f})"
        )
        # 도착 판정
        if dist < self.arrive_dist_m:
            self.get_logger().info(f"Reached WP{self.current_wp_idx}")

            # ✨ 도착 시 정지/일시정지/진행 로직
            if self.current_wp_idx in self.stop_wp_indices:
                self.get_logger().info(f"WP{self.current_wp_idx} is a STOP point. Stopping forever.")
                self.stopped_forever = True
                self.stop_robot()
                return

            if self.current_wp_idx in self.pause_wp_indices and self.pause_seconds > 0.0:
                self.paused_until = self.get_clock().now() + Duration(seconds=float(self.pause_seconds))
                self.get_logger().info(f"WP{self.current_wp_idx} is a PAUSE point. Pausing for {self.pause_seconds:.1f}s.")
                self.stop_robot()
                # 다음 루프에서 시간이 지나면 자동 재개
                # 다음 웨이포인트로는 일단 넘어가 두는 편이 자연스럽다.
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.get_logger().info("All waypoints reached. Stopping.")
                    self.stopped_forever = True
                else:
                    self._update_goal_xy_local()
                return

            # 기본: 다음 웨이포인트로 진행
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Stopping.")
                self.stopped_forever = True
                self.stop_robot()
                return
            else:
                self._update_goal_xy_local()
                return

        # 방위각 계산 (후진 모드면 π 더해줌)
        desired_yaw = math.atan2(dy, dx)

        #reverse_mode = False
        reverse_mode = self._in_range_check(self.current_wp_idx, self.reverse_ranges)
        # 후진 구간으로 처음 진입하는 경우, 1초간 정지
        if reverse_mode and not self.is_entering_reverse:
            self.get_logger().info(
                f"Entering reverse zone at target idx={self.current_wp_idx}. Pausing 1s."
            )
            self.paused_until = self.get_clock().now() + Duration(seconds=1.0)
            self.is_entering_reverse = True
            self.stop_robot()
            return
        
        # 후진 구간이 끝나면 플래그 리셋
        if not reverse_mode:
            self.is_entering_reverse = False

        if reverse_mode:
            desired_yaw = wrap_pi(desired_yaw + math.pi)  # ✨ 후진 조향
        # 조향 계산
        if self.curr_yaw is None:
            steering = 0
        else:
            #err = wrap_pi(desired_yaw - self.curr_yaw)
            #w   = max(-self.max_angular, min(self.max_angular, self.k_w * err))
            dt = self.timer_period # 제어 주기 (시간 간격)
            err = wrap_pi(desired_yaw - self.curr_yaw)

            # P (Proportional) 항
            p_term = self.k_p * err

            # I (Integral) 항
            self.integral_error += err * dt
            # Integral Windup 방지: I항 누적값 제한
            self.integral_error = max(self.integral_min, min(self.integral_max, self.integral_error))
            i_term = self.k_i * self.integral_error

            # D (Derivative) 항
            derivative_error = (err - self.previous_error) / dt
            d_term = self.k_d * derivative_error

            # 다음 계산을 위해 현재 오차를 이전 오차로 저장
            self.previous_error = err

            # PID 출력 결합
            w = p_term + i_term + d_term
            w = max(-self.max_angular, min(self.max_angular, w))
            steering_f = (w / self.max_angular) * self.max_steering
            steering_f = max(-self.max_steering, min(self.max_steering, steering_f))
            alpha = self.steering_alpha
            steering_lp = (1.0 - alpha) * self._prev_steering_f + alpha * steering_f
            self._prev_steering_f = steering_lp
            steering_i = int(round(steering_lp))
            steering_i = max(-self.max_steering, min(self.max_steering, steering_i))
            if abs(steering_i - self._prev_steering_i) < self.steering_step_hyst:
                steering_i = self._prev_steering_i
            delta = steering_i - self._prev_steering_i
            if delta > self.steering_rate_limit:
                steering_i = self._prev_steering_i + self.steering_rate_limit
            elif delta < -self.steering_rate_limit:
                steering_i = self._prev_steering_i - self.steering_rate_limit

            self._prev_steering_i = steering_i
            steering = steering_i
            # 디버깅 로그
            self.get_logger().info(
                f"[Controller idx={self.current_wp_idx} {'REV' if reverse_mode else 'FWD'}] "
                f"Desired={math.degrees(desired_yaw):.1f}, "
                f"Current={math.degrees(self.curr_yaw):.1f}, "
                f"Error={math.degrees(err):.1f} -> Steering={steering}"
            )

        final_steering = steering
        if self._is_in_initial_drive:
            # 저속 출발이 처음 시작되는 순간이라면, 종료 시간 설정
            if self._initial_drive_end_time is None:
                duration = Duration(seconds=self.initial_drive_duration_sec)
                self._initial_drive_end_time = self.get_clock().now() + duration
                self.get_logger().info(f"Starting initial drive at {self.initial_drive_pwm} PWM for {self.initial_drive_duration_sec} seconds.")

            # 아직 저속 주행 시간이 끝나지 않았다면
            if self.get_clock().now() < self._initial_drive_end_time:
                target_pwm = self.initial_drive_pwm
            # 저속 주행 시간이 끝났다면
            else:
                self.get_logger().info("Initial drive complete. Switching to target speed.")
                self._is_in_initial_drive = False # 저속 출발 모드 해제
                target_pwm = 0 # 한 틱은 0으로 보내서 충격을 줄이거나, 바로 아래 로직으로 넘어가도 됨
        
        # 2. 저속 출발 모드가 아니라면, 원래 로직대로 목표 속도 계산
        if not self._is_in_initial_drive:
            is_in_t_prepare = self._in_range_check(self.current_wp_idx, self.t_parking_prepare_ranges)
            is_in_p_prepare = self._in_range_check(self.current_wp_idx, self.parallel_parking_prepare_ranges)
            is_in_lidar_zone = self._in_lidar_activation_range(self.current_wp_idx)

            # 속도 선택
            if reverse_mode:
                pwm = self.speed_reverse_pwm
            # 1순위: 주차 준비 구역에 있다면 지정된 감속 속도 사용
            elif is_in_t_prepare or is_in_p_prepare:
                self.get_logger().info(f"In Prepare Zone, setting speed to {self.speed_prepare_zone_pwm}", throttle_duration_sec=2)
                pwm = self.speed_prepare_zone_pwm
            # 2순위: 라이다 활성화 구간이라면 지정된 속도 사용
            elif is_in_lidar_zone:
                self.get_logger().info(f"In LIDAR zone, setting speed to {self.speed_lidar_zone_pwm}", throttle_duration_sec=2)
                pwm = self.speed_lidar_zone_pwm
            # 3순위: 일반 전진 주행
            else:
                pwm = self.speed_forward_pwm
            target_pwm = pwm

        # 최종 PWM 값 제한 및 전송 (이 부분은 동일)
        final_pwm = max(-self.max_abs_pwm, min(self.max_abs_pwm, target_pwm))

        # 최종적으로 업데이트된 PWM 값으로 명령 전송
        self.publish_motion_command(final_steering, int(final_pwm), int(final_pwm))

    def _update_goal_xy_local(self):
        """현재 current_wp_idx 기준 goal_xy_local 갱신"""
        if self.current_wp_idx < len(self.waypoints):
            if self.origin_e0n0 is None:
                return
            e0, n0 = self.origin_e0n0
            g: Utm = toUtm8(latlon=self.waypoints[self.current_wp_idx][0],
                            lon=self.waypoints[self.current_wp_idx][1])
            ge, gn = g.eastingnorthing
            self.goal_xy_local = (ge - e0, gn - n0)

    def publish_motion_command(self, steering: int, left_pwm: int, right_pwm: int):
        msg = MotionCommand()
        msg.steering = int(steering)
        msg.left_speed = int(left_pwm)
        msg.right_speed = int(right_pwm)
        self.pub_mc.publish(msg)

    def stop_robot(self):
        self.publish_motion_command(0, 0, 0)
        self._is_in_initial_drive = True 
        self._initial_drive_end_time = None

def main(args=None):
    rclpy.init(args=args)
    node = GPSGotoNode()
    node.stop_robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
