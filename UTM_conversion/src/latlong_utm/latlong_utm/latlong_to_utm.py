#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix   # GNSS 위도·경도·고도 데이터를 담는 ROS 표준 메시지
from std_msgs.msg import String         # 문자열을 퍼블리시하기 위한 메시지
import utm                              # 위도·경도 → UTM 변환 라이브러리 (pip install utm)

# -------------------------------
# FixToUtm 노드 정의
# -------------------------------
class FixToUtm(Node):
    """
    NavSatFix 타입의 /ublox_gps_node/fix 토픽을 구독하고,
    위도·경도를 UTM 좌표계로 변환하여 /topic1 (String)으로 퍼블리시하는 노드
    """

    def __init__(self):
        super().__init__('fix_to_utm')   # 노드 이름을 'fix_to_utm'으로 등록

        # /ublox_gps_node/fix 구독 (NavSatFix 타입)
        # 새로운 메시지가 들어올 때마다 self.cb 콜백이 실행됨
        self.sub = self.create_subscription(
            NavSatFix,                   # 메시지 타입
            '/ublox_gps_node/fix',       # 구독할 토픽 이름
            self.cb,                     # 콜백 함수
            10)                          # QoS 큐 크기

        # /topic1 퍼블리셔 생성 (출력은 String 타입)
        # latlong_utm 같은 기존 코드와 호환되도록 String으로 발행
        self.pub = self.create_publisher(String, 'utm', 10)

    # -------------------------------
    # 콜백 함수: NavSatFix 수신 시 실행
    # -------------------------------
    def cb(self, msg: NavSatFix):
        # NavSatFix 메시지에서 위도(latitude), 경도(longitude) 추출
        lat, lon = msg.latitude, msg.longitude

        # 값이 None/NaN/Inf인 경우 변환하지 않고 경고 출력 후 리턴
        if any(map(lambda v: v is None or math.isnan(v) or math.isinf(v), [lat, lon])):
            self.get_logger().warn('Invalid lat/lon, skip.')
            return

        # utm.from_latlon() 함수로 위경도 → UTM 변환
        easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
        # easting  : UTM 동쪽 좌표 (m)
        # northing : UTM 북쪽 좌표 (m)
        # zone_num : UTM 존 번호 (예: 52)
        # zone_letter : UTM 존 문자 (예: S)

        # 변환된 좌표를 문자열로 묶어서 메시지 생성
        out = String()
        out.data = f'{easting} {northing} {zone_num} {zone_letter}'

        # /topic1 토픽으로 발행
        self.pub.publish(out)

        # 로그에도 출력
        self.get_logger().info(f'UTM -> {out.data}')

# -------------------------------
# 메인 함수
# -------------------------------
def main():
    rclpy.init()             # rclpy 초기화
    node = FixToUtm()        # FixToUtm 노드 인스턴스 생성
    try:
        rclpy.spin(node)     # spin()으로 콜백 루프 진입 → 메시지를 계속 처리
    finally:
        node.destroy_node()  # 종료 시 노드 파괴
        rclpy.shutdown()     # rclpy 종료

# -------------------------------
# 엔트리포인트
# -------------------------------
if __name__ == '__main__':
    main()
