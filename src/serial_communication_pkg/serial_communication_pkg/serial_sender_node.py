import time                                   # 지연(sleep) 등 시간 관련 함수 사용
import serial                                 # PySerial: 시리얼 포트 통신
import rclpy                                  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node                   # ROS 2 노드 기본 클래스
from rclpy.qos import QoSProfile              # QoS 설정 객체
from rclpy.qos import QoSHistoryPolicy        # QoS: 히스토리 정책
from rclpy.qos import QoSDurabilityPolicy     # QoS: 내구성(지속성) 정책
from rclpy.qos import QoSReliabilityPolicy    # QoS: 신뢰성 정책
from interfaces_pkg.msg import MotionCommand  # 사용자 정의 메시지(조향/좌우 속도)
from .lib import protocol_convert_func_lib as PCFL  # 시리얼 프로토콜 변환 유틸(문자열 생성)

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"       # 구독할 토픽명(제어 명령 수신)

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT = '/dev/ttyUSB0'                         # 시리얼 포트 경로(아두이노가 잡힌 디바이스)
#----------------------------------------------

ser = serial.Serial(PORT, 115200, timeout=1)  # 시리얼 포트 오픈(115200bps, 읽기 타임아웃 1초)
time.sleep(1)                                 # 보드 리셋/포트 안정화 대기(1초)

class SerialSenderNode(Node):                 # ROS 2 노드: 토픽 구독 → 시리얼 송신
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')    # 노드 이름 등록

    self.declare_parameter('sub_topic', sub_topic)  # 파라미터 선언(토픽명 커스터마이즈용)

    # 파라미터에서 실제 구독할 토픽명 읽기
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value

    # QoS 설정: 신뢰성 높음(REL), 최근 1개만 유지(KEEP_LAST, depth=1),揮発성(재시작 시 과거 메시지X)
    qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.RELIABLE,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1
    )

    # 토픽 구독 생성: MotionCommand 메시지를 수신하면 data_callback 호출
    self.subscription = self.create_subscription(
      MotionCommand, self.sub_topic, self.data_callback, qos_profile
    )


  def data_callback(self, msg):               # 수신 콜백: 메시지를 시리얼 포맷으로 바꿔 전송
    steering = msg.steering                   # 조향 값
    left_speed = msg.left_speed               # 좌측 속도
    right_speed = msg.right_speed             # 우측 속도

    serial_msg = PCFL.convert_serial_message( # 시리얼 전송용 문자열 생성(프로토콜에 맞춤)
      steering, left_speed, right_speed
    )
    ser.write(serial_msg.encode())            # 바이트로 인코딩하여 시리얼로 송신


def main(args=None):
  rclpy.init(args=args)                       # ROS 2 초기화
  node = SerialSenderNode()                   # 노드 인스턴스 생성
  try:
      rclpy.spin(node)                        # 콜백 처리 루프(CTRL+C까지 대기/실행)

  except KeyboardInterrupt:                   # 사용자가 중단(CTRL+C) 시
      print("\n\nshutdown\n\n")
      steering = 0                            # 안전 정지 명령(조향 0)
      left_speed = 0                          # 좌속도 0
      right_speed = 0                         # 우속도 0
      message = PCFL.convert_serial_message(  # 정지 패킷 생성
        steering, left_speed, right_speed
      )
      ser.write(message.encode())             # 정지 패킷 송신
      pass

  finally:
    ser.close()                               # 시리얼 포트 닫기(자원 정리)
    print('closed')

  node.destroy_node()                         # 노드 파괴(정리)
  rclpy.shutdown()                            # ROS 2 종료


if __name__ == '__main__':
  main()                                      # 스크립트 직접 실행 시 main() 진입
