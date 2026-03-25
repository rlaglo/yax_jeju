#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from interfaces_pkg.msg import MotionCommand

class ForwardDriverNode(Node):
    """
    아주 단순한 '전진 전용' 노드.
    주기적으로 MotionCommand를 퍼블리시하여 차량을 앞으로 움직이게 합니다.
    종료(CTRL+C) 시에는 즉시 정지 명령(0,0,0)을 1회 발행합니다.
    """

    def __init__(self):
        super().__init__('forward_driver_node')

        # 파라미터
        self.pub_topic = self.declare_parameter('pub_topic', 'topic_control_signal').value
        self.timer_period = float(self.declare_parameter('timer', 0.1).value)  # s
        self.steering = int(self.declare_parameter('steering', 0).value)      # 직진=0
        self.left_speed = int(self.declare_parameter('left_speed', 255).value)
        self.right_speed = int(self.declare_parameter('right_speed', 255).value)

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 퍼블리셔
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, qos)

        # 타이머
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f"[ForwardDriver] publishing to '{self.pub_topic}' "
            f"(steer={self.steering}, L={self.left_speed}, R={self.right_speed}, "
            f"period={self.timer_period}s)"
        )

    def timer_callback(self):
        msg = MotionCommand()
        msg.steering = int(self.steering)
        msg.left_speed = int(self.left_speed)
        msg.right_speed = int(self.right_speed)
        self.publisher.publish(msg)
        # 필요하면 아래 로그 주석 처리 (스팸 방지)
        self.get_logger().info(f"MC -> s={msg.steering}, L={msg.left_speed}, R={msg.right_speed}")

    def stop_once(self):
        # 안전 정지 신호 1회
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.publisher.publish(msg)
        self.get_logger().info("STOP sent (0,0,0)")

def main(args=None):
    rclpy.init(args=args)
    node = ForwardDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_once()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
