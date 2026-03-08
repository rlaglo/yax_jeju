# parking_replay_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from interfaces_pkg.msg import MotionCommand
import csv
import time

class ParkingReplayNode(Node):
    def __init__(self):
        super().__init__('parking_replay_node')
        
        # 리플레이할 CSV 파일 경로를 받는 Subscriber
        self.create_subscription(String, '/trigger_parking_replay', self.replay_callback, 10)
        
        # 미션 완료 상태를 알리는 Publisher
        self.status_publisher = self.create_publisher(Bool, '/parking_replay_status', 10)
        
        # 차량 제어 명령을 보내는 Publisher
        self.mc_publisher = self.create_publisher(MotionCommand, '/motion_command', 10)
        
        self.get_logger().info('Parking Replay Node is ready.')

    def replay_callback(self, msg):
        csv_file_path = msg.data
        self.get_logger().info(f"Starting replay of parking mission from: {csv_file_path}")

        try:
            with open(csv_file_path, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for row in csv_reader:
                    # driving_log.csv 파일의 내용을 읽어 MotionCommand 메시지 발행
                    steering = int(row['steering'])
                    left_speed = int(row['left_speed'])
                    right_speed = int(row['right_speed'])

                    mc_msg = MotionCommand()
                    mc_msg.steering = steering
                    mc_msg.left_speed = left_speed
                    mc_msg.right_speed = right_speed
                    self.mc_publisher.publish(mc_msg)
                    
                    time.sleep(0.02) # replay.py와 동일한 딜레이

            self.get_logger().info("Parking replay finished successfully.")
        
        except Exception as e:
            self.get_logger().error(f"Failed to execute replay: {e}")

        finally:
            # 차량 정지
            stop_msg = MotionCommand()
            stop_msg.steering = 0
            stop_msg.left_speed = 0
            stop_msg.right_speed = 0
            self.mc_publisher.publish(stop_msg)

            # 미션 완료 메시지 발행
            status_msg = Bool()
            status_msg.data = True
            self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()