# ~/hlfma2025/src/decision_making_pkg/decision_making_pkg/fake_gps_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import csv
import time

class FakeGPSPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps_publisher')
        
        # params.yaml에서 웨이포인트 파일 경로를 읽어옵니다.
        self.declare_parameter('waypoint_csv', 'waypoints.csv')
        csv_path = self.get_parameter('waypoint_csv').get_parameter_value().string_value
        
        self.waypoints = self.load_waypoints_from_csv(csv_path)
        self.current_wp_idx = 0
        
        self.publisher_ = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.timer = self.create_timer(0.2, self.publish_fake_gps) # 0.2초마다 GPS 발행
        
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded. Shutting down.")
            self.destroy_node()
        else:
            self.get_logger().info(f"Fake GPS Publisher started. Will publish {len(self.waypoints)} waypoints sequentially.")

    def load_waypoints_from_csv(self, path):
        waypoints = []
        try:
            with open(path, newline='') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    waypoints.append({'lat': float(row[0]), 'lon': float(row[1])})
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints for fake GPS.")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV {path}: {e}")
        return waypoints

    def publish_fake_gps(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info("Finished all fake waypoints. Stopping publisher.")
            self.timer.cancel()
            return

        waypoint = self.waypoints[self.current_wp_idx]
        
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = 0  # GPS 수신 상태 양호
        msg.latitude = waypoint['lat']
        msg.longitude = waypoint['lon']
        msg.altitude = 0.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published fake GPS for Waypoint #{self.current_wp_idx + 1}: Lat={waypoint['lat']}, Lon={waypoint['lon']}")
        
        # 다음 웨이포인트로 인덱스 이동
        self.current_wp_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()