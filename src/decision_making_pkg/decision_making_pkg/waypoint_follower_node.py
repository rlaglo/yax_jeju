import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from interfaces_pkg.msg import MotionCommand
import csv
import math
import utm

class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # Parameters
        self.declare_parameter('waypoints_file', '/home/jeanho/waypoints.csv')
        self.declare_parameter('utm_zone_number', 52) # Default for South Korea
        self.declare_parameter('utm_zone_letter', 'S') # Default for South Korea


        # Variables
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.load_waypoints()

        # Subscribers
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)

        # Publishers
        self.motion_command_publisher = self.create_publisher(MotionCommand, 'motion_command', 10)

    def load_waypoints(self):
        file_path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        utm_zone_number = self.get_parameter('utm_zone_number').get_parameter_value().integer_value
        utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip header
            for row in reader:
                easting = float(row[0])
                northing = float(row[1])
                lat, lon = utm.to_latlon(easting, northing, utm_zone_number, utm_zone_letter)
                self.waypoints.append((lat, lon))

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            # Stop the vehicle
            self.publish_motion_command(0, 0, 0)
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]

        distance = self.haversine_distance(self.current_lat, self.current_lon, target_lat, target_lon)
        bearing = self.calculate_bearing(self.current_lat, self.current_lon, target_lat, target_lon)

        # Simple proportional controller
        steering_angle = self.normalize_bearing(bearing)
        speed = 50 if distance > 1.0 else 0 # Stop when close to the waypoint

        if distance < 1.0:
            self.current_waypoint_index += 1

        self.publish_motion_command(steering_angle, speed, speed)

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371e3 # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
            math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
        return math.degrees(math.atan2(y, x))

    def normalize_bearing(self, bearing):
        # Normalize to a steering command range (e.g., -100 to 100)
        return int(max(-100, min(100, bearing)))

    def publish_motion_command(self, steering, left_speed, right_speed):
        msg = MotionCommand()
        msg.steering = steering
        msg.left_speed = left_speed
        msg.right_speed = right_speed
        self.motion_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()