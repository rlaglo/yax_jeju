import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand

class CmdVelToMotionCommand(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motion_command')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_ = self.create_publisher(MotionCommand, 'motion_command', 10)

    def cmd_vel_callback(self, msg):
        motion_command = MotionCommand()

        # This is a simple conversion, you may need to adjust the scaling factors
        # based on your robot's characteristics (wheelbase, wheel radius, etc.)
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Convert angular velocity to a steering angle (e.g., -100 to 100)
        # This is a placeholder, you'll need to find a suitable conversion
        # for your robot's steering mechanism.
        steering = int(angular_velocity * 50) # Example scaling factor

        # For a differential drive robot:
        # You'll need to determine your robot's wheel base
        wheel_base = 0.5 # meters, example value
        left_speed = int((linear_velocity - (angular_velocity * wheel_base / 2.0)) * 100)
        right_speed = int((linear_velocity + (angular_velocity * wheel_base / 2.0)) * 100)

        motion_command.steering = max(-100, min(100, steering))
        motion_command.left_speed = left_speed
        motion_command.right_speed = right_speed

        self.publisher_.publish(motion_command)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotionCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()