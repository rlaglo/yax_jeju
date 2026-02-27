#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import threading
import time


class TurtleBotJoyCombined(Node):
    def __init__(self):
        super().__init__('turtlebot_joy_combined')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize pygame for joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected! Please connect one and restart.")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(f"Joystick detected: {self.joystick.get_name()}")

        # Speed scaling factors
        self.linear_speed_scale = 2.0    # m/s
        self.angular_speed_scale = 1.0   # rad/s

        # Deadzone threshold (ignore small joystick noise)
        self.deadzone = 0.05

        # Start joystick reading thread
        self.running = True
        self.thread = threading.Thread(target=self.read_joystick_loop)
        self.thread.start()

    def apply_deadzone(self, value):
        """Return 0 if within deadzone range."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def read_joystick_loop(self):
        rate = 0.05  # 20 Hz
        while self.running:
            pygame.event.pump()  # process joystick events

            # Typical joystick mapping
            linear_axis = self.joystick.get_axis(1)
            angular_axis = -self.joystick.get_axis(3)

            # Apply deadzone to remove noise
            linear_axis = -self.apply_deadzone(linear_axis)
            angular_axis = self.apply_deadzone(angular_axis)

            twist = Twist()
            twist.linear.x = linear_axis * self.linear_speed_scale
            twist.angular.z = angular_axis * self.angular_speed_scale

            self.cmd_vel_pub.publish(twist)

            time.sleep(rate)

    def destroy_node(self):
        self.running = False
        self.thread.join()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotJoyCombined()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
