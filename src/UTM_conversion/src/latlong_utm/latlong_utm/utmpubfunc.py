#A node to publish utm coordinates for conversion
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

utmcoord = [[395201, 5673135, 32, 'U'],[428631, 4438653, 32, 'F'],[421184, 4983436, 31, 'T'],[500000, 5205164, 39, 'T'],[396775, 2433164, 27, 'Q']]
index = 0


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%s %s %s %s' %(utmcoord[index][0],utmcoord[index][1],utmcoord[index][2],utmcoord[index][3])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    global index

    while (index< len(utmcoord)):
        rclpy.spin_once(minimal_publisher)
        index = index +1

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()