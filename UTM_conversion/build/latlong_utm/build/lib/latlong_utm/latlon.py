import rclpy
from rclpy.node import Node

from std_msgs.msg import String

latcoord = [[51.2, 7.5],[-50.2, 8],[45,2],[47,51],[22,-22]]
index = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic2', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%s %s' %(latcoord[index][0],latcoord[index][1])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    global index
    minimal_publisher = MinimalPublisher()

    while (index< len(latcoord)):
        rclpy.spin_once(minimal_publisher)
        index = index +1

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()