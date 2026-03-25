import rclpy
from rclpy.node import Node
import utm

from std_msgs.msg import String

dat = []
l = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic2', 10)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        x = dat[l][0]
        y = dat[l][1]
        zn = dat[l][2]
        zl = dat[l][3]
        lat, lon = utm.to_latlon(x,y,zn,zl)
        msg.data = '%s %s' %(lat,lon)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        dat.append(list([i for i in msg.data.split(" ")]))
        global l
        dat[l][0] = float(dat[l][0])
        dat[l][1] = float(dat[l][1])
        dat[l][2] = float(dat[l][2])
        minimal_publisher = MinimalPublisher()
        while l < len(dat):
        	rclpy.spin_once(minimal_publisher)
        	l = l+1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
