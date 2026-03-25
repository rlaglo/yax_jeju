import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class RoiMergerNode(Node):
    def __init__(self):
        super().__init__('roi_merger_node')

        self.bridge = CvBridge()
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers for the two ROI image topics
        self.roi_sub_0 = Subscriber(self, Image, 'roi_image_0', qos_profile=self.qos_profile)
        self.roi_sub_2 = Subscriber(self, Image, 'roi_image_2', qos_profile=self.qos_profile)

        # Synchronize the incoming messages
        self.ts = ApproximateTimeSynchronizer([self.roi_sub_0, self.roi_sub_2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # Publisher for the merged image
        self.merged_roi_pub = self.create_publisher(Image, 'merged_roi_image', self.qos_profile)

    def image_callback(self, msg0, msg2):
        try:
            # Convert ROS Image messages to OpenCV images
            roi_image_0 = self.bridge.imgmsg_to_cv2(msg0, 'mono8')
            roi_image_2 = self.bridge.imgmsg_to_cv2(msg2, 'mono8')

            # Merge images side-by-side
            merged_image = np.hstack([roi_image_0, roi_image_2])

            # Convert the merged OpenCV image back to a ROS Image message
            merged_msg = self.bridge.cv2_to_imgmsg(merged_image, 'mono8')
            merged_msg.header.stamp = self.get_clock().now().to_msg()
            self.merged_roi_pub.publish(merged_msg)

            # Display the merged image in a window
            cv2.imshow('Merged ROI Image', merged_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Failed to process and merge images: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RoiMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()