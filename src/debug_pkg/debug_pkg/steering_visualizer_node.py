import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import PathPlanningResult, MotionCommand
import cv2
from cv_bridge import CvBridge

#---------------Variable Setting---------------
SUB_ROI_IMAGE_TOPIC = "roi_image"
SUB_SPLINE_PATH_TOPIC = "path_planning_result"
SUB_MOTION_COMMAND_TOPIC = "topic_control_signal"
PUB_TOPIC_NAME = "steering_visualized_img" # A new topic for the visualized output

#----------------------------------------------
class SteeringVisualizerNode(Node):
    def __init__(self):
        super().__init__('steering_visualizer_node')

        # Declare parameters
        self.sub_roi_image_topic = self.declare_parameter('sub_roi_image_topic', SUB_ROI_IMAGE_TOPIC).value
        self.sub_spline_path_topic = self.declare_parameter('sub_spline_path_topic', SUB_SPLINE_PATH_TOPIC).value
        self.sub_motion_command_topic = self.declare_parameter('sub_motion_command_topic', SUB_MOTION_COMMAND_TOPIC).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        # Configure QoS profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Initialize CvBridge
        self.cv_bridge = CvBridge()

        # Create subscribers
        self.roi_image_sub = self.create_subscription(
            Image, self.sub_roi_image_topic, self.image_callback, self.qos_profile)
        
        self.spline_path_sub = self.create_subscription(
            PathPlanningResult, self.sub_spline_path_topic, self.path_callback, self.qos_profile)

        self.motion_command_sub = self.create_subscription(
            MotionCommand, self.sub_motion_command_topic, self.motion_command_callback, self.qos_profile)

        # Create publisher
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)

        # Initialize variables to store data
        self.current_image = None
        self.spline_path = None
        self.steering_angle = 0.0

    def image_callback(self, msg: Image):
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.visualize_data()
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")

    def path_callback(self, msg: PathPlanningResult):
        self.spline_path = list(zip(msg.x_points, msg.y_points))
        self.visualize_data()

    def motion_command_callback(self, msg: MotionCommand):
        self.steering_angle = float(msg.steering)
        self.visualize_data()

    def visualize_data(self):
        if self.current_image is None:
            return

        display_image = self.current_image.copy()

        # Draw the path if it's available
        if self.spline_path:
            for (x, y) in self.spline_path:
                cv2.circle(display_image, (int(x), int(y)), 5, (0, 0, 255), -1)

        # Display the steering angle
        steer_val_text = f"Steer: {self.steering_angle:.2f}"
        cv2.putText(display_image, steer_val_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Publish the final image
        try:
            output_msg = self.cv_bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
            self.publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SteeringVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()