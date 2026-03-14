import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
# --- ADDED: Import for handling multiple callbacks safely ---
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces_pkg.msg import PathPlanningResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

import cv2
import numpy as np

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        # --- Use a ReentrantCallbackGroup for thread-safe callbacks ---
        self.callback_group = ReentrantCallbackGroup()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.path_sub = self.create_subscription(
            PathPlanningResult, 
            'path_planning_result', 
            self.path_callback, 
            self.qos_profile,
            callback_group=self.callback_group)
        
        self.steer_sub = self.create_subscription(
            Float32, 
            'steering_angle', 
            self.steer_callback, 
            self.qos_profile,
            callback_group=self.callback_group)

        self.publisher = self.create_publisher(Image, 'path_image', self.qos_profile)
        
        self.cv_bridge = CvBridge()
        self.black_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.current_steer = 0.0

    def steer_callback(self, msg: Float32):
        """Callback to update the current steering value."""
        self.current_steer = msg.data

    def path_callback(self, path_msg: PathPlanningResult):
        """Callback to draw the path and steering angle, then publish."""
        # Create a fresh black image for each frame
        vis_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw the path points if any exist
        for p in path_msg.path_points:
            cv2.circle(vis_img, (int(p.x), int(p.y)), 3, (0, 0, 255), -1)
        
        # Overlay steering value text onto the image
        steer_text = f"Steer: {self.current_steer:.2f}"
        cv2.putText(vis_img, steer_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
        # Convert to ROS Image message and publish
        img_msg = self.cv_bridge.cv2_to_imgmsg(vis_img, "bgr8")
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        # Use a MultiThreadedExecutor to handle multiple callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()