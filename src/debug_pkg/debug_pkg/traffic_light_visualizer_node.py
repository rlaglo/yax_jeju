import cv2
import random
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, BoundingBox2D, Detection

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "traffic_light_detections"
SUB_IMAGE_TOPIC_NAME = "image_raw"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "traffic_light_bounding_box_image"

# ----------------------------------------------

class TrafficLightVisualizer(Node):
    def __init__(self):
        super().__init__('traffic_light_visualizer_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.cv_bridge = CvBridge()
        self.class_colors = {}  # To store a color for each class

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        
    def get_color(self, class_name: str) -> Tuple[int, int, int]:
        if class_name not in self.class_colors:
            # Assign a random color
            self.class_colors[class_name] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        return self.class_colors[class_name]

    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        for detection in detection_msg.detections:
            
            # Bounding Box
            bbox = detection.bbox
            center_x, center_y = int(bbox.center.position.x), int(bbox.center.position.y)
            width, height = int(bbox.size.x), int(bbox.size.y)
            
            x1 = center_x - width // 2
            y1 = center_y - height // 2
            x2 = center_x + width // 2
            y2 = center_y + height // 2

            color = self.get_color(detection.class_name)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
            
            # Label
            label = f'{detection.class_name}: {detection.score:.2f}'
            label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            y1_label = max(y1, label_size[1])
            cv2.rectangle(cv_image, (x1, y1_label - label_size[1]), (x1 + label_size[0], y1_label + baseline), color, cv2.FILLED)
            cv2.putText(cv_image, label, (x1, y1_label), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        # Publish the image with bounding boxes
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()