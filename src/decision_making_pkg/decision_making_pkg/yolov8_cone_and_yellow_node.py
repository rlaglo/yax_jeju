#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

def norm_label(s: str) -> str:
    return s.strip().lower().replace('_', ' ')

class YOLOConeAndYellowNode(Node):
    def __init__(self):
        super().__init__('yolov8_cone_and_yellow_node')

        # ---------- Params ----------
        self.camera_position = self.declare_parameter('camera_position', 'left').value  # 'left' or 'right'
        self.image_topic     = self.declare_parameter('image_topic', f'/camera/{self.camera_position}/image_raw').value
        self.pub_topic       = self.declare_parameter('pub_topic',   f'/object/{self.camera_position}/detection_order').value
        self.reset_topic     = self.declare_parameter('reset_topic', f'/object/{self.camera_position}/reset').value

        # 디버그 이미지 퍼블리시 on/off
        self.visualize       = bool(self.declare_parameter('visualize', True).value)
        self.debug_topic     = self.declare_parameter('debug_image_topic', f'/debug/{self.camera_position}/image').value
        self.show_yellow_mask= bool(self.declare_parameter('overlay_yellow_mask', False).value)  # 원하면 true로

        # YOLO
        self.model_path = self.declare_parameter('model', 'best_cone.pt').value
        self.conf_thres = float(self.declare_parameter('conf_thres', 0.45).value)
        self.device     = self.declare_parameter('device', 'cuda:0').value
        cone_names_param = self.declare_parameter('cone_class_names', ['safety cone','cone','traffic cone','orange cone','blue cone']).value
        self.cone_keywords = {norm_label(x) for x in cone_names_param}

        # HSV (노란 박스)
        l1 = np.array(self.declare_parameter('yellow_hsv_low1',  [15,  90,  90]).value, dtype=np.uint8)
        h1 = np.array(self.declare_parameter('yellow_hsv_high1', [35, 255, 255]).value, dtype=np.uint8)
        l2 = np.array(self.declare_parameter('yellow_hsv_low2',  [35,  70,  70]).value, dtype=np.uint8)
        h2 = np.array(self.declare_parameter('yellow_hsv_high2', [45, 255, 255]).value, dtype=np.uint8)
        self.yellow_ranges = [(l1, h1), (l2, h2)]
        self.yellow_min_area = int(self.declare_parameter('yellow_min_area', 50000).value)

        # 퍼블리시 주기 제한
        self.publish_every_n = int(self.declare_parameter('publish_every_n', 3).value)
        self._frame_count = 0

        # ---------- IO ----------
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.sub_img   = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.sub_reset = self.create_subscription(Empty, self.reset_topic, self.on_reset, 10)
        self.pub_order = self.create_publisher(String, self.pub_topic, qos)
        self.pub_debug = self.create_publisher(Image, self.debug_topic, qos) if self.visualize else None

        self.bridge = CvBridge()
        self.detection_order = []

        # YOLO
        try:
            self.model = YOLO(self.model_path)
            self.model.fuse()
            self.get_logger().info(f"[{self.camera_position}] YOLO loaded: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"[{self.camera_position}] YOLO load error: {e}")
            raise

    def on_reset(self, _msg: Empty):
        self.get_logger().info(f"[{self.camera_position}] reset → clear detection_order")
        self.detection_order.clear()

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        found_cone, yolo_boxes = self.detect_cone_yolo(frame)
        found_yellow, yellow_contours, yellow_mask = self.detect_yellow_hsv(frame)

        # 최초 등장 순서 기록
        changed = False
        if found_cone and 'cone' not in self.detection_order:
            self.detection_order.append('cone'); changed = True
            self.get_logger().info(f"[{self.camera_position}] first seen: cone")
        if found_yellow and 'yellow' not in self.detection_order:
            self.detection_order.append('yellow'); changed = True
            self.get_logger().info(f"[{self.camera_position}] first seen: yellow")

        # 문자열 퍼블리시(레이트 제한)
        self._frame_count += 1
        if changed or (self._frame_count % self.publish_every_n == 0):
            out = String(); out.data = ",".join(self.detection_order)
            self.pub_order.publish(out)

        # 디버그 이미지 퍼블리시
        if self.visualize and self.pub_debug is not None:
            dbg = frame.copy()

            # YOLO 박스 그리기
            for (x1, y1, x2, y2, label) in yolo_boxes:
                cv2.rectangle(dbg, (x1,y1), (x2,y2), (0,255,0), 2)   # 바운딩 박스
                cv2.putText(dbg, label, (x1, max(0,y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)

            # Yellow 컨투어 그리기
            cv2.drawContours(dbg, yellow_contours, -1, (0,255,255), 2)

            # 원하면 마스크 반투명 오버레이
            if self.show_yellow_mask and yellow_mask is not None:
                overlay = dbg.copy()
                mask_col = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
                mask_col = (mask_col > 0).astype(np.uint8) * np.array([0,255,255], dtype=np.uint8)
                alpha = 0.3
                overlay = cv2.addWeighted(overlay, 1.0, mask_col, alpha, 0)
                dbg = overlay

            # detection_order 텍스트
            txt = f"order: {','.join(self.detection_order) if self.detection_order else '-'}"
            cv2.rectangle(dbg, (5,5), (5+300, 5+28), (0,0,0), -1)
            cv2.putText(dbg, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

            # 퍼블리시
            img_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            img_msg.header = msg.header  # 타임스탬프/프레임 유지
            self.pub_debug.publish(img_msg)

    # ---------- Helpers ----------
    def detect_cone_yolo(self, frame):
        boxes_draw = []
        try:
            res = self.model.predict(source=frame, conf=self.conf_thres, verbose=False, device=self.device)
            r0 = res[0].cpu()
            if r0.boxes is not None and len(r0.boxes) > 0:
                names = self.model.names
                for b in r0.boxes:
                    cls_idx = int(b.cls.item())
                    name = norm_label(names[cls_idx])
                    xyxy = b.xyxy.numpy().astype(int)[0].tolist()  # [x1,y1,x2,y2]
                    if name in self.cone_keywords:
                        boxes_draw.append((xyxy[0], xyxy[1], xyxy[2], xyxy[3], 'safety_cone'))
                return (len(boxes_draw) > 0), boxes_draw
        except Exception as e:
            self.get_logger().error(f"YOLO predict error: {e}")
        return False, boxes_draw

    def detect_yellow_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_total = None
        for low, high in self.yellow_ranges:
            mask = cv2.inRange(hsv, low, high)
            mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)
        kernel = np.ones((5,5), np.uint8)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_DILATE, kernel, iterations=1)

        cnts,_ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big_cnts = []
        found = False
        for c in cnts:
            if cv2.contourArea(c) >= self.yellow_min_area:
                big_cnts.append(c)
                found = True
        return found, big_cnts, mask_total
        

def main(args=None):
    rclpy.init(args=args)
    node = YOLOConeAndYellowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
