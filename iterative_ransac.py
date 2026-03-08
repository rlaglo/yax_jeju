import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray  # 시각화를 위한 라이브러리 추가
import numpy as np

class LidarLaneNode(Node):
    def __init__(self):
        super().__init__('lidar_lane_node')
        # 토픽 구독 및 발행
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.target_pub = self.create_publisher(Point, '/target_point', 10)
        
        # RViz2 시각화를 위한 마커 배열 퍼블리셔 추가
        self.marker_pub = self.create_publisher(MarkerArray, '/perception_markers', 10)

    def scan_callback(self, msg):
        frame_id = msg.header.frame_id  # RViz2에서 띄우기 위해 Lidar의 프레임 ID 저장
        
        # 1. 스캔 데이터를 (x, y) 좌표로 변환
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        mask = (ranges > 0.1) & (ranges < 5.0)
        x = ranges[mask] * np.cos(angles[mask])
        y = ranges[mask] * np.sin(angles[mask])
        points = np.column_stack((x, y))

        if len(points) < 10: return

        # 2. Sequential RANSAC 실행
        line1_inliers, remaining_points = self.run_ransac(points)
        line2_inliers, _ = self.run_ransac(remaining_points)

        # 시각화 마커 배열 초기화
        marker_array = MarkerArray()

        # 3. 중앙 타겟 포인트 계산 및 시각화
        if line1_inliers is not None and line2_inliers is not None:
            center1 = np.mean(line1_inliers, axis=0)
            center2 = np.mean(line2_inliers, axis=0)
            target = (center1 + center2) / 2
            
            # 타겟 포인트 발행
            target_msg = Point(x=target[0], y=target[1], z=0.0)
            self.target_pub.publish(target_msg)
            
            # 타겟 마커 생성 (초록색 구)
            target_marker = self.create_marker(frame_id, 0, Marker.SPHERE, target, [0.0, 1.0, 0.0, 1.0], scale=0.15)
            marker_array.markers.append(target_marker)

        # 4. 검출된 벽면(Inliers) 시각화
        if line1_inliers is not None:
            # 첫 번째 벽: 빨간색 점
            m1 = self.create_points_marker(frame_id, 1, line1_inliers, [1.0, 0.0, 0.0, 1.0])
            marker_array.markers.append(m1)
            
        if line2_inliers is not None:
            # 두 번째 벽: 파란색 점
            m2 = self.create_points_marker(frame_id, 2, line2_inliers, [0.0, 0.5, 1.0, 1.0])
            marker_array.markers.append(m2)

        # 마커 배열 발행
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    # 헬퍼 함수 1: 단일 타겟 마커 생성
    def create_marker(self, frame_id, marker_id, marker_type, pos, color, scale=0.1):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "perception_target"
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]
        return m

    # 헬퍼 함수 2: RANSAC Inlier 점 군집 마커 생성
    def create_points_marker(self, frame_id, marker_id, points, color):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "perception_walls"
        m.id = marker_id
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03  # 점의 가로 크기
        m.scale.y = 0.03  # 점의 세로 크기
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]
        
        for p in points:
            pt = Point(x=p[0], y=p[1], z=0.0)
            m.points.append(pt)
        return m

    def run_ransac(self, pts, iterations=50, threshold=0.05):
        if len(pts) < 10: 
            return None, pts
            
        best_inliers = None
        max_inlier_count = 0
        best_mask = np.zeros(len(pts), dtype=bool)

        for _ in range(iterations):
            idx = np.random.choice(pts.shape[0], 2, replace=False)
            p1, p2 = pts[idx]

            a = p1[1] - p2[1]
            b = p2[0] - p1[0]
            c = p1[0] * p2[1] - p2[0] * p1[1]

            norm = np.sqrt(a**2 + b**2)
            if norm < 1e-6: continue
            
            distances = np.abs(a * pts[:, 0] + b * pts[:, 1] + c) / norm
            current_mask = distances < threshold
            inlier_count = np.sum(current_mask)

            if inlier_count > max_inlier_count:
                max_inlier_count = inlier_count
                best_mask = current_mask

        if max_inlier_count > 0:
            best_inliers = pts[best_mask]
            remaining_pts = pts[~best_mask]
            return best_inliers, remaining_pts
        else:
            return None, pts

    def destroy_node(self):
        self.subscription.destroy()
        self.target_pub.destroy()
        self.marker_pub.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()