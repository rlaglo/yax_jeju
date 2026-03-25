import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import LaneInfo, PathPlanningResult, TargetPoint

import numpy as np

# --------------- ADDED: Tunable Parameters for Robustness ---------------
# Desired distance (in pixels) to offset the path from a single detected lane.
LANE_OFFSET = 40  # Tune this value as needed

# Window size for the moving average filter. A larger window creates a
# smoother path but adds a slight delay.
SMOOTHING_WINDOW_SIZE = 5
# -------------------------------------------------------------------------

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.lane_info_sub = self.create_subscription(LaneInfo, 'lane_info', self.lane_info_callback, self.qos_profile)
        self.path_pub = self.create_publisher(PathPlanningResult, 'path_planning_result', self.qos_profile)

        self.L_idx = -1
        self.R_idx = -1
        self.M_idx = -1

    # --------------- ADDED: Path Smoothing Function ---------------
    def smooth_path(self, path_points):
        """
        Smoothes the path's x-coordinates using a simple moving average filter.
        """
        if not path_points or len(path_points) < SMOOTHING_WINDOW_SIZE:
            return path_points

        # Use a copy of the x-coordinates for smoothing
        x_coords = [p.x for p in path_points]
        
        # Apply convolution for a moving average
        kernel = np.ones(SMOOTHING_WINDOW_SIZE) / SMOOTHING_WINDOW_SIZE
        smoothed_x = np.convolve(x_coords, kernel, mode='same')
        
        # Create new points with smoothed x-values
        smoothed_points = []
        for i, point in enumerate(path_points):
            new_point = TargetPoint()
            new_point.x = smoothed_x[i]
            new_point.y = point.y # Keep original y
            smoothed_points.append(new_point)
            
        # The convolution at the edges can be less accurate, so we replace
        # the start of the path with the original points to maintain responsiveness.
        for i in range(SMOOTHING_WINDOW_SIZE // 2):
            smoothed_points[i] = path_points[i]

        return smoothed_points
    # ---------------------------------------------------------------

    def lane_info_callback(self, lane_info: LaneInfo):
        path_msg = PathPlanningResult()
        path_points = []
        
        self.L_idx = -1
        self.R_idx = -1
        self.M_idx = -1

        for i in range(len(lane_info.lane_labels)):
            if lane_info.lane_labels[i] == 'left':
                self.L_idx = i
            elif lane_info.lane_labels[i] == 'right':
                self.R_idx = i
            elif lane_info.lane_labels[i] == 'middle':
                self.M_idx = i

        i = 0
        while i < len(lane_info.lane_points):
            
            p = TargetPoint()
            
            # Case 1: Both Left and Right lanes are detected (Ideal case)
            if self.L_idx != -1 and self.R_idx != -1:
                p.x = (lane_info.lane_points[i + self.L_idx].x + lane_info.lane_points[i + self.R_idx].x) / 2
                p.y = lane_info.lane_points[i].y
                path_points.append(p)

            # --------------- MODIFIED: Robust Single Lane Following ---------------
            # Case 2: Only Left lane is detected
            elif self.L_idx != -1 and self.R_idx == -1:
                # Create path with a fixed offset to the right of the left lane
                p.x = lane_info.lane_points[i].x + LANE_OFFSET
                p.y = lane_info.lane_points[i].y
                path_points.append(p)

            # Case 3: Only Right lane is detected
            elif self.L_idx == -1 and self.R_idx != -1:
                # Create path with a fixed offset to the left of the right lane
                p.x = lane_info.lane_points[i].x - LANE_OFFSET
                p.y = lane_info.lane_points[i].y
                path_points.append(p)
            # --------------------------------------------------------------------
                
            # Case 4: No lanes detected
            else:
                # If no lanes are detected, the path will be empty.
                # Consider adding logic to maintain the last known heading or stop.
                pass

            i += 1
        
        # --------------- ADDED: Apply Path Smoothing ---------------
        # Apply the smoothing function to the generated path points before publishing
        if path_points:
            smoothed_path = self.smooth_path(path_points)
            path_msg.path_points = smoothed_path
        else:
            path_msg.path_points = path_points
        # -----------------------------------------------------------

        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()