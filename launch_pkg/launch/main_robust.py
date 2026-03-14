from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_traffic_light_node',
            name='yolov8_traffic_light_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            output='screen'
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            name='yolov8_visualizer_node',
            output='screen'
        ),        
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_detector_node',
            name='traffic_light_detector_node',
            output='screen'
        ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_publisher_node',
        #     name='lidar_publisher_node',
        #     output='screen'
        # ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_processor_node',
        #     name='lidar_processor_node',
        #     output='screen'
        # ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_obstacle_detector_node',
        #     name='lidar_obstacle_detector_node',
        #     output='screen'
        # ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            # --- This is the only change ---
            executable='path_planner_robust_node',
            name='path_planner_node',
            output='screen'
        ),
        # Node(
        #     package='decision_making_pkg',
        #     executable='waypoint_follower_node',
        #     name='waypoint_follower_node',
        #     output='screen',
        #     parameters=[{'waypoints_file': 'src/decision_making_pkg/waypoints.csv'}]
        # ),  
        Node(
            package='debug_pkg',
            executable='path_visualizer_node',
            name='path_visualizer_node',
            output='screen'
        ),
        Node(
            package='debug_pkg',
            executable='traffic_light_visualizer_node',
            name='traffic_light_visualizer_node',
            output='screen'
        ),
            Node(
            package='debug_pkg',
            executable='steering_visualizer_node',
            name='steering_visualizer_node',
            output='screen'
        ),
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen'
        ),
    ])