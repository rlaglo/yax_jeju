from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Update this list to match your camera numbers
    cam_numbers = [0, 2] 
    
    nodes = []
    
    for i in cam_numbers:
        nodes.extend([
            Node(
                package='camera_perception_pkg',
                executable='image_publisher_node',
                name=f'image_publisher_node_{i}',
                output='screen',
                parameters=[
                    {'data_source': 'camera'},
                    {'cam_num': i}, 
                    {'pub_topic': f'image_raw_{i}'}
                ]
            ),
            Node(
                package='camera_perception_pkg',
                executable='yolov8_node',
                name=f'yolov8_node_{i}',
                output='screen',
                remappings=[
                    ('image_raw', f'image_raw_{i}'),
                    ('detections', f'detections_{i}')
                ]
            ),
            Node(
                package='camera_perception_pkg',
                executable='lane_info_extractor_node',
                name=f'lane_info_extractor_node_{i}',
                output='screen',
                parameters=[
                    # Corrected parameter name from 'sub_topic' to 'sub_detection_topic'
                    {'sub_detection_topic': f'detections_{i}'},
                    # Pass the unique topic name for the ROI image
                    {'roi_pub_topic': f'roi_image_{i}'}
                ]
            ),
            Node(
                package='debug_pkg',
                executable='yolov8_visualizer_node',
                name=f'yolov8_visualizer_node_{i}',
                output='screen',
                remappings=[
                    ('image_raw', f'image_raw_{i}'),
                    ('detections', f'detections_{i}'),
                    ('yolov8_visualized_img', f'yolov8_visualized_img_{i}')
                ]
            )
        ])
        
    # Add the new ROI merger node to the launch description
    nodes.append(
        Node(
            package='camera_perception_pkg',
            executable='roi_merger_node',
            name='roi_merger_node',
            output='screen'
        )
    )

    return LaunchDescription(nodes)