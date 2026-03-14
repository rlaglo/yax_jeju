import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    cam_num = DeclareLaunchArgument(
        'cam_num',
        default_value = '6',
        description = 'color_video num for stereo cam'
    )

    cutting_edge = DeclareLaunchArgument( # action을 의미, 실제로 저 값이 만들어지는 건 LaunchConfiguration 속에서 만들어짐
        'cutting_idx',
        default_value='100',
        description='roi_image cutting edge'
    )
    image_publisher_node = Node(
        package='camera_perception_pkg',
        executable='image_publisher_node',
        name='image_publisher_node',
        parameters=[
            {'cam_num': LaunchConfiguration('cam_num')}
        ]
    )
    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen'
    )
    lane_info_extractor_node = Node(
        package='camera_perception_pkg',
        executable='lane_info_extractor_node',
        name='lane_info_extractor_node',
        output='screen',
        parameters=[
            # 튜닝값
            {'cutting_idx': LaunchConfiguration('cutting_idx')}, # 너무 먼 곳까지 봐서 조향이 불안정하면 이 값을 줄여보면됨
        ]
    )
    return LaunchDescription([
        cam_num,
        cutting_edge,
        image_publisher_node,
        yolov8_node,
        lane_info_extractor_node
    ]
    )