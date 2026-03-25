import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the robot_localization configuration file
    config_dir = os.path.join(get_package_share_directory('ublox_dgnss'), 'launch')

    # Note: You may need to create this config file.
    # For now, we are passing parameters directly.

    return LaunchDescription([
        # Start the navsat_transform_node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'use_odometry_yaw': False}, # Set to true if you don't have an IMU
                {'wait_for_datum': False},
                {'yaw_offset': 1.5707963}, # Adjust this based on your IMU's orientation. 0 is East.
                {'publish_filtered_gps': True}
            ],
            remappings=[
                ('/gps/fix', '/fix'), # Remap to your NavSatFix topic
                ('/imu', '/your/imu/topic'), # Remap to your IMU topic
                ('/odometry/filtered', '/odometry/gps') # Output odometry topic
            ]
        ),

        # Start RViz with a configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('ublox_dgnss'), 'launch', 'navsat_rviz_config.rviz')]
        )
    ])