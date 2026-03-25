#!/usr/bin/env python3
"""
my_custom_pkg 전체 노드 런치 파일.

실행 방법:
  ros2 launch my_custom_pkg mission_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_custom_pkg')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # ── 1. Waypoint Follower (GPS) ──
        Node(
            package='my_custom_pkg',
            executable='waypoint_follower_node',
            name='waypoint_follower_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── 2. Lane Detection ──
        Node(
            package='my_custom_pkg',
            executable='lane_detection_node',
            name='lane_detection_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── 3. Tunnel Navigation ──
        Node(
            package='my_custom_pkg',
            executable='tunnel_nav_node',
            name='tunnel_nav_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── 4. Obstacle Detection ──
        Node(
            package='my_custom_pkg',
            executable='obstacle_detect_node',
            name='obstacle_detect_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── 5. Mission Controller (최종 결정) ──
        Node(
            package='my_custom_pkg',
            executable='mission_controller_node',
            name='mission_controller_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
