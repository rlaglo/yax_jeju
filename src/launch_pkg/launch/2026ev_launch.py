import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    # --- 각 패키지의 공유 디렉터리 경로 설정 ---
    ublox_gps_pkg = get_package_share_directory('ublox_gps')
    ntrip_client_pkg = get_package_share_directory('ntrip_client') # 패키지 이름이 ntrip_client_ros일 수 있음
    decision_making_pkg = get_package_share_directory('decision_making_pkg')
    fix2nmea_pkg = get_package_share_directory('fix2nmea')
    serial_communication_pkg = get_package_share_directory('serial_communication_pkg')
    camera_perception_pkg = get_package_share_directory('camera_perception_pkg')
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    my_custom_pkg = get_package_share_directory('my_custom_pkg')

    # gps_goto_node가 사용할 파라미터 파일 경로 설정
    params_file_path = os.path.join(decision_making_pkg, 'config', 'params.yaml')

    # --- 외부 패키지 Launch 파일 및 노드 실행 ---
    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_gps_pkg, 'launch', 'ublox_gps_node-launch.py')
        )
    )
    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntrip_client_pkg, 'ntrip_client_launch.py')
        )
    )
    fix2nmea_node = Node(
        package='fix2nmea', # 스크린샷 기준 패키지 이름
        executable='fix2nmea',
        name='fix2nmea_node'
    )
    serial_sender_node = Node(
        package = 'serial_communication_pkg',
        executable = 'serial_sender_node',
        name = 'serial_sender_node',
        output='screen'
    )

    camera_lane_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_perception_pkg, 'launch','camera_lane.launch.py')
        )
    )

    path_planner_node = Node(
        package='decision_making_pkg',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg,'launch','ydlidar_launch_view.py')
        )
    )
    js_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_custom_pkg,'launch','mission_launch.py')
        )
    )

    return LaunchDescription([
        ublox_gps_launch,
        ntrip_client_launch,
        fix2nmea_node,
        serial_sender_node,
        camera_lane_launch,
        path_planner_node,
        ydlidar_launch,
        js_launch
    ])