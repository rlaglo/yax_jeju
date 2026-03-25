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

    # 카메라 인식 관련 노드
    image_publisher_node = Node(
        package='camera_perception_pkg',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen'
    )

    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen'
    )
    
    traffic_light_detector_node = Node(
        package='camera_perception_pkg',
        executable='traffic_light_detector_node',
        name='traffic_light_detector_node',
        output='screen'
    )

    # 제어 및 통신 관련 노드
    serial_sender_node = Node(
        package='serial_communication_pkg',
        executable='serial_sender_node',
        name='serial_sender_node',
        output='screen'
    )
    
    gps_goto_node = Node(
        package='decision_making_pkg',
        executable='gps_goto_node',
        name='gps_goto_node',
        output='screen',
        parameters=[params_file_path] # 파라미터 파일 지정
    )

    return LaunchDescription([
        # GPS 관련
        ublox_gps_launch,
        ntrip_client_launch,
        fix2nmea_node,
        
        # 카메라 인식 관련
        image_publisher_node,
        yolov8_node,
        traffic_light_detector_node,

        # 제어 및 통신 관련
        gps_goto_node,
        serial_sender_node
    ])
