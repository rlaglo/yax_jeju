from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            parameters=[{'data_source': 'camera', 'cam_num': 4}],
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
            executable='lane_extract',
            name='lane_info_extractor_node',
            output='screen',
            parameters=[
                # 캘리브레이션으로 찾은 값
                {'perspective.src_points': [173, 132, 422, 134, 632, 241, 5, 225]},

                # 튜닝값
                {'roi.cutting_idx': 300}, # 너무 먼 곳까지 봐서 조향이 불안정하면 이 값을 줄여보면됨
                {'lane.width': 350}      # 버드아이뷰에서 보이는 차선 폭(pixel)에 맞게 조절
            ]
        ),

        # 경로/모션
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen'
        ),

        # ★ 시리얼 송신 (실차 구동에 필수)
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            # 패키지가 파라미터를 받는 경우에만 사용. 안 받으면 소스에서 상수 변경 필요(아래 3-2 참고)
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baudrate': 115200}],
            output='screen'
        ),
    ])