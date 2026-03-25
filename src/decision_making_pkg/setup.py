from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'decision_making_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line finds the 'waypoints' directory and includes all files in it
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhs',
    maintainer_email='hhs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = decision_making_pkg.path_planner_node:main',
            'motion_planner_node = decision_making_pkg.motion_planner_node:main',
            'waypoint_follower_node = decision_making_pkg.waypoint_follower_node:main', # Add this line
            'path_planner_robust_node = decision_making_pkg.path_planner_robust_node:main',
            'cmd_vel_to_motion_command = decision_making_pkg.cmd_vel_to_motion_command:main', # Add this line
            'forward_driver_node = decision_making_pkg.forward_driver_node:main',
            'gps_goto_node = decision_making_pkg.gps_goto_node:main',
            'fake_gps_publisher = decision_making_pkg.fake_gps_publisher:main',
            'yolov8_cone_and_yellow_node = decision_making_pkg.yolov8_cone_and_yellow_node:main',
        ],
    },
)