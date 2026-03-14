import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_custom_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsmoon',
    maintainer_email='2021145074@yonsei.ac.kr',
    description='GPS waypoint follower, lane/tunnel/obstacle nodes and mission controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_follower_node = my_custom_pkg.waypoint_follower_node:main',
            'lane_detection_node = my_custom_pkg.lane_detection_node:main',
            'tunnel_nav_node = my_custom_pkg.tunnel_nav_node:main',
            'obstacle_detect_node = my_custom_pkg.obstacle_detect_node:main',
            'mission_controller_node = my_custom_pkg.mission_controller_node:main',
        ],
    },
)
