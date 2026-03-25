from setuptools import find_packages, setup

package_name = 'debug_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_visualizer_node = debug_pkg.yolov8_visualizer_node:main',
            'path_visualizer_node = debug_pkg.path_visualizer_node:main',
            'traffic_light_visualizer_node = debug_pkg.traffic_light_visualizer_node:main', # Added new node
            'steering_visualizer_node = debug_pkg.steering_visualizer_node:main',

        ],
    },
)