from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ros2test'


# Function to recursively get all files in a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

# Get all files from multi_vehicle_RealCar directory
extra_files = package_files('ros2test/multi_vehicle_RealCar')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['multi_vehicle_RealCar/**/*', 'multi_vehicle_RealCar/**/**/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rviz2config'), glob('rviz2config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viet',
    maintainer_email='viet@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = ros2test.test_node:main",
            "keyboard = ros2test.olds.keyboard:main",
            "lane_follower = ros2test.lane_follower:main",
            "nav_goal_sender = ros2test.nav_goal_sender:main",
            "odom = ros2test.odom:main",
            "waypoints = ros2test.waypoints:main",
            "pure_pursuit = ros2test.pure_pursuit:main",
            "vehicle_control_ros = ros2test.vehicle_control_ros:main",
            "vehicle_control_ros_bridge = ros2test.vehicle_control_ros_bridge:main",
            "ekf = ros2test.ekf:main",
            "qcar2_bridge = ros2test.qcar2_bridge:main",
            "lidar_sub = ros2test.lidar_sub:main",
            'test_parameter_updates = ros2test.test_parameter_updates:main',
            'lidar_occupancy_node = ros2test.lidar_occupancy_node:main',
            'waypoint_alignment_helper = ros2test.waypoint_alignment_helper:main',
            'waypoints_qcar = ros2test.waypoints_qcar:main',
            'vehicle_main_ros_qcar = ros2test.multi_vehicle_RealCar.vehicle_main_ros_qcar:main',
        ],
    },
)
