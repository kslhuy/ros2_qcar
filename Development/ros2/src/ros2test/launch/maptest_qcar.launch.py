import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # package paths
    pkg_share = get_package_share_path('limo_nav_huy_test')
    pkg_description = get_package_share_path('limo_description')

    amcl_params = str(pkg_share / 'param' / 'amcl_params.yaml')
    rviz_config = str(pkg_share / 'rviz' / 'nav2_copy.rviz')
    model_path = str(pkg_description / 'urdf' / 'limo_ackerman.xacro')

    # robot_description from xacro
    robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    # IMPORTANT: Run bringup first!
    #   ros2 launch limo_bringup limo_start.launch.py
    # Then run this file to start navigation and RViz.

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/agilex/agilex_ws/bib_cran_map.yaml'}]
        ),

        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_params],
            output='screen'
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
        ),

        # Static TF: SDCQcar → map
        # This is the ONLY place where calibration (translation + rotation) between
        # the SDCSRoadMap world and the physical SLAM map is defined.
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        # Adjust yaw (radians) and x/y (meters) to align SDCSRoadMap with your physical map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sdcqcar_to_map',
            arguments=['-2.1000', '0.4000', '0', '-1.3963', '0', '0', 'SDCQcar', 'map'],
            output='screen'
        ),

        # QCar-style Waypoints (publishes raw SDCSRoadMap coords in SDCQcar frame)
        # NO calibration parameters needed - TF handles alignment
        Node(
            package='limo_nav_huy_test',
            executable='waypoints_qcar',
            name='waypoints_qcar',
            parameters=[{
                'nodeSequence': [10, 2, 4, 6, 8, 10],
            }],
            output='screen'
        ),

        # # QCar-style Vehicle Controller
        # Node(
        #     package='limo_nav_huy_test',
        #     executable='vehicle_main_ros_qcar',
        #     name='vehicle_main_ros_qcar',
        #     output='screen'
        # ),

        # Robot model publisher - NEEDED because limo_start doesn't provide it!
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
