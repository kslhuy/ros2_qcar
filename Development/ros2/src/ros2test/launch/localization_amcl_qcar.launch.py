import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    qcar2_share = get_package_share_directory('qcar2_nodes')
    ros2test_share = get_package_share_directory('ros2test')

    default_map_yaml = '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.yaml'
    default_params_file = os.path.join(qcar2_share, 'config', 'qcar2_slam_and_nav.yaml')
    rviz_config = os.path.join(ros2test_share, 'rviz2config', 'conf.rviz')

    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('run_rviz')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    localization_start_delay_sec = LaunchConfiguration('localization_start_delay_sec')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw_deg = LaunchConfiguration('initial_pose_yaw_deg')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        },
        convert_types=True,
    )

    qcar2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qcar2_share, 'launch', 'qcar2_launch.py')
        )
    )

    fixed_lidar_frame = Node(
        package='qcar2_nodes',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
    )

    dead_reckoning_odom = Node(
        package='ros2test',
        executable='qcar_dead_reckoning_odom',
        name='qcar_dead_reckoning_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    delayed_localization_stack = TimerAction(
        period=localization_start_delay_sec,
        actions=[
            map_server_node,
            amcl_node,
            lifecycle_manager,
        ],
    )

    initial_pose_helper = Node(
        condition=IfCondition(publish_initial_pose),
        package='ros2test',
        executable='amcl_initial_pose_publisher',
        name='amcl_initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'x': initial_pose_x,
            'y': initial_pose_y,
            'yaw_deg': initial_pose_yaw_deg,
        }],
    )

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', rviz_log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Path to map YAML file for saved-map localization',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file for map_server and AMCL',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Launch RViz if true',
        ),
        DeclareLaunchArgument(
            'localization_start_delay_sec',
            default_value='2.0',
            description='Delay AMCL and map_server startup to let TF and sensor topics warm up.',
        ),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='false',
            description='If true, publish an initial pose to AMCL after it subscribes to /initialpose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x in the map frame.',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y in the map frame.',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw_deg',
            default_value='0.0',
            description='Initial pose yaw in degrees in the map frame.',
        ),
        DeclareLaunchArgument(
            'rviz_log_level',
            default_value='warn',
            description='RViz ROS log level (debug, info, warn, error, fatal)',
        ),
        qcar2_launch,
        fixed_lidar_frame,
        dead_reckoning_odom,
        delayed_localization_stack,
        initial_pose_helper,
        rviz_node,
    ])
