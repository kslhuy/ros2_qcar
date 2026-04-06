import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    qcar2_share = get_package_share_directory('qcar2_nodes')
    ros2test_share = get_package_share_directory('ros2test')

    cartographer_config_dir = os.path.join(qcar2_share, 'config')
    rviz_config = os.path.join(ros2test_share, 'rviz2config', 'conf.rviz')
    default_map_yaml = '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.yaml'
    default_pbstream = '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.pbstream'
    default_cartographer_config_basename = 'qcar2_2d_localization_stable.lua'

    pbstream = LaunchConfiguration('pbstream')
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    resolution = LaunchConfiguration('resolution')
    run_rviz = LaunchConfiguration('run_rviz')
    use_static_map_server = LaunchConfiguration('use_static_map_server')
    cartographer_map_topic = LaunchConfiguration('cartographer_map_topic')
    cartographer_min_log_level = LaunchConfiguration('cartographer_min_log_level')
    cartographer_config_basename = LaunchConfiguration('cartographer_config_basename')
    load_frozen_state = LaunchConfiguration('load_frozen_state')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    publish_sdc_map_tf = LaunchConfiguration('publish_sdc_map_tf')
    enable_external_waypoint_publisher = LaunchConfiguration('enable_external_waypoint_publisher')
    sdc_map_x = LaunchConfiguration('sdc_map_x')
    sdc_map_y = LaunchConfiguration('sdc_map_y')
    sdc_map_z = LaunchConfiguration('sdc_map_z')
    sdc_map_yaw = LaunchConfiguration('sdc_map_yaw')
    sdc_map_pitch = LaunchConfiguration('sdc_map_pitch')
    sdc_map_roll = LaunchConfiguration('sdc_map_roll')

    # ['-1.6000', '0.1000', '0', '-1.5708', '0', '0', 'SDCQcar', 'map'],
    #   -1.0000 -0.7000 0 \
    #   5.4716 0 0 \

#     ros2 run tf2_ros static_transform_publisher \
#   0.1000 -0.1000 0 \
#   1.5621 0 0 \
#   SDCQcar map
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

    cartographer_node_with_pbstream = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        additional_env={'GLOG_minloglevel': cartographer_min_log_level},
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
            '-load_state_filename', pbstream,
            '-load_frozen_state', load_frozen_state,
        ],
        remappings=[
            ('scan', '/scan'),
            # ('imu', '/qcar2_imu'),
        ],
        condition=IfCondition(PythonExpression(["'", pbstream, "' != ''"])),
    )

    cartographer_node_without_pbstream = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        additional_env={'GLOG_minloglevel': cartographer_min_log_level},
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            # ('imu', '/qcar2_imu'),
        ],
        condition=UnlessCondition(PythonExpression(["'", pbstream, "' != ''"])),
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        condition=UnlessCondition(use_static_map_server),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        remappings=[
            ('map', cartographer_map_topic),
        ],
    )

    # sdcqcar_to_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='sdcqcar_to_map',
    #     condition=IfCondition(publish_sdc_map_tf),
    #     arguments=[
    #         sdc_map_x,
    #         sdc_map_y,
    #         sdc_map_z,
    #         sdc_map_yaw,
    #         sdc_map_pitch,
    #         sdc_map_roll,
    #         'SDCQcar',
    #         'map',
    #     ],
    #     output='screen',
    # )

    # # QCar-style Waypoints
    # waypoints_qcar = Node(
    #     package='ros2test',
    #     executable='waypoints_qcar',
    #     name='waypoints_qcar',
    #     condition=IfCondition(enable_external_waypoint_publisher),
    #     parameters=[{
    #         'nodeSequence': [10, 2, 4, 6, 8, 10],
    #     }],
    #     output='screen'
    # )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        condition=IfCondition(use_static_map_server),
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        condition=IfCondition(use_static_map_server),
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
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
            'pbstream',
            default_value=default_pbstream,
            description='Path to Cartographer .pbstream map to load (set empty string to disable loading)',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Path to map YAML file for map_server',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Cartographer occupancy grid resolution',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Cartographer occupancy grid publish period',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Launch RViz if true',
        ),
        DeclareLaunchArgument(
            'use_static_map_server',
            default_value='true',
            description='If true, start nav2 map_server from YAML. Default is true for saved-map localization stability; set false to use Cartographer as /map source.',
        ),
        DeclareLaunchArgument(
            'cartographer_map_topic',
            default_value='/map',
            description='Topic name for Cartographer occupancy grid output (use /map for Nav2).',
        ),
        DeclareLaunchArgument(
            'cartographer_min_log_level',
            default_value='1',
            description='Cartographer glog level: 0=INFO, 1=WARNING, 2=ERROR, 3=FATAL',
        ),
        DeclareLaunchArgument(
            'cartographer_config_basename',
            default_value=default_cartographer_config_basename,
            description='Cartographer Lua config basename. Default is the stability-first qcar2_2d_localization_stable.lua; use qcar2_2d_localization.lua to try pure localization, or qcar2_2d.lua for mapping.',
        ),
        DeclareLaunchArgument(
            'load_frozen_state',
            default_value='true',
            description='Keep loaded .pbstream trajectory frozen so Cartographer does not deform the saved map.',
        ),
        DeclareLaunchArgument(
            'rviz_log_level',
            default_value='warn',
            description='RViz ROS log level (debug, info, warn, error, fatal)',
        ),
        DeclareLaunchArgument(
            'publish_sdc_map_tf',
            default_value='false',
            description='Fallback static SDCQcar->map TF publisher. Keep false when vehicle system publishes runtime SDCQcar->map TF.',
        ),
        DeclareLaunchArgument(
            'enable_external_waypoint_publisher',
            default_value='false',
            description='Start external waypoints_qcar publisher. Keep false to use internal path generation in vehicle system.',
        ),
        DeclareLaunchArgument(
            'sdc_map_x',
            default_value='0.1000',
            description='Static TF translation x for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_y',
            default_value='-0.1000',
            description='Static TF translation y for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_z',
            default_value='0',
            description='Static TF translation z for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_yaw',
            default_value='1.5621',
            description='Static TF yaw (rad) for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_pitch',
            default_value='0',
            description='Static TF pitch (rad) for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_roll',
            default_value='0',
            description='Static TF roll (rad) for SDCQcar -> map',
        ),
        qcar2_launch,
        fixed_lidar_frame,
        cartographer_node_with_pbstream,
        cartographer_node_without_pbstream,
        cartographer_occupancy_grid_node,
        # sdcqcar_to_map,
        map_server_node,
        lifecycle_manager,
        # waypoints_qcar,
        rviz_node,
    ])
