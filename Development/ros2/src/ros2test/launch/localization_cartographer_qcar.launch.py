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
    default_map_yaml = os.path.join(ros2test_share, 'map', 'bib_cran.yaml')

    pbstream = LaunchConfiguration('pbstream')
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    resolution = LaunchConfiguration('resolution')
    run_rviz = LaunchConfiguration('run_rviz')
    sdc_map_x = LaunchConfiguration('sdc_map_x')
    sdc_map_y = LaunchConfiguration('sdc_map_y')
    sdc_map_z = LaunchConfiguration('sdc_map_z')
    sdc_map_yaw = LaunchConfiguration('sdc_map_yaw')
    sdc_map_pitch = LaunchConfiguration('sdc_map_pitch')
    sdc_map_roll = LaunchConfiguration('sdc_map_roll')

    # ['-1.6000', '0.1000', '0', '-1.5708', '0', '0', 'SDCQcar', 'map'],

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
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'qcar2_2d.lua',
            '-load_state_filename', pbstream,
        ],
        remappings=[
            ('scan', '/scan'),
        ],
        condition=IfCondition(PythonExpression(["'", pbstream, "' != ''"])),
    )

    cartographer_node_without_pbstream = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'qcar2_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),
        ],
        condition=UnlessCondition(PythonExpression(["'", pbstream, "' != ''"])),
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        remappings=[
            ('map', '/cartographer_map'),
        ],
    )

    sdcqcar_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sdcqcar_to_map',
        arguments=[
            sdc_map_x,
            sdc_map_y,
            sdc_map_z,
            sdc_map_yaw,
            sdc_map_pitch,
            sdc_map_roll,
            'SDCQcar',
            'map',
        ],
        output='screen',
    )

    # QCar-style Waypoints
    waypoints_qcar = Node(
        package='limo_nav_huy_test',
        executable='waypoints_qcar',
        name='waypoints_qcar',
        parameters=[{
            'nodeSequence': [10, 2, 4, 6, 8, 10],
        }],
        output='screen'
    ),

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
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
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pbstream',
            default_value='',
            description='Path to Cartographer .pbstream map to load (empty string disables loading)',
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
            'sdc_map_x',
            default_value='-1.6000',
            description='Static TF translation x for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_y',
            default_value='0.1000',
            description='Static TF translation y for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_z',
            default_value='0',
            description='Static TF translation z for SDCQcar -> map',
        ),
        DeclareLaunchArgument(
            'sdc_map_yaw',
            default_value='-1.5708',
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
        sdcqcar_to_map,
        map_server_node,
        lifecycle_manager,
        waypoints_qcar,
        rviz_node,
    ])
