import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    default_ekf_params_file = os.path.join(ros2test_share, 'config', 'qcar_amcl_ekf.yaml')
    default_rf2o_params_file = os.path.join(ros2test_share, 'config', 'qcar_rf2o.yaml')
    rviz_config = os.path.join(ros2test_share, 'rviz2config', 'conf.rviz')

    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rf2o_params_file = LaunchConfiguration('rf2o_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('run_rviz')
    rviz_log_level = LaunchConfiguration('rviz_log_level')

    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        },
        convert_types=True,
    )

    configured_ekf_params = RewrittenYaml(
        source_file=ekf_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
        },
        convert_types=True,
    )

    configured_rf2o_params = RewrittenYaml(
        source_file=rf2o_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
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

    wheel_imu_odom = Node(
        package='ros2test',
        executable='qcar_dead_reckoning_odom',
        name='qcar_dead_reckoning_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'odom_topic': '/odom/wheel',
            'publish_tf': False,
            'publish_rate': 80.0,
        }],
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[configured_rf2o_params],
    )

    ekf_node = Node(
        package='ros2test',
        executable='qcar_ekf_odom_fusion',
        name='ekf_filter_node',
        output='screen',
        parameters=[configured_ekf_params],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_nav2_params],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_nav2_params],
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
            'ekf_params_file',
            default_value=default_ekf_params_file,
            description='Full path to the EKF parameters file',
        ),
        DeclareLaunchArgument(
            'rf2o_params_file',
            default_value=default_rf2o_params_file,
            description='Full path to the RF2O parameters file',
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
            'rviz_log_level',
            default_value='warn',
            description='RViz ROS log level (debug, info, warn, error, fatal)',
        ),
        qcar2_launch,
        fixed_lidar_frame,
        wheel_imu_odom,
        rf2o_node,
        ekf_node,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        rviz_node,
    ])
