import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_prefix = get_package_share_directory('test_cartographer')
    configuration_dir = LaunchConfiguration('configuration_dir',default=os.path.join(cartographer_prefix, 'lua'))
    configuration_basename = LaunchConfiguration('configuration_basename',default='cartographer_2lidar.lua')
    rviz_config_dir = os.path.join(get_package_share_directory('test_cartographer'), 'rviz', 'cartographer.rviz')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    resolution = LaunchConfiguration('resolution', default='0.02')

    return LaunchDescription([

        DeclareLaunchArgument(
            'configuration_dir',
            default_value=configuration_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', configuration_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[
                ('/imu', '/imu/data')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("test_world"), 'param', 'ekf.yaml')],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'),
    ]) 
