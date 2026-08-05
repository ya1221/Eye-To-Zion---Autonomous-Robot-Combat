import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('robot_stats'),
        'config',
        'robot_stats_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the robot_stats parameter file'
    )

    robot_stats_node = Node(
        package='robot_stats',
        executable='robot_stats_node',
        name='robot_stats',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        robot_stats_node
    ])
