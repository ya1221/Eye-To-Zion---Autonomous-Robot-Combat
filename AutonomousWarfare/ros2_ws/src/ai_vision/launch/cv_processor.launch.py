import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ai_vision')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'cv_processor_params.yaml'),
        description='Full path to the cv_processor parameter YAML file',
    )

    cv_processor_node = Node(
        package='ai_vision',
        executable='cv_processor_node',
        name='cv_processor',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([params_arg, cv_processor_node])
