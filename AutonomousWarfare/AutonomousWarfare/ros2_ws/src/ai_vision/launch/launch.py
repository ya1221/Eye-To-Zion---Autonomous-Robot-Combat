import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ai_vision')

    cv_params_arg = DeclareLaunchArgument(
        'cv_params_file',
        default_value=os.path.join(pkg_share, 'config', 'cv_processor_params.yaml'),
        description='Full path to the cv_processor parameter YAML file',
    )
    ai_params_arg = DeclareLaunchArgument(
        'ai_params_file',
        default_value=os.path.join(pkg_share, 'config', 'ai_inference_params.yaml'),
        description='Full path to the ai_inference parameter YAML file',
    )

    cv_processor_node = Node(
        package='ai_vision',
        executable='cv_processor_node',
        name='cv_processor',
        parameters=[LaunchConfiguration('cv_params_file')],
        output='screen',
    )

    ai_inference_node = Node(
        package='ai_vision',
        executable='ai_inference_node',
        name='ai_inference',
        parameters=[LaunchConfiguration('ai_params_file')],
        output='screen',
    )

    return LaunchDescription([
        cv_params_arg,
        ai_params_arg,
        cv_processor_node,
        ai_inference_node,
    ])