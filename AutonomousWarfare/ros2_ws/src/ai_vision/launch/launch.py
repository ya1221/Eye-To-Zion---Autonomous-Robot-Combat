"""Bring up the whole vision pipeline (capture + inference) in one process group.

This is the entry point robot_bringup includes. The per-node
cv_processor.launch.py / ai_inference.launch.py files exist for the
split-container deployment, where each container starts only its own node.
"""
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
        # Scoped to this process rather than the container, so sharing a
        # container with nav2/control does not hand them a 2-thread OpenMP
        # pool or the ultralytics offline flags.
        additional_env={
            'OMP_NUM_THREADS': '2',
            'YOLO_OFFLINE': 'True',
            'YOLOV8_NO_TELEMETRY': '1',
        },
        output='screen',
    )

    return LaunchDescription([
        cv_params_arg,
        ai_params_arg,
        cv_processor_node,
        ai_inference_node,
    ])
