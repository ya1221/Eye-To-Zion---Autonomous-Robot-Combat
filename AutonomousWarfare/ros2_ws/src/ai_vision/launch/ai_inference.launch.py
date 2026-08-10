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
        default_value=os.path.join(pkg_share, 'config', 'ai_inference_params.yaml'),
        description='Full path to the ai_inference parameter YAML file',
    )

    ai_inference_node = Node(
        package='ai_vision',
        executable='ai_inference_node',
        name='ai_inference',
        parameters=[LaunchConfiguration('params_file')],
        # Scoped to this process so the tuning holds however ai_inference is containerized.
        additional_env={
            'OMP_NUM_THREADS': '2',
            'YOLO_OFFLINE': 'True',
            'YOLOV8_NO_TELEMETRY': '1',
        },
        output='screen',
    )

    return LaunchDescription([params_arg, ai_inference_node])
