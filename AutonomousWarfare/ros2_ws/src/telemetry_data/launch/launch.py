import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    telegraf_bridge_node = Node(
        package='telemetry_data',
        executable='telegraf_bridge',
        name='telegraf_bridge',
        output='screen'
    )

    return LaunchDescription([
        telegraf_bridge_node
    ])
