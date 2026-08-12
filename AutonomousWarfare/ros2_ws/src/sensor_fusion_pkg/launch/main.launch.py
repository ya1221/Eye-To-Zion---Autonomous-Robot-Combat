import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_fusion_pkg',
            executable='sensor_fusion_node', # make sure this matches the name registered in your setup.py
            name='sensor_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': os.environ.get('USE_SIM_TIME', 'false').lower() == 'true'
            }]
        )
    ])