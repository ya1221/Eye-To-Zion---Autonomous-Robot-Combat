from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactical_brain',
            executable='brain_node', # ודא שזה השם שרשום ב-setup.py שלך
            name='tactical_brain_node',
            output='screen'
        )
    ])