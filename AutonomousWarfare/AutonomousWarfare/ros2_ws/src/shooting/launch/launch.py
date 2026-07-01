from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  

    shooting_node = Node(
        package="shooting",
        executable='shooting_node',
        name='shooting_node',
        output='screen',
    )
    return LaunchDescription([
        shooting_node
    ])
