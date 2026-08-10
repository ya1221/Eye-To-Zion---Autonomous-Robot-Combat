"""Bring up the audio impact-detection pipeline."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    audio_processor_node = Node(
        package='ai_audio',
        executable='audio_processor_node',
        name='audio_processor',
        output='screen',
    )

    return LaunchDescription([
        audio_processor_node,
    ])
