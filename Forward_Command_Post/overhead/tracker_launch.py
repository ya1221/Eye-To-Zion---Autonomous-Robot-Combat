import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cnt_team', default_value='2', description='Number of teams'),
        DeclareLaunchArgument('camera_id', default_value='0', description='Camera device ID'),
        DeclareLaunchArgument('grid_n', default_value='2000', description='Grid coordinate resolution'),
        DeclareLaunchArgument('targets_start', default_value='4', description='Targets starting ArUco ID'),

        ExecuteProcess(
            cmd=[
                'python3', '/app/overhead_tracker.py',
                '--ros-args',
                '-p', ['cnt_team:=', LaunchConfiguration('cnt_team')],
                '-p', ['camera_id:=', LaunchConfiguration('camera_id')],
                '-p', ['grid_n:=', LaunchConfiguration('grid_n')],
                '-p', ['targets_start:=', LaunchConfiguration('targets_start')]
            ],
            output='screen'
        )
    ])
