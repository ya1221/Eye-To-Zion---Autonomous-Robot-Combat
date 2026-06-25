import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Real deployment graph: sensor_fusion_pkg + tactical_brain, unmodified
    # from what will run on the RPi5.
    bringup_pkg = FindPackageShare('etz_bringup')
    launch_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg.find('etz_bringup'), 'launch', 'main.launch.py')
        )
    )

    # Gazebo-only: stands in for the real EKF+ArUco pose publisher (see
    # tactical_brain/mock_pose_publisher.py). Never add this to
    # main.launch.py - it's not part of the RPi5 deployment graph.
    mock_pose_publisher = Node(
        package='tactical_brain',
        executable='mock_pose_publisher',
        name='mock_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        launch_main,
        mock_pose_publisher,
    ])
