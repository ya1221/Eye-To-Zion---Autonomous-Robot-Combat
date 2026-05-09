import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the YAML file path
    # Make sure 'your_package_name' matches your actual package
    ydlidar_config_path = os.path.join(
        get_package_share_directory('localization'), 'config', 'lidar.yaml'
    )

    slam_config_path = os.path.join(
        get_package_share_directory('localization'), 'config', 'slam.yaml'
    )

    # 2. Define the Node with the parameter file
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_config_path, {'use_sim_time': False}]  
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path, {'use_sim_time': False}] 
    )

    

    return LaunchDescription([
        ydlidar_node,
        slam_toolbox_node,
    ])