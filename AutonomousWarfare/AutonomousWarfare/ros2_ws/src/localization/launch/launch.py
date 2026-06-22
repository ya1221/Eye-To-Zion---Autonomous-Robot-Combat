import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    ydlidar_config_path = os.path.join(pkg_share, 'config', 'lidar.yaml')
    slam_config_path = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

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

    # ---- EKF Local: odom → base_link (smooth, no jumps) ----
    ekf_local_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
            ],
            output='screen',
    )
 
    # ---- EKF Global: map → odom (ArUco corrected) ----
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        output='screen',
    )

    return LaunchDescription([
        ydlidar_node,
        slam_toolbox_node,
        ekf_local_node,
        ekf_global_node
    ])