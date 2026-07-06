import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    ydlidar_config_path = os.path.join(pkg_share, 'config', 'lidar.yaml')
    slam_config_path = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    rf2o_config = os.path.join(pkg_share, 'config', 'rf2o.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_config_path, {'use_sim_time': False}]  
    )

    # ldlidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld06.launch.py')
    #     )
    # )

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
            name='ekf_local',
            parameters=[ekf_local_config],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
            ],
    )
 
    # ---- EKF Global: map → odom (ArUco corrected) ----
    ekf_global_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[ekf_global_config],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
            ],
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        parameters=[rf2o_config],
        output='screen',
    )


    # ---- Position Listener: ArUco → /aruco/odom (via Zenoh) ----
    position_listener_node = Node(
        package='position_listener',
        executable='position_listener_node',
        name='position_listener_node',
        parameters=[{
            'team_index': 0,
            'grid_to_meters_scale': 1.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        ydlidar_node,
        # ldlidar_launch,
        slam_toolbox_node,
        ekf_local_node,
        ekf_global_node,
        position_listener_node,
        rf2o_node,
    ])