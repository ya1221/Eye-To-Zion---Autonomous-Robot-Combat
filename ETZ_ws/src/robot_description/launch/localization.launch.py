import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')

    ekf_config_path  = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(pkg_dir, 'config', 'slam.yaml')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': True}
        ]
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_tf': False,      
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
            'odom_topic': '/rf2o/odom',
            'laser_scan_topic': '/scan' 
        }]
        # שורת ה-arguments שהתנגשה עם העברת הפרמטרים נמחקה
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path, 
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        ekf,
        TimerAction(period=5.0, actions=[rf2o]),
        slam,
    ])