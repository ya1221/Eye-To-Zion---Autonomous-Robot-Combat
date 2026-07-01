"""
Launch file for ICM-20948 IMU driver + Madgwick orientation filter.

Pipeline:
  icm20948_driver  →  /imu/data_raw  (accel + gyro, no orientation)
                   →  /imu/mag       (magnetometer)
                          ↓
  imu_filter_madgwick  →  /imu/data  (full IMU with orientation quaternion)
                          ↓
  Your EKF (robot_localization) subscribes to /imu/data
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('imu_sensor')

    icm_params_file = os.path.join(pkg_dir, 'config', 'imu_sensor_params.yaml')
    madgwick_params_file = os.path.join(pkg_dir, 'config', 'madgwick_params.yaml')

    use_mag_arg = DeclareLaunchArgument(
        'use_mag',
        default_value='true',
        description='Enable magnetometer for absolute yaw heading'
    )

    # ─── ICM-20948 Driver Node ────────────────────────────────────────
    imu_sensor_driver_node = Node(
        package='imu_sensor',
        executable='imu_sensor_driver_node',
        name='imu_sensor_driver_node',
        output='screen',
        parameters=[icm_params_file],
        # Published topics:
        #   /imu/data_raw   (sensor_msgs/Imu, no orientation)
        #   /imu/mag        (sensor_msgs/MagneticField)
        #   /imu/temperature (sensor_msgs/Temperature, if enabled)
    )

    # ─── Madgwick Orientation Filter ──────────────────────────────────
    # Fuses accel + gyro + mag → outputs orientation quaternion
    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[madgwick_params_file],
        remappings=[
            # Input: raw IMU and magnetometer from our driver
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/mag',      '/imu/mag'),
            # Output: full IMU with orientation → consumed by EKF
            ('imu/data',     '/imu/data'),
        ],
    )
    return LaunchDescription([
        use_mag_arg,
        # imu_sensor_driver_node,
        imu_filter_madgwick_node,
    ])