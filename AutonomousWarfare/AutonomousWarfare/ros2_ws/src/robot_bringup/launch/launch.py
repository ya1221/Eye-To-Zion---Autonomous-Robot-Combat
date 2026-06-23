from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # --- Hardware (ros2_control, robot_state_publisher, twist_to_ackermann) --- #
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hardware'), 'launch', 'launch.py'])
        ),
    )

    # --- Localization (YDLidar driver, SLAM Toolbox) --- #
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'launch.py'])
        ),
    )

    # --- Navigation (Nav2 stack) --- #
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('navigation'), 'launch', 'launch.py'])
        ),
    )

    # --- Foxglove Bridge (visualization) --- #
    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('foxglove'), 'launch', 'launch.py'])
        ),
    )

    telemetry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('telemetry_data'), 'launch', 'launch.py'])
        ),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('imu_sensor'), 'launch', 'launch.py'])
        ),
    )

    return LaunchDescription([
        LogInfo(msg='========== Robot Bringup Starting =========='),
        hardware_launch,
        localization_launch,
        navigation_launch,
        foxglove_launch,
        telemetry_launch,
        imu_launch,
        LogInfo(msg='========== All Subsystems Launched =========='),
    ])
