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
            PathJoinSubstitution([FindPackageShare('icm20948_ros2'), 'launch', 'icm20948.launch.py'])
        ),
    )

    shooting_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('shooting'), 'launch', 'launch.py'])
        ),
    )

    # --- Sensor fusion (YOLO /ai/detections + /scan -> local_enemy_position) --- #
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('sensor_fusion_pkg'), 'launch', 'main.launch.py'])
        ),
    )

    # --- Tactical brain (BT + A* orchestrator, drives controller_server directly) --- #
    tactical_brain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tactical_brain'), 'launch', 'main.launch.py'])
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
        shooting_launch,
        sensor_fusion_launch,
        tactical_brain_launch,
        LogInfo(msg='========== All Subsystems Launched =========='),
    ])
