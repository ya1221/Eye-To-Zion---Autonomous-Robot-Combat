from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

    def include(filename):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', filename])
            )
        )

    return LaunchDescription([
        # 1. Gazebo + RSP + spawn + controllers (event-driven sequencing inside)
        include('simulation.launch.py'),

        # 2. EKF + rf2o + SLAM (needs /scan and /odom — controllers must be up first)
        TimerAction(period=8.0, actions=[include('localization.launch.py')]),

        # 3. Nav2 + TwistToAckermann (needs odom→map TF chain up)
        TimerAction(period=12.0, actions=[include('navigation.launch.py')]),

        # 4. RViz2 + TelegrafBridge (purely visual, start last)
        TimerAction(period=14.0, actions=[include('visualization.launch.py')]),

    ])



