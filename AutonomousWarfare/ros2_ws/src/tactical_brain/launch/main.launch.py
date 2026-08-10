import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Defaults to real wall-clock time (correct for the RPi5, which has no
    # Gazebo /clock). Gazebo testing sets USE_SIM_TIME=true via
    # docker-compose so timestamps match the rest of the nav2/TF stack.
    use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower() == 'true'

    # Identity/calibration on the overhead-camera tracker, relayed via the zenoh-bridge-ros2dds container (see docker-compose.yml) - tactical_brain_node just subscribes to its plain ROS2 topics directly.
    return LaunchDescription([
        Node(
            package='tactical_brain',
            executable='brain_node', # ודא שזה השם שרשום ב-setup.py שלך
            name='tactical_brain_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_id': int(os.environ.get('ROBOT_ID', 3)),
                'my_team_idx': int(os.environ.get('MY_TEAM_IDX', 0)),
                'arena_size_meters': float(os.environ.get('ARENA_SIZE_METERS', 2.0)),
                'grid_n': int(os.environ.get('GRID_N', 2000)),
            }]
        )
    ])