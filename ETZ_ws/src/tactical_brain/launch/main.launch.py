import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Defaults to real wall-clock time (correct for the RPi5, which has no
    # Gazebo /clock). Gazebo testing sets USE_SIM_TIME=true via
    # docker-compose so timestamps match the rest of the nav2/TF stack.
    use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower() == 'true'

    return LaunchDescription([
        Node(
            package='tactical_brain',
            executable='brain_node', # ודא שזה השם שרשום ב-setup.py שלך
            name='tactical_brain_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        # zenoh_node owns the Zenoh session/subscriptions and republishes
        # the parsed fleet world-state on /world/enemies, /world/teammates,
        # and /odometry/filtered - tactical_brain_node only subscribes.
        Node(
            package='tactical_brain',
            executable='zenoh_node',
            name='zenoh_node',
            output='screen',
            parameters=[{
                'zenoh_anchor_endpoint': os.environ.get('ZENOH_ANCHOR_ENDPOINT', ''),
                'use_sim_time': use_sim_time
            }]
        )
    ])