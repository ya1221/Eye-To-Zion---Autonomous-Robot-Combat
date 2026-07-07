from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    foxglove_bridge_launch_dir = os.path.join(
        get_package_share_directory('foxglove_bridge'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(foxglove_bridge_launch_dir, 'foxglove_bridge_launch.xml')
            ),
            # Pass the list of topics you want to whitelist here
            launch_arguments={
                'topic_whitelist': "['/robot_description', '/map', '/tf', '/plan']"
            }.items()
        )
    ])