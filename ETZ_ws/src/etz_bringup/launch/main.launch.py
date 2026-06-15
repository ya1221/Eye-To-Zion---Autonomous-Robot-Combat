from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # מציאת הנתיבים לחבילות
    brain_pkg = FindPackageShare('tactical_brain')
    sensors_pkg = FindPackageShare('sensor_fusion_pkg')

    # ייבוא ה-Launch של המוח
    launch_brain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(brain_pkg.find('tactical_brain'), 'launch', 'main.launch.py')
        )
    )

    # ייבוא ה-Launch של החיישנים
    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_pkg.find('sensor_fusion_pkg'), 'launch', 'main.launch.py')
        )
    )

    # החזרת רשימה שמריצה את שניהם יחד (החיישנים קודם)
    return LaunchDescription([
        launch_sensors,
        launch_brain
    ])