from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # ---  Define Default Paths --- #
    default_params_file = PathJoinSubstitution([
        FindPackageShare('navigation'), 'config', 'nav2.yaml'
    ])

    default_bt_xml_file = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    ])


    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    ipc_arg     = DeclareLaunchArgument('use_intra_process_comms', default_value='false')
    autostart_arg    = DeclareLaunchArgument('autostart', default_value='true')
    params_file_arg  = DeclareLaunchArgument('params_file', default_value=default_params_file)
    default_bt_arg   = DeclareLaunchArgument('default_bt_xml', default_value=default_bt_xml_file)

    # ---  Capture Values --- #
    use_sim_time_val = LaunchConfiguration('use_sim_time')
    params_file_val = LaunchConfiguration('params_file')
    ipc_val = LaunchConfiguration('use_intra_process_comms')


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time':            use_sim_time_val,
            'use_intra_process_comms': LaunchConfiguration('use_intra_process_comms'),
            'autostart':               LaunchConfiguration('autostart'),
            'params_file':             LaunchConfiguration('params_file'),
            'default_bt_xml_filename': LaunchConfiguration('default_bt_xml'),
        }.items(),
    )


    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        default_bt_arg,
        ipc_arg,
        nav2_launch,

    ])