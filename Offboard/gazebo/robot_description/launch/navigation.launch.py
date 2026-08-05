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
        FindPackageShare('robot_description'), 'config', 'nav2.yaml'
    ])

    default_bt_xml_file = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    ])


    # --- Declare Launch Arguments (CLI Interface) --- #
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    ipc_arg     = DeclareLaunchArgument('use_intra_process_comms', default_value='true')
    autostart_arg    = DeclareLaunchArgument('autostart', default_value='true')
    params_file_arg  = DeclareLaunchArgument('params_file', default_value=default_params_file)
    default_bt_arg   = DeclareLaunchArgument('default_bt_xml', default_value=default_bt_xml_file)

    # ---  Capture Values --- #
    use_sim_time_val = LaunchConfiguration('use_sim_time')
    # params_file_val = LaunchConfiguration('params_file')
    # ipc_val = LaunchConfiguration('use_intra_process_comms')

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

    # nav2_container = ComposableNodeContainer(
    #     name='nav2_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='nav2_planner',
    #             plugin='nav2_planner::PlannerServer',
    #             name='planner_server', 
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_controller',
    #             plugin='nav2_controller::ControllerServer',
    #             name='controller_server',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_bt_navigator',
    #             plugin='nav2_bt_navigator::BtNavigator',
    #             name='bt_navigator',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_behaviors',
    #             plugin='behavior_server::BehaviorServer',
    #             name='behavior_server',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_smoother',
    #             plugin='nav2_smoother::SmootherServer',
    #             name='smoother_server',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_velocity_smoother',
    #             plugin='nav2_velocity_smoother::VelocitySmoother',
    #             name='velocity_smoother',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #             package='nav2_waypoint_follower',
    #             plugin='nav2_waypoint_follower::WaypointFollower',
    #             name='waypoint_follower',
    #             parameters=[params_file_val],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         ),
    #         ComposableNode(
    #            package='robot_description',
    #             plugin='robot_description::TwistToAckermann', 
    #             name='twist_to_ackermann_node',
    #             parameters=[{'use_sim_time': use_sim_time_val}],
    #             extra_arguments=[{'use_intra_process_comms': False}]
    #         )
    #     ],
    #     output='screen',
    # )

    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     parameters=[params_file_val]


    # twist_to_ackermann_cmd = Node(
    #     package='robot_description',
    #     executable='TwistToAckermann',
    #     name='twist_to_ackermann_node',
    #     parameters=[{'use_sim_time': use_sim_time_val}],
    #     output='screen'
    # )

    return LaunchDescription([
        # Arguments first
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        default_bt_arg,
        ipc_arg,
        nav2_launch,

        # Actions second
        # twist_to_ackermann_cmd
    ])
