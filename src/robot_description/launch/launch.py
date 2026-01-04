import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

    # Paths (installed/share-safe)
    world = PathJoinSubstitution([pkg_share, 'world', 'maze', 'maze.world'])
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'world', 'robot', 'robot.xml'
    )
    controllers_yaml = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'controllers.yaml'
    )

    ekf_config_path = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'ekf.yaml')
    
    slam_config_path = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'slam.yaml')
    

    # /robot_description for Gazebo spawn
    robot_description = ParameterValue(Command(['cat ', robot_description_path]), value_type=str)

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[PathJoinSubstitution([pkg_share, 'world']), ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]
    )

    gzserver = ExecuteProcess(
        cmd=['gdb','--ex','run','--ex','bt full','--args',
             'gzserver', world, '--verbose',
             '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',
            ],
        output='screen'
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-topic', '/robot_description', '-x', '-1.5', '-y', '0', '-z', '0.16'],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml
        ],
    )
    ackermann_steering_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml
        ],
    )

    robot_controller = Node(
        package='robot_description',
        executable='robot_controller_node',
        name='robot_controller',
        output='screen'
    )

    robot_map = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config_path, 
            {
            "odom_frame": "odom",
            "map_frame": "map",
            "base_frame": "base_link",
            "scan_topic": "scan",
            "mode": "mapping",
            "map_update_interval": 1.0,
            "use_sim_time": True
        }]
    )

    # --- Nav2 (relocatable) ---
    default_params_file = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'config', 'nav2.yaml'
    ])
    default_bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    ])

    # Launch args (so you can override via CLI)
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart    = DeclareLaunchArgument('autostart', default_value='true')
    params_file  = DeclareLaunchArgument('params_file', default_value=default_params_file)
    default_bt   = DeclareLaunchArgument('default_bt_xml', default_value=default_bt_xml)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
            'default_bt_xml_filename': LaunchConfiguration('default_bt_xml'),
        }.items(),
        
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/itay3711/AutonomousWarfare/src/robot_description/rviz2/rviz2_panel.rviz'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    TwistToAckermann = Node(
        package='robot_description',
        executable='TwistToAckermann',
        name='TwistToAckermann', 
    )

    Ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    Rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "publish_tf": False,
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "odom_topic": "/rf2o/odom",
            "laser_scan_topic": "/scan",
            "init_pose_from_topic": "/odom"
        }],
    )



    # Sequencing: spawn robot -> JSB -> Ackermann
    spawn_jsb = RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster]))
    spawn_ack = RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[ackermann_steering_controller]))

    return LaunchDescription([
        # Clean env; no global remap hacks
        SetEnvironmentVariable(name='RCL_ARGS', value=''),
        SetEnvironmentVariable(name='ROS_ARGUMENTS', value=''),

        set_model_path,
        TimerAction(period=0.5, actions=[gzserver]),
        TimerAction(period=1.0, actions=[gzclient]),
        rsp,
        TimerAction(period=2.0, actions=[spawn_robot]),
        spawn_jsb,
        spawn_ack,
        robot_controller,
        robot_map,

        use_sim_time, autostart, params_file, default_bt,
        nav2_launch,
        rviz,
        TwistToAckermann,
        Ekf,
        Rf2o
    ])
