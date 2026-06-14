import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')
    pkg_dir = get_package_share_directory('robot_description')

    world = PathJoinSubstitution([pkg_share, 'world', 'maze', 'maze.world'])
    robot_description_path = os.path.join(pkg_dir, 'world', 'robot', 'robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[PathJoinSubstitution([pkg_share, 'world']), ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', world, '--verbose',
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
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
        arguments=['-entity', 'robot1', '-topic', '/robot_description',
                   '-x', '-1', '-y', '0', '-z', '1'],
        output='screen'
    )

    robot_controller = Node(
        package='robot_description',
        executable='robot_controller_node',
        name='robot_controller',
        output='screen'
    )

    pkg_dir = get_package_share_directory('robot_description')
    jsb_yaml       = os.path.join(pkg_dir, 'config', 'joint_state_broadcaster.yaml')
    ackermann_yaml = os.path.join(pkg_dir, 'config', 'ackermann_steering_controller.yaml')

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', jsb_yaml,
        ],
    )

    ackermann_steering_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', ackermann_yaml,
        ],
    )

    # Strict sequencing: robot must be spawned before JSB, JSB before Ackermann
    spawn_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )
    spawn_ack = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[ackermann_steering_controller])
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='RCL_ARGS', value=''),
        SetEnvironmentVariable(name='ROS_ARGUMENTS', value=''),
        set_model_path,
        TimerAction(period=0.5, actions=[gzserver]),
        TimerAction(period=1.0, actions=[gzclient]),
        rsp,
        TimerAction(period=2.0, actions=[spawn_robot]),
        robot_controller,
        spawn_jsb,
        spawn_ack,
    ])
