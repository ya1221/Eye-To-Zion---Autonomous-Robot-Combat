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
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_description_path = os.path.join(robot_description_pkg, 'urdf', 'robot_urdf.xacro')

    hardware_pkg = get_package_share_directory('hardware')
    ackermann_yaml = os.path.join(hardware_pkg, 'config', 'ackermann_steering_controller.yaml')
    joint_state_broadcaster_yaml = os.path.join(hardware_pkg, 'config', 'joint_state_broadcaster.yaml')
    controller_manager_yaml = os.path.join(hardware_pkg, 'config', 'controller_manager.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_manager_yaml, ackermann_yaml, joint_state_broadcaster_yaml],
        output="both",
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description}]
    )
 

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    ackermann_steering_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    return LaunchDescription([
       controller_manager_node,
       rsp,
       joint_state_broadcaster,
       ackermann_steering_controller,
    ])