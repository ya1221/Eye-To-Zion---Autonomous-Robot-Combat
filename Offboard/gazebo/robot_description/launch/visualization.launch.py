import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')
    pkg_dir = get_package_share_directory('robot_description')
    config_path = os.path.join(pkg_dir, 'config', 'telegraf.conf')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz2', 'rviz2_panel.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Start Telegraf first so /tmp/telegraf.sock exists before telegraf_bridge_node connects
    telegraf_cmd = ExecuteProcess(
        cmd=['telegraf', '--config', config_path],
        output='screen',
        name='telegraf_process'
    )

    # Delay telegraf_bridge_node by 3s to give Telegraf time to create the socket
    telegraf_bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='robot_description',
            executable='telegraf_bridge_node',
            name='telegraf_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # map_to_image = Node(
    #     package='robot_description',
    #     executable='map_to_image_node',
    #     name='map_to_image_node',
    #     parameters=[{'use_sim_time': True}]
    # )

    gstreamer_sender = Node(
        package='robot_description',
        executable='gstreamer_sender_node',
        name='gstreamer_sender_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gstreamer_cmd = ExecuteProcess(
        cmd=[
            'gst-launch-1.0', '-v',
            'udpsrc', 'port=5000',
            'caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96',
            '!', 'rtph264depay',
            '!', 'h264parse',
            '!', 'avdec_h264',
            '!', 'videoconvert',
            '!', 'autovideosink', 'sync=false',
        ],
        output='screen',
        name='gstreamer_cmd'
    )

    return LaunchDescription([
        # telegraf_cmd,       # 1. Start Telegraf (creates the socket)
        # telegraf_bridge,    # 2. Connect after 3s delay
        rviz,
        gstreamer_cmd,
        gstreamer_sender,
    ])
