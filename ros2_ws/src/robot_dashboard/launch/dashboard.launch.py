"""
dashboard.launch.py
Launches all nodes needed to stream data to the web dashboard:
  - system_monitor_node  : CPU temp, usage, RAM, fan speed
  - bin_levels_node      : organic / non-organic bin levels
  - odom_placeholder_node: placeholder /odom (remove once motor_bridge is live)
  - rosbridge_websocket  : WebSocket bridge on port 9090 for the frontend
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    system_monitor = Node(
        package='robot_dashboard',
        executable='system_monitor_node',
        name='system_monitor_node',
        output='screen',
    )

    bin_levels = Node(
        package='robot_dashboard',
        executable='bin_levels_node',
        name='bin_levels_node',
        output='screen',
    )

    odom_placeholder = Node(
        package='robot_dashboard',
        executable='odom_node',
        name='odom_placeholder_node',
        output='screen',
    )

    return LaunchDescription([
        system_monitor,
        bin_levels,
        odom_placeholder,
        rosbridge_launch,
    ])
