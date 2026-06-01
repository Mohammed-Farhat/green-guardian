"""
Launch the mission control node.

Run on the Pi:
    ros2 launch robot_vision mission.launch.py

Override serial port or cooldown:
    ros2 launch robot_vision mission.launch.py gripper_serial_port:=/dev/ttyUSB0
    ros2 launch robot_vision mission.launch.py cooldown_seconds:=15.0

Check port assignment after plugging Arduino #2:
    ls /dev/ttyUSB*
    sudo usermod -a -G dialout $USER   # (once, then re-login)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'gripper_serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port connected to Arduino #2 (gripper FSM)',
    )

    cooldown_arg = DeclareLaunchArgument(
        'cooldown_seconds',
        default_value='10.0',
        description='Seconds to ignore detections after a gripper trigger',
    )

    mission_node = Node(
        package='robot_vision',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'gripper_serial_port': LaunchConfiguration('gripper_serial_port'),
            'cooldown_seconds':    LaunchConfiguration('cooldown_seconds'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        cooldown_arg,
        mission_node,
    ])
