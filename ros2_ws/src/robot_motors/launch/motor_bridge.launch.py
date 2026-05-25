from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch the motor bridge node for Green Guardian.

    Usage:
        ros2 launch robot_motors motor_bridge.launch.py

    Override serial port:
        ros2 launch robot_motors motor_bridge.launch.py serial_port:=/dev/ttyACM0

    Test motor after launch (new terminal):
        ros2 topic pub /cmd_vel geometry_msgs/Twist \\
          "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

    Teleop keyboard (new terminal):
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

    Check odometry:
        ros2 topic echo /odom

    Check TF:
        ros2 run tf2_ros tf2_echo odom base_footprint

    ⚠️ Prerequisites:
        1. motor_control.ino uploaded to Arduino #1
        2. Arduino connected to Pi5 via USB
        3. 12V power supply connected to motor driver
        4. Check port: ls /dev/tty* after plugging Arduino
        5. Permissions: sudo usermod -a -G dialout $USER (then re-login)
    """

    # Allow serial port override from command line
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino #1'
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('robot_motors'), 'config', 'hardware_params.yaml'
    ])

    motor_bridge_node = Node(
        package='robot_motors',
        executable='motor_bridge',
        name='motor_bridge',
        output='screen',
        parameters=[
            params_file,
            {'serial_port': LaunchConfiguration('serial_port')},
            {'publish_tf': False},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        motor_bridge_node,
    ])
