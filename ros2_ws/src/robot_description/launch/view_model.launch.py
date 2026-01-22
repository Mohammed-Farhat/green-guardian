from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "urdf",
        "green_guardian.urdf.xacro"
    ])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "rviz",
        "green_guardian.rviz"
    ])

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        ),
    ])
