import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    default_model = os.path.join(pkg_share, 'urdf', 'green_guardian.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to robot URDF/Xacro file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz,
        description='Absolute path to RViz config file'
    )

    # Run xacro -> robot_description
    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2,
    ])
