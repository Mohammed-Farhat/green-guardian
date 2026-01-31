from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Use xacro to expand at runtime
    robot_description = os.popen(f"xacro {urdf_path}").read()
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_config.rviz')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
    ])
