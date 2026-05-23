from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    nav2_config = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'nav2_params.yaml'
    )

    map_file = LaunchConfiguration('map')

    pkg_share = FindPackageShare('robot_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'use_sim_time': False,
            }],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_config, {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }]
        ),
    ])
