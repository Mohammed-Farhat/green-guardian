import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    world = LaunchConfiguration('world')
    use_cmd_vel_stamper = LaunchConfiguration('use_cmd_vel_stamper')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('robot_navigation')
    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')

    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'show_robot.launch.py')
        ),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(sim),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': nav2_params,
        }.items(),
    )

    cmd_vel_stamper = Node(
        package='robot_gazebo',
        executable='cmd_vel_stamper',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'input_topic': '/cmd_vel'},
            {'output_topic': '/diff_drive_controller/cmd_vel'},
            {'frame_id': 'base_footprint'},
        ],
        condition=IfCondition(use_cmd_vel_stamper),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument(
                'sim',
                default_value='true',
                description='Launch Gazebo simulation if true',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='empty.world',
                description='World file from robot_gazebo/worlds (e.g. empty.world, arena_large.world)',
            ),
            DeclareLaunchArgument(
                'use_cmd_vel_stamper',
                default_value='true',
                description='Convert /cmd_vel (Twist) -> diff_drive_controller/cmd_vel (TwistStamped)',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Launch RViz if true',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=os.path.join(pkg_share, 'rviz', 'nav2_view.rviz'),
                description='Full path to an RViz2 config file',
            ),
            DeclareLaunchArgument(
                'nav2_params',
                default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
                description='Full path to Nav2 parameters YAML',
            ),
            DeclareLaunchArgument(
                'slam_params',
                default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml'),
                description='Full path to SLAM Toolbox parameters YAML',
            ),
            robot_gazebo_launch,
            slam_launch,
            TimerAction(period=5.0, actions=[nav2_launch]),
            TimerAction(period=6.0, actions=[cmd_vel_stamper]),
            rviz_node,
        ]
    )
