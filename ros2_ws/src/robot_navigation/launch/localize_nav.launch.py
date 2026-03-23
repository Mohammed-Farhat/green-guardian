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
    map_yaml = LaunchConfiguration('map')
    use_cmd_vel_stamper = LaunchConfiguration('use_cmd_vel_stamper')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('robot_navigation')
    nav2_params = LaunchConfiguration('nav2_params')

    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'show_robot.launch.py')
        ),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(sim),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': nav2_params,
            'map': map_yaml,
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

    initial_pose_publisher = Node(
        package='robot_navigation',
        executable='initial_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'frame_id': 'map'},
            {'x': ParameterValue(initial_pose_x, value_type=float)},
            {'y': ParameterValue(initial_pose_y, value_type=float)},
            {'yaw': ParameterValue(initial_pose_yaw, value_type=float)},
            {'publish_count': 5},
            {'publish_period_sec': 1.0},
        ],
        condition=IfCondition(auto_initial_pose),
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
                'map',
                default_value='',
                description='Full path to a map YAML file (required for AMCL localization)',
            ),
            DeclareLaunchArgument(
                'auto_initial_pose',
                default_value='true',
                description='Publish an initial pose (0,0,0) on /initialpose to bootstrap AMCL',
            ),
            DeclareLaunchArgument(
                'initial_pose_x',
                default_value='0.0',
                description='Initial pose X in map frame (meters)',
            ),
            DeclareLaunchArgument(
                'initial_pose_y',
                default_value='0.0',
                description='Initial pose Y in map frame (meters)',
            ),
            DeclareLaunchArgument(
                'initial_pose_yaw',
                default_value='0.0',
                description='Initial pose yaw in map frame (radians)',
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
            robot_gazebo_launch,
            localization_launch,
            TimerAction(period=2.0, actions=[initial_pose_publisher]),
            TimerAction(period=3.0, actions=[nav2_launch]),
            TimerAction(period=4.0, actions=[cmd_vel_stamper]),
            rviz_node,
        ]
    )
