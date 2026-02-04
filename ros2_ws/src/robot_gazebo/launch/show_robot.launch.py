import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    controllers_yaml = os.path.join(get_package_share_directory('robot_gazebo'), 'config', 'controllers.yaml')

    xacro_path = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([FindPackageShare('robot_gazebo'), 'worlds', world])

    # Xacro -> URDF
    doc = xacro.process_file(xacro_path, mappings={'controllers_yaml': controllers_yaml})
    robot_urdf = doc.toxml()

    # Write URDF to a temp file for ros_gz_sim create
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    tmp.write(robot_urdf.encode('utf-8'))
    tmp.close()

    # Start Gazebo (Harmonic) using ros_gz_sim launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/green_guardian/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[('/scan', '/scan_raw')],
        output='screen',
    )

    scan_frame_rewriter = Node(
        package='robot_gazebo',
        executable='scan_frame_rewriter',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
                'frame_id': 'laser_frame',
            }
        ],
    )

    ground_truth_tf = Node(
        package='robot_gazebo',
        executable='ground_truth_tf_broadcaster',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'input_odom_topic': '/model/green_guardian/odometry',
                'output_odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
            }
        ],
    )

    # Publish TF from robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}],
    )

    # Spawn robot into Gazebo (this is the correct Jazzy/Harmonic way)
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'green_guardian',
            '-file', tmp.name,
            '-x', '0.0', '-y', '0.0', '-z', '0.20', '-Y', '0.0'
        ],
        output='screen'
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawner_dd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawner_jsb, spawner_dd]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='World file from robot_gazebo/worlds (e.g. empty.world, arena_large.world)',
        ),
        gz_sim,
        gz_bridge,
        rsp,
        scan_frame_rewriter,
        ground_truth_tf,
        spawn,
        spawn_controllers
    ])
