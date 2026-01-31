import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro


def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_gazebo_pkg = get_package_share_directory('robot_gazebo')
    controllers_yaml = os.path.join(robot_gazebo_pkg, 'config', 'controllers.yaml')


    xacro_path = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    world_path = os.path.join(robot_gazebo_pkg, 'worlds', 'empty.world')

    # Xacro -> URDF
    doc = xacro.process_file(xacro_path, mappings={'controllers_yaml': controllers_yaml})
    robot_urdf = doc.toxml()

    # Write URDF to a temp file for ros_gz_sim create
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    tmp.write(robot_urdf.encode("utf-8"))
    tmp.close()

    # Start Gazebo (Harmonic) using ros_gz_sim launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
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
            '-x', '0.0', '-y', '0.0', '-z', '0.20'
        ],
        output='screen'
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_dd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawner_jsb, spawner_dd]
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        rsp,
        spawn,
        spawn_controllers
])
