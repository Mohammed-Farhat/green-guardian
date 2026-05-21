from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launches a minimal RViz2 preview of the Green Guardian URDF.

    Nodes started:
      1. robot_state_publisher  — parses XACRO, publishes /robot_description
                                  and broadcasts all fixed TF frames
      2. joint_state_publisher_gui — publishes wheel joint states so the
                                     continuous wheel joints are not frozen
      3. rviz2                  — opens the visualizer

    Run with:
        ros2 launch robot_description display.launch.py

    Hardware NOT required — runs fully on the laptop.
    """

    pkg_share = FindPackageShare('robot_description')

    # ---------------------------------------------------------------------------
    # Process the XACRO file into a URDF string at launch time.
    # Command() runs `xacro robot.urdf.xacro` and captures its output.
    # ---------------------------------------------------------------------------
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    return LaunchDescription([

        # 1. Robot State Publisher
        #    Reads robot_description param, publishes TF for every fixed joint.
        #    Required by RViz2 to render the model and by Nav2 for transforms.
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

        # 2. Joint State Publisher GUI
        #    Only for no-hardware URDF preview — gives sliders to spin wheels.
        #    When motor_bridge is running it owns /joint_states from real encoder
        #    data; launching this GUI alongside it causes conflicts.
        #    Uncomment only when previewing the URDF without hardware.
        #
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen',
        # ),

        # 3. RViz2
        #    Opens with default config — you will need to:
        #      a) Set Fixed Frame to "base_footprint" (or "base_link")
        #      b) Add a RobotModel display → Topic: /robot_description
        #      c) Add a TF display to verify all frames
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

    ])
