# ============================================================
#  GREEN GUARDIAN — SLAM Bringup Launch
#  Package: robot_slam
#
#  Starts everything needed to build a map with slam_toolbox:
#    1. motor_bridge       — wheels + odometry  (/odom, /cmd_vel)
#    2. imu_node + EKF     — fused odometry TF  (odom → base_footprint)
#    3. rplidar_node       — laser scans         (/scan, frame: laser_link)
#    4. robot_state_publisher + slam_toolbox
#
#  Usage:
#    ros2 launch robot_slam slam_bringup.launch.py
#
#  Override serial ports if needed:
#    ros2 launch robot_slam slam_bringup.launch.py \
#        motor_port:=/dev/ttyACM1 lidar_port:=/dev/ttyUSB1
#
#  Drive the robot (separate terminal):
#    ros2 run teleop_twist_keyboard teleop_twist_keyboard
#
#  Save the map when done (separate terminal):
#    ros2 run nav2_map_server map_saver_cli -f $HOME/arena_map
#    → produces $HOME/arena_map.yaml  and  $HOME/arena_map.pgm
#
#  ⚠️  Use $HOME — NOT ~/  — when passing the path to ros2 launch.
#      Bash does NOT expand ~ after the := argument operator.
#
#  Then stop this launch (Ctrl-C) and run robot_navigation instead.
# ============================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────
    motor_port_arg = DeclareLaunchArgument(
        'motor_port',
        default_value='/dev/ttyACM0',
        description='Serial port of the motor/encoder Arduino (ls /dev/tty* to find it)'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of the RPLidar A1 (ls /dev/tty* to find it)'
    )

    # ── 1. Motor bridge ───────────────────────────────────────────
    # Handles /cmd_vel → PWM and encoder ticks → /odom
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_motors'),
                'launch', 'motor_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('motor_port'),
        }.items()
    )

    # ── 2. IMU node + EKF ─────────────────────────────────────────
    # imu_node → /imu  |  ekf_filter_node fuses /odom + /imu
    # EKF publishes TF: odom → base_footprint
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_imu'),
                'launch', 'imu.launch.py'
            ])
        ])
    )

    # ── 3. RPLidar A1 ────────────────────────────────────────────
    # Publishes /scan.  frame_id MUST be laser_link to match the URDF TF tree.
    # The rplidar_a1_launch.py default is 'laser' — we override it here.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch', 'rplidar_a1_launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id':    'laser_link',   # must match sensors.xacro
        }.items()
    )

    # ── 4. SLAM (robot_state_publisher + slam_toolbox) ───────────
    # RSP broadcasts fixed TF frames (base_link → laser_link, etc.)
    # slam_toolbox builds the occupancy map and publishes map → odom TF
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch', 'slam.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        # Arguments
        motor_port_arg,
        lidar_port_arg,

        # Informational message printed at launch time
        LogInfo(msg=[
            '\n\n',
            '╔══════════════════════════════════════════════════════╗\n',
            '║          GREEN GUARDIAN  —  SLAM  BRINGUP            ║\n',
            '╠══════════════════════════════════════════════════════╣\n',
            '║  Drive:  ros2 run teleop_twist_keyboard               ║\n',
            '║           teleop_twist_keyboard                       ║\n',
            '║                                                       ║\n',
            '║  Save map (when done mapping):                        ║\n',
            '║    ros2 run nav2_map_server map_saver_cli             ║\n',
            '║             -f $HOME/arena_map                        ║\n',
            '║  (use $HOME not ~/ — tilde not expanded by ros2)      ║\n',
            '║                                                       ║\n',
            '║  Then Ctrl-C this terminal and launch nav2 instead.   ║\n',
            '╚══════════════════════════════════════════════════════╝\n',
        ]),

        # Nodes
        motor_launch,
        imu_launch,
        lidar_launch,
        slam_launch,
    ])
