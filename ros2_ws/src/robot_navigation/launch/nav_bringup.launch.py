# ============================================================
#  GREEN GUARDIAN — Autonomous Navigation Bringup Launch
#  Package: robot_navigation
#
#  Starts everything needed for fully autonomous Nav2 navigation:
#    1. motor_bridge       — wheels + odometry  (/odom, /cmd_vel)
#    2. imu_node + EKF     — fused odometry TF  (odom → base_footprint)
#    3. rplidar_node       — laser scans         (/scan, frame: laser_link)
#    4. robot_state_publisher
#    5. map_server         — loads the saved .yaml map
#    6. amcl               — particle-filter localisation (map → odom TF)
#    7. controller_server  — DWB local planner → /cmd_vel
#    8. planner_server     — NavFn global planner
#    9. behavior_server    — spin / backup / wait recoveries
#   10. bt_navigator       — NavigateToPose / NavigateThroughPoses
#   11. waypoint_follower  — FollowWaypoints
#   12. lifecycle_manager  — manages nodes 5-11
#
#  Usage:
#    ros2 launch robot_navigation nav_bringup.launch.py \
#        map:=$HOME/arena_map.yaml
#
#  Override serial ports if needed:
#    ros2 launch robot_navigation nav_bringup.launch.py \
#        map:=$HOME/arena_map.yaml \
#        motor_port:=/dev/ttyACM1 \
#        lidar_port:=/dev/ttyUSB1
#
#  ⚠️  Always use $HOME (not ~/) for the map path.
#      Bash does NOT expand ~ after the := argument operator.
#      map_server would receive the literal string "~/arena_map.yaml"
#      and fail with FileNotFoundError.
#
#  In RViz after launch:
#    1. Add: Map, LaserScan (/scan), RobotModel, TF, Path
#    2. Click "2D Pose Estimate" → click where the robot is on the map
#    3. Wait for the AMCL particle cloud to converge (~2 s)
#    4. Click "Nav2 Goal" → click destination → robot navigates autonomously
#
#  Prerequisites:
#    • Map saved with: ros2 run nav2_map_server map_saver_cli -f $HOME/arena_map
#    • SLAM launch must be stopped before starting this launch
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
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the saved map YAML (e.g. $HOME/arena_map.yaml). Use $HOME not ~/. REQUIRED.'
    )

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
    # Handles /cmd_vel → PWM  and  encoder ticks → /odom
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
    # EKF publishes TF: odom → base_footprint   (motor_bridge has publish_tf=False)
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
            'frame_id':    'laser_link',   # must match sensors.xacro → laser_link
        }.items()
    )

    # ── 4-12. Full Nav2 stack (RSP + map_server + AMCL + planners) ──
    # robot_description/nav2.launch.py already includes robot_state_publisher
    # and all Nav2 nodes managed by a single lifecycle_manager.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch', 'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        # Arguments (map must be first — it has no default value)
        map_arg,
        motor_port_arg,
        lidar_port_arg,

        # Informational message printed at launch time
        LogInfo(msg=[
            '\n\n',
            '╔══════════════════════════════════════════════════════╗\n',
            '║     GREEN GUARDIAN  —  AUTONOMOUS NAVIGATION         ║\n',
            '╠══════════════════════════════════════════════════════╣\n',
            '║  Map: ', LaunchConfiguration('map'), '\n',
            '╠══════════════════════════════════════════════════════╣\n',
            '║  In RViz:                                            ║\n',
            '║    1. Click "2D Pose Estimate"                        ║\n',
            '║       → click where the robot IS on the map          ║\n',
            '║    2. Wait for AMCL particles to converge (~2 s)     ║\n',
            '║    3. Click "Nav2 Goal"                               ║\n',
            '║       → click the destination on the map             ║\n',
            '╚══════════════════════════════════════════════════════╝\n',
        ]),

        # Nodes — order: hardware first, then Nav2
        # RViz runs on the LAPTOP, not the Pi (Pi has no display).
        # On your laptop run:
        #   ros2 run rviz2 rviz2 -d <path_to_nav2.rviz>
        motor_launch,
        imu_launch,
        lidar_launch,
        nav2_launch,
    ])
