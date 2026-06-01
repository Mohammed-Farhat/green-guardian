# ============================================================
#  GREEN GUARDIAN — Nav2 Launch (ROS 2 Jazzy)
#
#  Starts:
#    robot_state_publisher  — URDF + static TF
#    map_server             — loads the saved map
#    amcl                   — localises robot on the map
#    controller_server      — DWB local planner + cmd_vel output
#    planner_server         — NavFn global planner
#    behavior_server        — spin / backup / wait behaviours
#    bt_navigator           — NavigateToPose / NavigateThroughPoses
#    waypoint_follower      — FollowWaypoints action
#    lifecycle_manager      — manages all of the above
#
#  Usage:
#    ros2 launch robot_description nav2.launch.py map:=/full/path/to/map.yaml
#
#  Prerequisites (must already be running):
#    ros2 launch robot_motors  motor_bridge.launch.py
#    ros2 launch robot_imu     imu.launch.py          # EKF → odom→base_footprint TF
#    ros2 launch rplidar_ros   rplidar_a1_launch.py
#
#  In RViz after launch:
#    1. Add map, LaserScan, RobotModel, TF, and Path displays
#    2. Click "2D Pose Estimate" and click where the robot is on the map
#    3. Wait for AMCL particles to converge, then send a Nav2 Goal
# ============================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

        # ── Launch argument ───────────────────────────────────────
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file (e.g. ~/arena_map.yaml)'
        ),

        # ── robot_state_publisher ─────────────────────────────────
        # Parses XACRO and publishes all fixed TF frames below base_link.
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

        # ── map_server ────────────────────────────────────────────
        # Publishes the saved occupancy-grid map on /map.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_config, {'yaml_filename': map_file}]
        ),

        # ── amcl ──────────────────────────────────────────────────
        # Particle-filter localisation on the loaded map.
        # Publishes TF: map → odom
        # Set initial pose with "2D Pose Estimate" in RViz.
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── controller_server ─────────────────────────────────────
        # DWB local planner. Publishes /cmd_vel → motor_bridge.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── planner_server ────────────────────────────────────────
        # NavFn global planner — Dijkstra/A* on the global costmap.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── behavior_server ───────────────────────────────────────
        # Recovery behaviours: spin, backup, drive_on_heading, wait.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── bt_navigator ──────────────────────────────────────────
        # Behaviour-tree orchestration of the planners / controllers.
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── waypoint_follower ─────────────────────────────────────
        # FollowWaypoints action — needed for multi-point coverage paths.
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_config]
        ),

        # ── lifecycle_manager ─────────────────────────────────────
        # Drives the lifecycle state-machine for every Nav2 node above.
        # Order matters: map_server must be active before amcl can configure.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }]
        ),
    ])
