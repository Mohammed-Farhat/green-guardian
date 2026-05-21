from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('robot_imu'),
        'config', 'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus':      1,
                'i2c_addr':     0x68,
                'publish_rate': 50.0,
                'frame_id':     'imu_link',
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
