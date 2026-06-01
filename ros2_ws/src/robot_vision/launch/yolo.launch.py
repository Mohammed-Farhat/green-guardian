"""
Launch the YOLO detection node.

Run on the LAPTOP (same ROS_DOMAIN_ID=0 as the Pi):
    ros2 launch robot_vision yolo.launch.py

Override model path or confidence:
    ros2 launch robot_vision yolo.launch.py model_path:=/path/to/best.pt
    ros2 launch robot_vision yolo.launch.py confidence_threshold:=0.4

Monitor output:
    ros2 topic echo /garbage_detection
    ros2 run rqt_image_view rqt_image_view /camera/yolo_annotated
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.expanduser('~/green-guardian/ros2_ws/src/best.pt'),
        description='Absolute path to the YOLOv11n .pt model file',
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='YOLO confidence threshold (0.0 – 1.0)',
    )

    yolo_node = Node(
        package='robot_vision',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model_path':           LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        yolo_node,
    ])
