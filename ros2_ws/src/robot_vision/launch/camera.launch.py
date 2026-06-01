"""
Launch the v4l2_camera driver for the Logitech C920.

Run on the Pi:
    ros2 launch robot_vision camera.launch.py

Override device:
    ros2 launch robot_vision camera.launch.py video_device:=/dev/video2

Verify stream (any machine on same ROS_DOMAIN_ID):
    ros2 topic hz /camera/image_raw
    ros2 run rqt_image_view rqt_image_view /camera/image_raw
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device path for the Logitech C920',
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device':   LaunchConfiguration('video_device'),
            'image_size':     [640, 480],
            # [numerator, denominator] → 1/15 s per frame = 15 fps
            'time_per_frame': [1, 15],
            'camera_frame_id': 'camera_optical_link',
            # Publish as raw so cv_bridge and image_transport can handle it.
            # NOTE: YUYV buffers are ~600 KB each; if the Pi reports
            # "Failed mapping device memory", raise CMA in
            # /boot/firmware/config.txt (cma=256M) and reboot.
            'pixel_format':   'YUYV',
        }],
        # Remap the relative topic ~/image_raw to the agreed-upon /camera/image_raw.
        # The node name is 'camera', so without this remap it would already be
        # /camera/image_raw, but being explicit avoids surprises.
        remappings=[
            ('~/image_raw', '/camera/image_raw'),
            ('~/camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        video_device_arg,
        camera_node,
    ])
