"""
Launch the usb_cam driver for the Logitech C920.

Why usb_cam instead of v4l2_camera:
    On the Pi 5 (kernel 6.8.x-raspi) the ros2 v4l2_camera node fails for the
    C920 — raw YUYV trips a vb2_core_reqbufs WARNING in uvcvideo ("Failed
    mapping device memory"), and MJPG mmaps fine but that build can't decode
    it. usb_cam decodes MJPG → rgb8 in userspace, using the MJPG path the
    driver accepts.

Install first:
    sudo apt install ros-jazzy-usb-cam

Run on the Pi:
    ros2 launch robot_vision usb_cam.launch.py

Override device:
    ros2 launch robot_vision usb_cam.launch.py video_device:=/dev/video2

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
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        output='screen',
        parameters=[{
            'video_device':   LaunchConfiguration('video_device'),
            'image_width':    640,
            'image_height':   480,
            'framerate':      15.0,
            # mjpeg2rgb: read MJPG off the camera (the path that maps OK on the
            # Pi 5) and decode to rgb8. cv_bridge on the laptop turns rgb8 into
            # the bgr8 that yolo_node requests.
            'pixel_format':   'mjpeg2rgb',
            'io_method':      'mmap',
            'frame_id':       'camera_optical_link',
            'camera_name':    'camera',
        }],
        # usb_cam publishes the relative topic 'image_raw'; remap to the
        # agreed-upon /camera/image_raw so the rest of the stack is unchanged.
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        video_device_arg,
        camera_node,
    ])
