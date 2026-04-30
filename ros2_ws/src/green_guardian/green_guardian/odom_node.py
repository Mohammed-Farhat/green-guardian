#!/usr/bin/env python3
"""
odom_node.py
Placeholder odometry publisher — publishes zero velocity until
motor_bridge (Arduino #1) is connected and publishing real /odom.

Once motor_bridge is running it will publish to /odom directly,
and this node should be removed from the launch file.

Topics published:
  /odom  (nav_msgs/Odometry) — linear.x = speed in m/s (zero placeholder)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomPlaceholderNode(Node):

    def __init__(self):
        super().__init__('odom_placeholder_node')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz
        self.get_logger().info('OdomPlaceholderNode started (publishing zeros).')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        # All velocities and pose = 0 by default
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPlaceholderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
