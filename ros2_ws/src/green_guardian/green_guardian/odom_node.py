#!/usr/bin/env python3
"""
odom_node.py
Velocity bridge — subscribes to /cmd_vel and republishes those velocities
on /odom so the web dashboard always reflects actual commanded speed.

Resets to zero if no /cmd_vel message is received within 0.5 s (key/button
released). Replace with real motor-bridge odometry when the Arduino is wired.

Topics:
  subscribed:  /cmd_vel  (geometry_msgs/Twist)
  published:   /odom     (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

CMD_TIMEOUT_S = 0.5   # reset speed to 0 if no cmd_vel received within this interval
PUBLISH_HZ    = 20


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_placeholder_node')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self._linear_x  = 0.0
        self._angular_z = 0.0
        self._last_cmd  = self.get_clock().now()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_odom)
        self.get_logger().info('OdomNode started — mirroring /cmd_vel → /odom.')

    def _cmd_vel_cb(self, msg: Twist):
        self._linear_x  = msg.linear.x
        self._angular_z = msg.angular.z
        self._last_cmd  = self.get_clock().now()

    def _publish_odom(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd).nanoseconds / 1e9
        if elapsed > CMD_TIMEOUT_S:
            self._linear_x  = 0.0
            self._angular_z = 0.0

        msg = Odometry()
        msg.header.stamp        = now.to_msg()
        msg.header.frame_id     = 'odom'
        msg.child_frame_id      = 'base_link'
        msg.twist.twist.linear.x  = self._linear_x
        msg.twist.twist.angular.z = self._angular_z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
