#!/usr/bin/env python3
"""
bin_levels_node.py
Publishes organic and non-organic bin fill levels at 1 Hz.

Currently a placeholder — levels are set to 0.0 until real sensors are wired.
To manually simulate levels, publish to:
  /bins/set_organic      (std_msgs/Float32) — value 0.0 to 100.0
  /bins/set_non_organic  (std_msgs/Float32) — value 0.0 to 100.0

Topics published:
  /bins/organic      (std_msgs/Float32) — fill percent 0–100
  /bins/non_organic  (std_msgs/Float32) — fill percent 0–100

Service:
  /bins/reset  (std_srvs/Trigger) — reset both bins to 0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class BinLevelsNode(Node):

    def __init__(self):
        super().__init__('bin_levels_node')

        # Internal state
        self.organic_level     = 0.0
        self.non_organic_level = 0.0

        # Publishers
        self.pub_organic     = self.create_publisher(Float32, '/bins/organic',     10)
        self.pub_non_organic = self.create_publisher(Float32, '/bins/non_organic', 10)

        # Subscribers — allow manual override from other nodes (gripper FSM later)
        self.create_subscription(Float32, '/bins/set_organic',
                                 self._set_organic_cb, 10)
        self.create_subscription(Float32, '/bins/set_non_organic',
                                 self._set_non_organic_cb, 10)

        # Reset service — dashboard "Empty Bins" button calls this
        self.create_service(Trigger, '/bins/reset', self._reset_cb)

        self.timer = self.create_timer(1.0, self.publish_all)
        self.get_logger().info('BinLevelsNode started.')

    # ------------------------------------------------------------------
    def publish_all(self):
        self.pub_organic.publish(Float32(data=self.organic_level))
        self.pub_non_organic.publish(Float32(data=self.non_organic_level))

    def _set_organic_cb(self, msg: Float32):
        self.organic_level = max(0.0, min(100.0, msg.data))

    def _set_non_organic_cb(self, msg: Float32):
        self.non_organic_level = max(0.0, min(100.0, msg.data))

    def _reset_cb(self, request, response):
        self.organic_level     = 0.0
        self.non_organic_level = 0.0
        response.success = True
        response.message = 'Bins reset to 0'
        self.get_logger().info('Bins reset.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BinLevelsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
