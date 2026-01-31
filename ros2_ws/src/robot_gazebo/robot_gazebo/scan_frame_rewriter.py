from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class _Config:
    input_topic: str
    output_topic: str
    frame_id: str


class ScanFrameRewriter(Node):
    def __init__(self) -> None:
        super().__init__('scan_frame_rewriter')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('frame_id', 'laser_frame')

        self._config = _Config(
            input_topic=str(self.get_parameter('input_topic').value),
            output_topic=str(self.get_parameter('output_topic').value),
            frame_id=str(self.get_parameter('frame_id').value),
        )

        self._pub = self.create_publisher(LaserScan, self._config.output_topic, 10)
        self.create_subscription(LaserScan, self._config.input_topic, self._on_scan, 10)

        self.get_logger().info(
            'Rewriting %s frame_id -> %s (publishing on %s)'
            % (self._config.input_topic, self._config.frame_id, self._config.output_topic)
        )

    def _on_scan(self, msg: LaserScan) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._config.frame_id
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanFrameRewriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
