from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class _Config:
    input_topic: str
    output_topic: str
    frame_id: str


class CmdVelStamper(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_stamper')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')

        self._config = _Config(
            input_topic=str(self.get_parameter('input_topic').value),
            output_topic=str(self.get_parameter('output_topic').value),
            frame_id=str(self.get_parameter('frame_id').value),
        )

        self._pub = self.create_publisher(TwistStamped, self._config.output_topic, 10)
        self.create_subscription(Twist, self._config.input_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            'Stamping %s (Twist) -> %s (TwistStamped)'
            % (self._config.input_topic, self._config.output_topic)
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._config.frame_id
        out.twist = msg
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
