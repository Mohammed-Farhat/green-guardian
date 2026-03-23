from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass(frozen=True)
class _Config:
    frame_id: str
    x: float
    y: float
    yaw: float
    publish_count: int
    publish_period_sec: float


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_period_sec', 1.0)

        self._config = _Config(
            frame_id=str(self.get_parameter('frame_id').value),
            x=float(self.get_parameter('x').value),
            y=float(self.get_parameter('y').value),
            yaw=float(self.get_parameter('yaw').value),
            publish_count=int(self.get_parameter('publish_count').value),
            publish_period_sec=float(self.get_parameter('publish_period_sec').value),
        )

        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._remaining = max(0, self._config.publish_count)
        self._timer = self.create_timer(self._config.publish_period_sec, self._tick)

        self.get_logger().info(
            'Publishing initial pose to /initialpose: (x=%.2f y=%.2f yaw=%.2f rad) in frame %s'
            % (self._config.x, self._config.y, self._config.yaw, self._config.frame_id)
        )

    def _tick(self) -> None:
        if self._remaining <= 0:
            self._timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id
        msg.pose.pose.position.x = self._config.x
        msg.pose.pose.position.y = self._config.y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = _quaternion_from_yaw(self._config.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Reasonable defaults: (x, y, yaw) variances.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942  # ~= (15deg)^2

        self._pub.publish(msg)
        self._remaining -= 1


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

