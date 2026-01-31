from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class _Config:
    input_odom_topic: str
    output_odom_topic: str
    odom_frame: str
    base_frame: str
    publish_tf: bool
    publish_odom: bool


class GroundTruthTFBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_tf_broadcaster')

        self.declare_parameter('input_odom_topic', '/model/green_guardian/odometry')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odom', True)

        self._config = _Config(
            input_odom_topic=str(self.get_parameter('input_odom_topic').value),
            output_odom_topic=str(self.get_parameter('output_odom_topic').value),
            odom_frame=str(self.get_parameter('odom_frame').value),
            base_frame=str(self.get_parameter('base_frame').value),
            publish_tf=bool(self.get_parameter('publish_tf').value),
            publish_odom=bool(self.get_parameter('publish_odom').value),
        )

        self._tf_broadcaster = TransformBroadcaster(self) if self._config.publish_tf else None
        self._odom_pub = (
            self.create_publisher(Odometry, self._config.output_odom_topic, 10)
            if self._config.publish_odom
            else None
        )

        self.create_subscription(Odometry, self._config.input_odom_topic, self._on_odom, 10)

        self.get_logger().info(
            'Bridging ground-truth odom %s -> %s, TF %s->%s (publish_tf=%s publish_odom=%s)'
            % (
                self._config.input_odom_topic,
                self._config.output_odom_topic,
                self._config.odom_frame,
                self._config.base_frame,
                self._config.publish_tf,
                self._config.publish_odom,
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._config.odom_frame
            tf_msg.child_frame_id = self._config.base_frame
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

        if self._odom_pub is not None:
            out = Odometry()
            out.header.stamp = stamp
            out.header.frame_id = self._config.odom_frame
            out.child_frame_id = self._config.base_frame
            out.pose = msg.pose
            out.twist = msg.twist
            self._odom_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GroundTruthTFBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
