#!/usr/bin/env python3
"""
Random wanderer for Green Guardian.

Picks a random *free* cell on the loaded map and sends it as a NavigateToPose
goal. When the goal finishes (success or fail), picks another one.

Used for the autonomous-wander demo while the vision pipeline is not yet
integrated — gives the impression of "looking for waste" by visiting random
free cells.

Usage (after nav_bringup is up and AMCL is localised):
    python3 random_wanderer.py
"""
import math
import random
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class RandomWanderer(Node):
    def __init__(self):
        super().__init__('random_wanderer')

        self.declare_parameter('cell_margin', 4)   # keep N cells away from walls
        self.declare_parameter('settle_sec', 1.0)  # pause between goals
        self.cell_margin = int(self.get_parameter('cell_margin').value)
        self.settle_sec  = float(self.get_parameter('settle_sec').value)

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # /map is published TRANSIENT_LOCAL by map_server
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos
        )

        self.map: OccupancyGrid | None = None
        self.get_logger().info('Waiting for /map …')

    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        if self.map is not None:
            return
        self.map = msg
        w, h, r = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        self.get_logger().info(
            f'Map received: {w}x{h} @ {r:.3f} m/px, origin ({ox:.2f}, {oy:.2f})'
        )
        self.get_logger().info('Waiting for /navigate_to_pose action server …')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server ready. Wandering!')
        self.send_random_goal()

    # ------------------------------------------------------------------
    def _random_free_pose(self):
        """Return (x, y, yaw) in the map frame, on a free cell, or None."""
        info = self.map.info
        data = self.map.data
        w, h = info.width, info.height
        m = self.cell_margin

        for _ in range(200):
            mx = random.randint(m, w - 1 - m)
            my = random.randint(m, h - 1 - m)

            # Reject if this cell or any neighbour in the margin is non-free.
            ok = True
            for dy in range(-m, m + 1):
                for dx in range(-m, m + 1):
                    if data[(my + dy) * w + (mx + dx)] != 0:
                        ok = False
                        break
                if not ok:
                    break
            if not ok:
                continue

            wx = info.origin.position.x + (mx + 0.5) * info.resolution
            wy = info.origin.position.y + (my + 0.5) * info.resolution
            yaw = random.uniform(-math.pi, math.pi)
            return wx, wy, yaw

        return None

    # ------------------------------------------------------------------
    def send_random_goal(self):
        pose = self._random_free_pose()
        if pose is None:
            self.get_logger().warn('No free cell found — retrying in 2 s')
            self.create_timer(2.0, self._retry_once)
            return

        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'New goal: x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.0f}°'
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _retry_once(self):
        # one-shot timer; cancel itself and retry the goal
        self.send_random_goal()

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by server — picking another')
            self.send_random_goal()
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED  (action_msgs/GoalStatus)
        name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, str(status))
        self.get_logger().info(f'Goal finished ({name}) — next in {self.settle_sec:.1f}s')
        self.create_timer(self.settle_sec, self._next_after_pause)

    def _next_after_pause(self):
        self.send_random_goal()


def main():
    rclpy.init()
    node = RandomWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
