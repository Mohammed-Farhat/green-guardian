from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass(frozen=True)
class _Config:
    map_topic: str
    base_frame: str
    min_frontier_cells: int
    blacklist_radius: float
    tick_period_sec: float
    goal_timeout_sec: float


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_cells', 12)
        self.declare_parameter('blacklist_radius', 0.75)
        self.declare_parameter('tick_period_sec', 1.0)
        self.declare_parameter('goal_timeout_sec', 60.0)

        self._config = _Config(
            map_topic=str(self.get_parameter('map_topic').value),
            base_frame=str(self.get_parameter('base_frame').value),
            min_frontier_cells=int(self.get_parameter('min_frontier_cells').value),
            blacklist_radius=float(self.get_parameter('blacklist_radius').value),
            tick_period_sec=float(self.get_parameter('tick_period_sec').value),
            goal_timeout_sec=float(self.get_parameter('goal_timeout_sec').value),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self._config.map_topic, self._on_map, map_qos)

        self._active_goal: Optional[Tuple[float, float]] = None
        self._active_goal_start_time_sec: Optional[float] = None
        self._blacklisted_goals: List[Tuple[float, float]] = []
        self._done = False

        self.create_timer(self._config.tick_period_sec, self._tick)

        self.get_logger().info(
            'Waiting for map on %s and Nav2 action /navigate_to_pose...' % self._config.map_topic
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _tick(self) -> None:
        if self._map is None:
            return
        if self._done:
            return

        now_sec = float(self.get_clock().now().nanoseconds) * 1e-9

        if self._active_goal is not None:
            if (
                self._active_goal_start_time_sec is not None
                and now_sec - self._active_goal_start_time_sec > self._config.goal_timeout_sec
            ):
                self.get_logger().warn('Goal timed out; blacklisting and replanning.')
                self._blacklisted_goals.append(self._active_goal)
                self._active_goal = None
                self._active_goal_start_time_sec = None
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            return

        map_frame = self._map.header.frame_id or 'map'
        robot_xy = self._lookup_robot_xy(map_frame=map_frame, base_frame=self._config.base_frame)
        if robot_xy is None:
            return

        goal_xy = self._pick_frontier_goal(map_msg=self._map, robot_xy=robot_xy)
        if goal_xy is None:
            return

        self._send_goal(map_frame=map_frame, robot_xy=robot_xy, goal_xy=goal_xy)

    def _lookup_robot_xy(self, *, map_frame: str, base_frame: str) -> Optional[Tuple[float, float]]:
        try:
            tf_msg = self._tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                'No TF %s -> %s yet (%s)' % (map_frame, base_frame, str(exc))
            )
            return None
        return (tf_msg.transform.translation.x, tf_msg.transform.translation.y)

    def _pick_frontier_goal(
        self, *, map_msg: OccupancyGrid, robot_xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        if width <= 0 or height <= 0:
            return None

        data = map_msg.data
        if len(data) != width * height:
            self.get_logger().warn('Map data size mismatch; skipping.')
            return None

        frontier_mask = self._compute_frontier_mask(width=width, height=height, data=data)
        clusters = self._cluster_frontiers(
            width=width, height=height, frontier_mask=frontier_mask
        )
        clusters = [c for c in clusters if len(c) >= self._config.min_frontier_cells]
        if not clusters:
            self.get_logger().info('No frontiers left. Exploration complete.')
            self._done = True
            return None

        origin = map_msg.info.origin.position
        resolution = float(map_msg.info.resolution)

        best_goal: Optional[Tuple[float, float]] = None
        best_dist2 = float('inf')
        for cluster in clusters:
            goal_xy = self._cluster_centroid_xy(
                cluster=cluster, width=width, origin_xy=(origin.x, origin.y), resolution=resolution
            )
            if self._is_blacklisted(goal_xy):
                continue

            dx = goal_xy[0] - robot_xy[0]
            dy = goal_xy[1] - robot_xy[1]
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_goal = goal_xy

        return best_goal

    def _compute_frontier_mask(
        self, *, width: int, height: int, data: Sequence[int]
    ) -> List[bool]:
        def is_free(v: int) -> bool:
            return v == 0

        def is_unknown(v: int) -> bool:
            return v == -1

        frontier_mask = [False] * (width * height)
        for idx, val in enumerate(data):
            if not is_free(val):
                continue

            x = idx % width
            y = idx // width
            if x > 0 and is_unknown(data[idx - 1]):
                frontier_mask[idx] = True
                continue
            if x + 1 < width and is_unknown(data[idx + 1]):
                frontier_mask[idx] = True
                continue
            if y > 0 and is_unknown(data[idx - width]):
                frontier_mask[idx] = True
                continue
            if y + 1 < height and is_unknown(data[idx + width]):
                frontier_mask[idx] = True
                continue

        return frontier_mask

    def _cluster_frontiers(
        self, *, width: int, height: int, frontier_mask: Sequence[bool]
    ) -> List[List[int]]:
        def neighbor_indices(i: int) -> Iterable[int]:
            x = i % width
            y = i // width
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        yield ny * width + nx

        visited = [False] * (width * height)
        clusters: List[List[int]] = []

        for idx, is_frontier in enumerate(frontier_mask):
            if not is_frontier or visited[idx]:
                continue

            cluster: List[int] = []
            q: Deque[int] = deque([idx])
            visited[idx] = True

            while q:
                i = q.popleft()
                cluster.append(i)
                for n in neighbor_indices(i):
                    if frontier_mask[n] and not visited[n]:
                        visited[n] = True
                        q.append(n)

            clusters.append(cluster)

        return clusters

    def _cluster_centroid_xy(
        self,
        *,
        cluster: Sequence[int],
        width: int,
        origin_xy: Tuple[float, float],
        resolution: float,
    ) -> Tuple[float, float]:
        sx = 0.0
        sy = 0.0
        inv_n = 1.0 / float(len(cluster))
        for idx in cluster:
            cx = float(idx % width) + 0.5
            cy = float(idx // width) + 0.5
            sx += cx
            sy += cy
        mx = sx * inv_n
        my = sy * inv_n
        return (origin_xy[0] + mx * resolution, origin_xy[1] + my * resolution)

    def _is_blacklisted(self, goal_xy: Tuple[float, float]) -> bool:
        r2 = self._config.blacklist_radius * self._config.blacklist_radius
        for gx, gy in self._blacklisted_goals:
            dx = goal_xy[0] - gx
            dy = goal_xy[1] - gy
            if dx * dx + dy * dy <= r2:
                return True
        return False

    def _send_goal(
        self, *, map_frame: str, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> None:
        yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = map_frame
        pose.pose.position.x = float(goal_xy[0])
        pose.pose.position.y = float(goal_xy[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info('Navigating to frontier goal (%.2f, %.2f)' % goal_xy)

        send_future = self._nav_client.send_goal_async(goal, feedback_callback=None)
        send_future.add_done_callback(lambda fut: self._on_goal_response(fut, goal_xy))

    def _on_goal_response(self, future, goal_xy: Tuple[float, float]) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal; blacklisting and replanning.')
            self._blacklisted_goals.append(goal_xy)
            return

        self._active_goal = goal_xy
        self._active_goal_start_time_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, goal_xy))

    def _on_result(self, future, goal_xy: Tuple[float, float]) -> None:
        result = future.result()
        status = int(result.status)

        # GoalStatus.STATUS_SUCCEEDED == 4, but avoid importing goal_status to stay lightweight.
        if status == 4:
            self.get_logger().info('Reached frontier goal.')
        else:
            self.get_logger().warn('Failed to reach goal (status=%d); blacklisting.' % status)
            self._blacklisted_goals.append(goal_xy)

        self._active_goal = None
        self._active_goal_start_time_sec = None


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
