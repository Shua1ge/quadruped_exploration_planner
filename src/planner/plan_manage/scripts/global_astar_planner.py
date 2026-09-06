#!/usr/bin/env python3
"""Deterministic 2-D global A* adapter for SCAN-Planner reference-path mode."""

import heapq
import math
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


Cell = Tuple[int, int]
Point2 = Tuple[float, float]


class OccupancyGrid2D:
    def __init__(self, points: Sequence[Tuple[float, float, float]], resolution: float,
                 inflation_radius: float, obstacle_min_z: float, obstacle_max_z: float,
                 padding: float):
        filtered = [(x, y) for x, y, z in points if obstacle_min_z <= z <= obstacle_max_z]
        if not filtered:
            raise ValueError("global map contains no points in the configured body-height band")
        self.resolution = resolution
        self.min_x = math.floor((min(p[0] for p in filtered) - padding) / resolution) * resolution
        self.min_y = math.floor((min(p[1] for p in filtered) - padding) / resolution) * resolution
        max_x = math.ceil((max(p[0] for p in filtered) + padding) / resolution) * resolution
        max_y = math.ceil((max(p[1] for p in filtered) + padding) / resolution) * resolution
        self.width = int(round((max_x - self.min_x) / resolution)) + 1
        self.height = int(round((max_y - self.min_y) / resolution)) + 1

        raw = {self.world_to_cell(x, y) for x, y in filtered}
        radius_cells = int(math.ceil(inflation_radius / resolution))
        offsets = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx * resolution, dy * resolution) <= inflation_radius + 1e-9:
                    offsets.append((dx, dy))
        self.occupied: Set[Cell] = set()
        for cell_x, cell_y in raw:
            for dx, dy in offsets:
                candidate = (cell_x + dx, cell_y + dy)
                if self.in_bounds(candidate):
                    self.occupied.add(candidate)

    def world_to_cell(self, x: float, y: float) -> Cell:
        return (int(round((x - self.min_x) / self.resolution)),
                int(round((y - self.min_y) / self.resolution)))

    def cell_to_world(self, cell: Cell) -> Point2:
        return (self.min_x + cell[0] * self.resolution,
                self.min_y + cell[1] * self.resolution)

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.occupied

    def segment_is_free(self, start: Cell, end: Cell) -> bool:
        x0, y0 = self.cell_to_world(start)
        x1, y1 = self.cell_to_world(end)
        return self.world_segment_is_free((x0, y0), (x1, y1))

    def world_segment_is_free(self, start: Point2, end: Point2) -> bool:
        """Check the actual world-space segment without snapping its ends twice."""
        x0, y0 = start
        x1, y1 = end
        distance = math.hypot(x1 - x0, y1 - y0)
        sample_count = max(1, int(math.ceil(distance / (0.5 * self.resolution))))
        for i in range(sample_count + 1):
            ratio = i / sample_count
            cell = self.world_to_cell(x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0))
            if not self.is_free(cell):
                return False
        return True


def astar(grid: OccupancyGrid2D, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if not grid.is_free(start) or not grid.is_free(goal):
        return None
    if start == goal:
        return [start]

    neighbours = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    queue = [(0.0, 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    closed = set()

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        for dx, dy, step_cost in neighbours:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.is_free(nxt):
                continue
            # Forbid diagonal corner-cutting between two occupied wall cells.
            if dx and dy:
                if not grid.is_free((current[0] + dx, current[1])):
                    continue
                if not grid.is_free((current[0], current[1] + dy)):
                    continue
            candidate_cost = current_cost + step_cost
            if candidate_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = candidate_cost
            parent[nxt] = current
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(queue, (candidate_cost + heuristic, candidate_cost, nxt))
    return None


def simplify_path(grid: OccupancyGrid2D, path: Sequence[Cell]) -> List[Cell]:
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        farthest = anchor + 1
        for candidate in range(anchor + 2, len(path)):
            if grid.segment_is_free(path[anchor], path[candidate]):
                farthest = candidate
            else:
                break
        simplified.append(path[farthest])
        anchor = farthest
    return simplified


def resample(points: Sequence[Point2], spacing: float) -> List[Point2]:
    if len(points) < 2:
        return list(points)
    result = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        sample_count = max(1, int(math.ceil(distance / spacing)))
        for i in range(1, sample_count + 1):
            ratio = i / sample_count
            result.append((start[0] + ratio * (end[0] - start[0]),
                           start[1] + ratio * (end[1] - start[1])))
    return result


class GlobalAStarPlanner(Node):
    def __init__(self):
        super().__init__("global_astar_planner")
        self.resolution = self.declare_parameter("resolution", 0.20).value
        self.inflation_radius = self.declare_parameter("inflation_radius", 0.45).value
        self.obstacle_min_z = self.declare_parameter("obstacle_min_z", 0.08).value
        self.obstacle_max_z = self.declare_parameter("obstacle_max_z", 0.85).value
        self.padding = self.declare_parameter("map_padding", 1.0).value
        self.path_spacing = self.declare_parameter("path_spacing", 0.50).value
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.grid: Optional[OccupancyGrid2D] = None
        self.position: Optional[Point2] = None

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(PointCloud2, "global_map", self.map_callback, map_qos)
        self.create_subscription(Odometry, "body_pose", self.odom_callback, 10)
        self.create_subscription(PoseStamped, "goal", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "initial_path", 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, "global_planning/status", status_qos)

    def publish_status(self, value: str):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def map_callback(self, msg: PointCloud2):
        if self.grid is not None:
            return
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((float(point[0]), float(point[1]), float(point[2])))
        try:
            self.grid = OccupancyGrid2D(
                points, float(self.resolution), float(self.inflation_radius),
                float(self.obstacle_min_z), float(self.obstacle_max_z), float(self.padding))
        except ValueError as error:
            self.get_logger().error(str(error))
            self.publish_status("MAP_ERROR")
            return
        self.get_logger().info(
            f"Global grid ready: {self.grid.width}x{self.grid.height}, "
            f"resolution={self.grid.resolution:.2f} m, occupied={len(self.grid.occupied)}")
        self.publish_status("READY")

    def odom_callback(self, msg: Odometry):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_callback(self, msg: PoseStamped):
        if self.grid is None:
            self.get_logger().warning("Reject goal: global map is not ready")
            self.publish_status("MAP_NOT_READY")
            return
        if self.position is None:
            self.get_logger().warning("Reject goal: odometry is not ready")
            self.publish_status("ODOM_NOT_READY")
            return

        start = self.grid.world_to_cell(*self.position)
        goal_xy = (msg.pose.position.x, msg.pose.position.y)
        goal = self.grid.world_to_cell(*goal_xy)
        if not self.grid.is_free(start):
            self.get_logger().error(f"Reject goal: start {self.position} is occupied or outside the map")
            self.publish_status("START_OCCUPIED")
            return
        if not self.grid.is_free(goal):
            self.get_logger().error(f"Reject goal: goal {goal_xy} is occupied or outside the map")
            self.publish_status("GOAL_OCCUPIED")
            return

        self.publish_status("PLANNING")
        raw_path = astar(self.grid, start, goal)
        if raw_path is None:
            self.get_logger().error(
                f"NO_PATH from ({self.position[0]:.2f}, {self.position[1]:.2f}) "
                f"to ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
            self.publish_status("NO_PATH")
            return

        cells = simplify_path(self.grid, raw_path)
        world_points = [self.grid.cell_to_world(cell) for cell in cells]
        world_points[0] = self.position
        world_points[-1] = goal_xy
        world_points = resample(world_points, float(self.path_spacing))

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        for index, (x, y) in enumerate(world_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            if index + 1 < len(world_points):
                nx, ny = world_points[index + 1]
            else:
                nx, ny = world_points[index - 1]
            yaw = math.atan2(ny - y, nx - x)
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path_msg.poses.append(pose)

        # Validate the exact simplified/resampled route against the same inflated map.
        for first, second in zip(world_points[:-1], world_points[1:]):
            if not self.grid.world_segment_is_free(first, second):
                self.get_logger().error(
                    "Internal safety check rejected the generated path segment "
                    f"({first[0]:.2f}, {first[1]:.2f}) -> "
                    f"({second[0]:.2f}, {second[1]:.2f})")
                self.publish_status("PATH_VALIDATION_FAILED")
                return

        self.path_pub.publish(path_msg)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(world_points[:-1], world_points[1:]))
        self.get_logger().info(
            f"Published collision-free initial_path: {len(world_points)} poses, {length:.2f} m")
        self.publish_status("PATH_PUBLISHED")


def main(args=None):
    rclpy.init(args=args)
    node = GlobalAStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
