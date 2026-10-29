#!/usr/bin/env python3
"""LocalMapPatch to persistent sparse TopoGraphDelta pipeline."""

import json
import math
import time
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import (
    FailureEvidence, LocalMapPatch, TopoEdge, TopoGraphDelta, TopoNode)
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from explorer_core.topology import (
    TopologyEdge, TopologyGraph, TopologyNode, clearance_field,
    extract_topology, oracle_metrics, routing_oracle_metrics)
from explorer_core.safe_region_graph import (
    PersistentSafeRegionTracker, ShadowStabilityTracker,
    extract_safe_region_graph,
    safe_region_oracle_metrics)
from explorer_core.topology_persistence import (
    ExplicitBlockageTracker, node_world_xy, occupancy_at_world)
from explorer_core.terrain_graph import (
    STATE_NAMES, UNKNOWN, TerrainTileStore, experience_summary)
from explorer_core.wavefront_graph import IncrementalWavefrontGraph
from explorer_core.region_overlay import region_rgb


class GlobalRepresentationNode(Node):
    """Maintain and publish the sparse global graph used for route estimates."""

    def __init__(self):
        super().__init__("global_representation")
        self.max_oracle_queries = int(
            self.declare_parameter("max_oracle_queries", 8).value)
        self.query_attachment_radius = float(
            self.declare_parameter("sparse_attachment_radius", 3.0).value)
        self.spatial_engine = str(self.declare_parameter(
            "spatial_engine", "wavefront").value)
        if self.spatial_engine not in ("wavefront", "skeleton"):
            raise ValueError("spatial_engine must be wavefront or skeleton")
        self.wavefront_tile_size = float(self.declare_parameter(
            "wavefront_tile_size", 2.0).value)
        self.wavefront = IncrementalWavefrontGraph(
            self.wavefront_tile_size, self.query_attachment_radius)
        self.wavefront_compact_routes = bool(self.declare_parameter(
            "wavefront_compact_routes", True).value)
        self.traversal_anchor = None
        self.traversal_frame = None
        self.traversal_tf_buffer = Buffer()
        self.traversal_tf_listener = TransformListener(self.traversal_tf_buffer, self)
        self.traversal_transform_failures = 0
        self.traversal_transformed_samples = 0
        self.topology_oracle_period_revisions = max(1, int(self.declare_parameter(
            "topology_oracle_period_revisions", 10).value))
        self.topology_oracle_enabled = bool(self.declare_parameter(
            "topology_oracle_enabled", False).value)
        self.last_topology_oracle = {}
        self.snapshot_period_revisions = max(1, int(
            self.declare_parameter("snapshot_period_revisions", 10).value))
        self.safe_region_shadow_enabled = bool(self.declare_parameter(
            "safe_region_shadow_enabled", False).value)
        self.safe_region_tile_size = float(self.declare_parameter(
            "safe_region_tile_size", 2.0).value)
        self.portal_bottleneck_width = float(self.declare_parameter(
            "portal_bottleneck_width", 1.2).value)
        self.safe_region_update_period_revisions = max(1, int(
            self.declare_parameter(
                "safe_region_update_period_revisions", 1).value))
        self.safe_region_oracle_period_revisions = max(1, int(
            self.declare_parameter(
                "safe_region_oracle_period_revisions", 10).value))
        self.blocked_confirmation_patches = max(1, int(
            self.declare_parameter(
                "blocked_confirmation_patches", 3).value))
        self.min_free_cells_for_destructive_update = max(1, int(
            self.declare_parameter(
                "min_free_cells_for_destructive_update", 16).value))
        self.safe_region_identity = PersistentSafeRegionTracker()
        self.safe_region_stability = ShadowStabilityTracker()
        self.last_safe_oracle_metrics = {
            "safe_region_queries": 0.0,
            "safe_region_dense_connected": 0.0,
            "safe_region_connectivity_recall": 1.0,
            "safe_region_false_positive_rate": 0.0,
            "safe_region_mean_abs_cost_error": 0.0,
            "safe_region_max_abs_cost_error": 0.0,
            "safe_region_oracle_ms": 0.0,
            "safe_region_oracle_sampled": False,
        }
        self.last_safe_shadow_metrics = {}
        self.global_safe_regions = {}
        self.global_safe_portals = {}
        self.global_nodes: Dict[int, TopologyNode] = {}
        self.global_edges: Dict[int, TopologyEdge] = {}
        self.global_node_messages: Dict[int, TopoNode] = {}
        self.global_edge_messages: Dict[int, TopoEdge] = {}
        self.node_attribute_radius = float(self.declare_parameter(
            "terrain_node_radius", 0.50).value)
        if not math.isfinite(self.node_attribute_radius) or self.node_attribute_radius <= 0.0:
            raise ValueError("terrain_node_radius must be finite and positive")
        self.experience_half_life = float(self.declare_parameter(
            "experience_half_life_seconds", 300.0).value)
        if not math.isfinite(self.experience_half_life) or self.experience_half_life <= 0.0:
            raise ValueError("experience_half_life_seconds must be finite and positive")
        self.terrain = TerrainTileStore(
            tile_size=self.wavefront_tile_size,
            stale_scans=int(self.declare_parameter(
                "terrain_stale_scans", 60).value),
            drop_scans=int(self.declare_parameter(
                "terrain_drop_scans", 300).value),
            rough_slope=float(self.declare_parameter(
                "terrain_rough_slope", 0.20).value),
            rough_roughness=float(self.declare_parameter(
                "terrain_rough_roughness", 0.06).value))
        self.node_attribute_hash: Dict[int, tuple] = {}
        self.edge_attribute_hash: Dict[int, tuple] = {}
        self.terrain_reset_count = 0
        self.graph_changes = ({}, {}, set(), {}, {}, set())
        self.node_blockage = ExplicitBlockageTracker(
            self.blocked_confirmation_patches)
        self.edge_blockage = ExplicitBlockageTracker(
            self.blocked_confirmation_patches)
        self.graph_revision = 0
        self.last_map_revision = 0
        self.route_constraint_revision = 0
        self.viewpoint_constraint_radius = float(self.declare_parameter(
            "viewpoint_constraint_radius", 0.60).value)
        self.max_viewpoint_constraints = max(1, int(self.declare_parameter(
            "max_viewpoint_constraints", 128).value))
        if self.viewpoint_constraint_radius <= 0.0:
            raise ValueError("viewpoint_constraint_radius must be positive")
        self.viewpoint_constraints = {}
        self.failure_evidence_received = 0
        self.failure_evidence_forwarded = 0

        self.global_update_period = float(self.declare_parameter(
            "global_update_period", 0.5).value)
        if not math.isfinite(self.global_update_period) or self.global_update_period < 0.1:
            raise ValueError("global_update_period must be finite and >= 0.1 seconds")
        self.skip_unchanged_patches = bool(self.declare_parameter(
            "skip_unchanged_patches", True).value)
        self._pending_patch = None
        self.received_patches = self.replaced_pending_patches = self.unchanged_patches = 0
        self._last_patch_key = None
        self._traversal_cost_revision = 0
        self.create_timer(self.global_update_period, self.process_pending_patch)

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            LocalMapPatch, "grid_map/local_map_patch",
            self.queue_patch, sensor_qos)
        self.create_subscription(Odometry, "body_pose", self.traversal_callback, sensor_qos)
        self.delta_pub = self.create_publisher(
            TopoGraphDelta, "global_representation/topology_delta", 10)
        snapshot_qos = QoSProfile(depth=1)
        snapshot_qos.reliability = ReliabilityPolicy.RELIABLE
        snapshot_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.snapshot_pub = self.create_publisher(
            TopoGraphDelta, "global_representation/topology_snapshot",
            snapshot_qos)
        self.safe_region_pub = self.create_publisher(
            String, "global_representation/safe_region_snapshot", snapshot_qos)
        self.graph_vis_pub = self.create_publisher(
            MarkerArray, "global_representation/graph_markers", snapshot_qos)
        failure_qos = QoSProfile(depth=20)
        failure_qos.reliability = ReliabilityPolicy.RELIABLE
        failure_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            FailureEvidence, "planning/failure_evidence",
            self.failure_evidence_callback, failure_qos)
        self.failure_evidence_pub = self.create_publisher(
            FailureEvidence, "global_representation/failure_evidence",
            failure_qos)
        self.metrics_pub = self.create_publisher(
            String, "global_representation/oracle_metrics", 10)
        self.get_logger().info(
            f"[SPATIAL_ENGINE] engine={self.spatial_engine} "
            f"tile_size={self.wavefront.tile_size:.2f}m update_period={self.global_update_period:.2f}s "
            "raw connectivity; no Global body inflation; audit oracle disabled by default")

    @staticmethod
    def node_message(node: TopologyNode, patch: LocalMapPatch) -> TopoNode:
        msg = TopoNode()
        msg.id = node.node_id
        msg.position.x = patch.origin.x + (node.cell[0] + 0.5) * patch.resolution
        msg.position.y = patch.origin.y + (node.cell[1] + 0.5) * patch.resolution
        msg.position.z = patch.origin.z
        msg.degree = node.degree
        msg.clearance = node.clearance
        return msg

    def traversal_callback(self, message):
        """Measured travel only, with timestamp/frame/gap/displacement gates."""
        if self.spatial_engine != "wavefront" or self.wavefront.resolution is None:
            return
        stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        point = (message.pose.pose.position.x, message.pose.pose.position.y)
        body_z = message.pose.pose.position.z
        if (stamp <= 0 or not 0 <= now-stamp <= 500000000 or
                not all(math.isfinite(v) for v in point)):
            self.traversal_anchor = None
            self.wavefront.traversal_direction = None
            return
        if message.header.frame_id != getattr(self, "traversal_frame", None):
            buffer = getattr(self, "traversal_tf_buffer", None)
            try:
                if buffer is None or not message.header.frame_id or not self.traversal_frame:
                    raise TransformException("Missing traversal frame/TF buffer")
                transform = buffer.lookup_transform(self.traversal_frame,
                    message.header.frame_id, Time.from_msg(message.header.stamp))
                sample = PointStamped(header=message.header, point=message.pose.pose.position)
                transformed = do_transform_point(sample, transform)
                point = (transformed.point.x, transformed.point.y)
                body_z = transformed.point.z
                if not all(math.isfinite(v) for v in point):
                    raise TransformException("Non-finite traversal transform")
                self.traversal_transformed_samples = getattr(self, "traversal_transformed_samples", 0)+1
            except TransformException:
                self.traversal_transform_failures = getattr(self, "traversal_transform_failures", 0)+1
                self.traversal_anchor = None
                self.wavefront.traversal_direction = None
                return  # Never relabel an odom point or substitute latest TF.
        previous = getattr(self, "traversal_anchor", None)
        last_stamp = getattr(self, "traversal_last_stamp", 0)
        if stamp <= last_stamp:
            return
        self.traversal_last_stamp = stamp
        if previous is None:
            self.traversal_anchor = (stamp, point)
            self.traversal_body_z = body_z
            self.wavefront.traversal_direction = None
            return
        gap = (stamp-previous[0])*1e-9
        distance = math.dist(point, previous[1])
        if (last_stamp and stamp-last_stamp > 500000000) or gap > 2.0 or distance/gap > 2.0:
            self.traversal_anchor = (stamp, point)
            self.traversal_body_z = body_z
            self.wavefront.traversal_direction = None
            return  # missing samples / pose jump is not successful travel
        if distance < max(.10, self.wavefront.resolution*.5):
            return  # standing jitter must not rebuild the graph
        self.traversal_anchor = (stamp, point)
        dz = body_z-getattr(self, "traversal_body_z", body_z)
        self.traversal_body_z = body_z
        self.wavefront.record_traversal(
            previous[1], point, dz, stamp * 1e-9)
        self._traversal_cost_revision = getattr(self, "_traversal_cost_revision", 0) + 1

    @staticmethod
    def edge_message(edge: TopologyEdge, patch: LocalMapPatch) -> TopoEdge:
        msg = TopoEdge()
        msg.id = edge.edge_id
        msg.source_id = edge.source_id
        msg.target_id = edge.target_id
        msg.length = edge.length
        msg.minimum_clearance = edge.minimum_clearance
        for x, y in edge.cells:
            point = Point()
            point.x = patch.origin.x + (x + 0.5) * patch.resolution
            point.y = patch.origin.y + (y + 0.5) * patch.resolution
            point.z = patch.origin.z
            msg.polyline.append(point)
        return msg


    def reset_world(self, reason):
        """A Local restart is a new map epoch; stale graph facts must not survive."""
        self.wavefront = IncrementalWavefrontGraph(
            self.wavefront_tile_size, self.query_attachment_radius)
        self.terrain.clear()
        self.global_nodes.clear()
        self.global_edges.clear()
        self.global_node_messages.clear()
        self.global_edge_messages.clear()
        self.global_safe_regions.clear()
        self.global_safe_portals.clear()
        self.node_attribute_hash.clear()
        self.edge_attribute_hash.clear()
        self.last_safe_shadow_metrics = {}
        self.graph_changes = ({}, {}, set(), {}, {}, set())
        self.last_map_revision = 0
        self._last_patch_key = None
        self._published_spatial_revision = -1
        self._spatial_fill_marker = None
        self._force_graph_snapshot = True
        self._spatial_fill_marker = None
        self._last_metrics = {}
        self.traversal_anchor = None
        self.terrain_reset_count += 1
        self.get_logger().warning(f"[GLOBAL_WORLD_RESET] reason={reason}")

    @staticmethod
    def tile_occupancy_revision(raw, resolution, origin, key, tile_size):
        """Stable hash of the raw slice inside one world tile (None if outside)."""
        min_x = key[0] * tile_size
        min_y = key[1] * tile_size
        x0 = int(math.floor((min_x - origin[0]) / resolution))
        x1 = int(math.ceil((min_x + tile_size - origin[0]) / resolution))
        y0 = int(math.floor((min_y - origin[1]) / resolution))
        y1 = int(math.ceil((min_y + tile_size - origin[1]) / resolution))
        x0, x1 = max(0, x0), min(raw.shape[1], x1)
        y0, y1 = max(0, y0), min(raw.shape[0], y1)
        if x1 <= x0 or y1 <= y0:
            return None
        value = 1469598103934665603
        for item in raw[y0:y1, x0:x1].ravel():
            value ^= int(item) + 1
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
        return value

    def ingest_terrain(self, patch, raw):
        """Store tile summaries, then invalidate old terrain where geometry moved."""
        scan = int(patch.scan)
        self.terrain.observe_scan(scan)
        changed = self.terrain.update(scan, patch.terrain_tiles)
        origin = (float(patch.origin.x), float(patch.origin.y))
        resolution = float(patch.resolution)
        tile_size = self.wavefront_tile_size
        min_tx = math.floor(origin[0] / tile_size)
        max_tx = math.floor(
            (origin[0] + patch.width * resolution - 1e-9) / tile_size)
        min_ty = math.floor(origin[1] / tile_size)
        max_ty = math.floor(
            (origin[1] + patch.height * resolution - 1e-9) / tile_size)
        observed = 0
        for tx in range(min_tx, max_tx + 1):
            for ty in range(min_ty, max_ty + 1):
                key = (tx, ty)
                revision = self.tile_occupancy_revision(
                    raw, resolution, origin, key, tile_size)
                if revision is None:
                    continue
                self.terrain.observe_occupancy(key, revision, scan)
                observed += 1
        return changed, observed

    @staticmethod
    def _quantize(value, step):
        return (None if not math.isfinite(value)
                else int(round(float(value) / step)))

    def _attribute_digest(self, aggregate, state, traversability,
                          traversals, experience):
        return (int(state), int(round(traversability * 100.0)),
                int(traversals), int(round(experience * 20.0)),
                int(aggregate["rise_cells"]), int(aggregate["drop_cells"]),
                int(aggregate["overhead_cells"]),
                int(aggregate["measured_columns"]) // 10,
                self._quantize(aggregate["slope"], 0.05),
                self._quantize(aggregate["roughness"], 0.02))

    def _element_terrain(self, element_id, aggregate, now_seconds, hashes):
        traversals, experience = experience_summary(
            self.wavefront.experience_tiles, aggregate["tiles"], now_seconds,
            self.experience_half_life)
        state = int(aggregate["state"])
        traversability = self.terrain.traversability(
            aggregate, traversals, experience)
        digest = self._attribute_digest(
            aggregate, state, traversability, traversals, experience)
        changed = hashes.get(element_id) != digest
        hashes[element_id] = digest
        return state, traversability, traversals, experience, changed

    def apply_terrain_attributes(self, scan, now_seconds):
        """Refresh node/edge attributes; return elements whose cost materially changed."""
        changed_nodes, changed_edges = set(), set()
        for element_id, message in self.global_node_messages.items():
            x, y = float(message.position.x), float(message.position.y)
            radius = self.node_attribute_radius
            domain = (self.wavefront.spatial_view.domains.get(element_id)
                      if getattr(self, "wavefront_compact_routes", False) else None)
            points = ([((self.wavefront.nodes[k].cell[0]+.5)*self.wavefront.resolution,
                        (self.wavefront.nodes[k].cell[1]+.5)*self.wavefront.resolution)
                       for k in domain.members] if domain else
                      [(x, y), (x + radius, y), (x - radius, y),
                       (x, y + radius), (x, y - radius)])
            aggregate = self.terrain.aggregate(points)
            state, traversability, traversals, experience, changed = (
                self._element_terrain(
                    element_id, aggregate, now_seconds, self.node_attribute_hash))
            self._assign_terrain_message(
                message, aggregate, state, traversability, traversals, experience)
            if changed:
                changed_nodes.add(element_id)
        for element_id, message in self.global_edge_messages.items():
            points = [(float(point.x), float(point.y))
                      for point in message.polyline]
            if not points:
                endpoints = [
                    self.global_node_messages[node_id]
                    for node_id in (int(message.source_id), int(message.target_id))
                    if node_id in self.global_node_messages]
                points = [(item.position.x, item.position.y)
                          for item in endpoints]
            step = max(1, len(points) // 32)
            sampled = points[::step]
            if points and sampled[-1] != points[-1]:
                sampled.append(points[-1])
            aggregate = self.terrain.aggregate(sampled)
            state, traversability, traversals, experience, changed = (
                self._element_terrain(
                    element_id, aggregate, now_seconds, self.edge_attribute_hash))
            self._assign_terrain_message(
                message, aggregate, state, traversability, traversals, experience)
            if changed:
                changed_edges.add(element_id)
        for node_id in list(self.node_attribute_hash):
            if node_id not in self.global_node_messages:
                del self.node_attribute_hash[node_id]
        for edge_id in list(self.edge_attribute_hash):
            if edge_id not in self.global_edge_messages:
                del self.edge_attribute_hash[edge_id]
        return changed_nodes, changed_edges

    @staticmethod
    def _assign_terrain_message(message, aggregate, state, traversability,
                                traversals, experience):
        message.ground_min = float(aggregate["ground_min"])
        message.ground_median = float(aggregate["ground_median"])
        message.ground_max = float(aggregate["ground_max"])
        message.terrain_slope = float(aggregate["slope"])
        message.terrain_roughness = float(aggregate["roughness"])
        message.rise_cells = int(aggregate["rise_cells"])
        message.drop_cells = int(aggregate["drop_cells"])
        message.overhead_cells = int(aggregate["overhead_cells"])
        message.measured_columns = int(aggregate["measured_columns"])
        message.last_scan = int(aggregate["last_scan"])
        message.terrain_state = int(state)
        message.traversability = float(traversability)
        message.traversals = int(traversals)
        message.experience_score = float(experience)

    def _json_float(self, value):
        return float(value) if math.isfinite(float(value)) else None

    def _terrain_payload(self, aggregate, now_seconds):
        records = getattr(getattr(self, "wavefront", None),
                          "experience_tiles", {})
        traversals, experience = experience_summary(
            records, aggregate["tiles"], now_seconds,
            getattr(self, "experience_half_life", 300.0))
        traversability = self.terrain.traversability(
            aggregate, traversals, experience)
        return {
            "ground_min": self._json_float(aggregate["ground_min"]),
            "ground_median": self._json_float(aggregate["ground_median"]),
            "ground_max": self._json_float(aggregate["ground_max"]),
            "slope": self._json_float(aggregate["slope"]),
            "roughness": self._json_float(aggregate["roughness"]),
            "rise_cells": int(aggregate["rise_cells"]),
            "drop_cells": int(aggregate["drop_cells"]),
            "overhead_cells": int(aggregate["overhead_cells"]),
            "measured_columns": int(aggregate["measured_columns"]),
            "last_scan": int(aggregate["last_scan"]),
            "state": STATE_NAMES[int(aggregate["state"])],
            "tiles": len(aggregate["tiles"]),
            "traversals": int(traversals),
            "experience_score": round(float(experience), 3),
            "traversability": float(traversability),
        }

    def _now_seconds(self):
        try:
            return self.get_clock().now().nanoseconds * 1e-9
        except (AttributeError, RuntimeError):
            return 0.0

    def region_terrain(self, region_id):
        """Query: terrain/experience summary of one persistent safe region."""
        item = self.global_safe_regions.get(int(region_id))
        if item is None or getattr(self, "terrain", None) is None:
            return None
        radius = getattr(self, "node_attribute_radius", 0.5)
        aggregate = self.terrain.aggregate(
            [(item["x"], item["y"]), (item["x"] + radius, item["y"]),
             (item["x"] - radius, item["y"]), (item["x"], item["y"] + radius),
             (item["x"], item["y"] - radius)])
        return self._terrain_payload(aggregate, self._now_seconds())

    def portal_state(self, portal_id):
        """Query: terrain/experience summary of one safe-region portal."""
        item = self.global_safe_portals.get(int(portal_id))
        if item is None or getattr(self, "terrain", None) is None:
            return None
        source = self.global_safe_regions.get(int(item["source"]))
        target = self.global_safe_regions.get(int(item["target"]))
        if source is None or target is None:
            return None
        points = [(source["x"], source["y"]), (target["x"], target["y"]),
                  ((source["x"] + target["x"]) * 0.5,
                   (source["y"] + target["y"]) * 0.5)]
        aggregate = self.terrain.aggregate(points)
        return self._terrain_payload(aggregate, self._now_seconds())

    def route_cost(self, edge_id):
        """Query: route-cost multiplier Global assigned to one sparse edge."""
        message = self.global_edge_messages.get(int(edge_id))
        if message is None:
            return None
        return float(getattr(message, "traversability", 1.0))

    def nearest_safe_region(self, x: float, y: float):
        """Return the nearest persistent region within the attachment radius."""
        if not self.global_safe_regions:
            return None
        region_id, distance = min(
            ((int(region_id), math.hypot(
                float(item["x"]) - x, float(item["y"]) - y))
             for region_id, item in self.global_safe_regions.items()),
            key=lambda item: item[1])
        return (region_id if distance <= self.viewpoint_constraint_radius
                else None)

    def publish_graph_markers(self, header):
        """Show current graph facts; slice height is NOT a terrain estimate.

        Stable marker slots replace the previous contents, including an empty
        constraint list. Constraint points are execution evidence, not walls.
        """
        output = MarkerArray()
        for marker_id, name, kind, size, rgb in (
                (0, "spatial_nodes", Marker.SPHERE_LIST, 0.10, (0.05, 0.55, 0.95)),
                (1, "spatial_edges", Marker.LINE_LIST, 0.035, (0.05, 0.75, 0.60)),
                (2, "viewpoint_evidence", Marker.POINTS, 0.20, (1.0, 0.3, 0.05))):
            marker = Marker()
            marker.header = header
            marker.ns, marker.id = name, marker_id
            marker.type, marker.action = kind, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = size
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 0.9
            output.markers.append(marker)
        output.markers[0].points = [
            Point(x=message.position.x, y=message.position.y, z=0.10)
            for message in self.global_node_messages.values()]
        for edge in self.global_edge_messages.values():
            if (edge.source_id not in self.global_node_messages
                    or edge.target_id not in self.global_node_messages):
                continue
            for first, second in zip(edge.polyline, edge.polyline[1:]):
                output.markers[1].points.extend((
                    Point(x=first.x, y=first.y, z=0.10),
                    Point(x=second.x, y=second.y, z=0.10)))
        output.markers[2].points = [
            Point(x=item["x"], y=item["y"], z=0.10)
            for item in self.viewpoint_constraints.values()]
        # Stable spatial floor identity comes from this same geometry view,
        # never from Explorer frontier groups or its current active task.
        fill_revision = (self.wavefront.spatial_view.floor_revision
                         if getattr(self, "spatial_engine", None) == "wavefront"
                         and getattr(self, "wavefront_compact_routes", True) else -1)
        fill = getattr(self, "_spatial_fill_marker", None)
        if fill is None or getattr(self, "_spatial_fill_revision", None) != fill_revision:
            fill = Marker()
            fill.ns, fill.id = "spatial_domains", 3
            fill.type, fill.action = Marker.TRIANGLE_LIST, Marker.ADD
            fill.pose.orientation.w = 1.
            fill.color.a = 1.
            fill.scale.x = fill.scale.y = fill.scale.z = 1.
            if fill_revision >= 0:
                resolution = self.wavefront.resolution or 1.
                for domain in self.wavefront.spatial_view.domains.values():
                    rgb = region_rgb(domain.place_id)
                    color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=.30)
                    for x0, y0, x1, y1 in domain.rectangles:
                        a = Point(x=float(x0*resolution), y=float(y0*resolution), z=.04)
                        b = Point(x=float(x1*resolution), y=float(y0*resolution), z=.04)
                        c = Point(x=float(x1*resolution), y=float(y1*resolution), z=.04)
                        d = Point(x=float(x0*resolution), y=float(y1*resolution), z=.04)
                        fill.points.extend((a, b, c, a, c, d))
                        fill.colors.extend((color,)*6)
            self._spatial_fill_marker = fill
            self._spatial_fill_revision = fill_revision
        fill.header = header
        output.markers.append(fill)  # empty ADD also clears an obsolete fill
        self.graph_vis_pub.publish(output)

    def safe_region_snapshot(self, map_revision=None):
        """Build the graph snapshot together with execution-derived overlays."""
        region_terrain, portal_state, route_cost = {}, {}, {}
        for region_id in self.global_safe_regions:
            payload = self.region_terrain(region_id)
            if payload is not None:
                region_terrain[str(region_id)] = payload
        for portal_id, item in self.global_safe_portals.items():
            payload = self.portal_state(portal_id)
            if payload is None:
                continue
            portal_state[str(portal_id)] = payload
            route_cost[str(portal_id)] = payload["traversability"]
        return {
            "map_revision": int(
                self.last_map_revision if map_revision is None
                else map_revision),
            "route_constraint_revision": int(self.route_constraint_revision),
            "regions": list(self.global_safe_regions.values()),
            "portals": [
                item for item in self.global_safe_portals.values()
                if item["source"] in self.global_safe_regions
                and item["target"] in self.global_safe_regions],
            "spatial_supports": dict(
                self.wavefront.routing_view().supports,
                source_map_revision=int(self.last_map_revision if map_revision is None else map_revision),
                world_epoch=int(getattr(self, "terrain_reset_count", 0)))
                if getattr(self, "spatial_engine", None) == "wavefront"
                and getattr(self, "wavefront_compact_routes", True) else {},
            "region_terrain": region_terrain,
            "portal_state": portal_state,
            "route_cost": route_cost,
            "viewpoint_constraints": list(
                self.viewpoint_constraints.values()),
        }

    def publish_safe_region_snapshot(self, map_revision=None):
        msg = String()
        msg.data = json.dumps(
            self.safe_region_snapshot(map_revision), separators=(",", ":"))
        self.safe_region_pub.publish(msg)

    def failure_evidence_callback(self, evidence: FailureEvidence):
        """Persist execution-derived route evidence and forward it immediately.

        Occupancy/topology remains geometric.  A rejected viewpoint is stored
        as a graph-owned constraint overlay, so a later map refresh cannot
        silently erase execution knowledge about the same approach.
        """
        self.failure_evidence_received += 1
        position = evidence.position
        if (int(evidence.request_id) <= 0
                or not all(math.isfinite(value) for value in (
                    position.x, position.y, position.z,
                    evidence.required_clearance))):
            self.get_logger().warning(
                "[FAILURE_EVIDENCE_REJECTED] non-finite or unscoped evidence")
            return

        failure_scope = int(evidence.failure_scope)
        if failure_scope == FailureEvidence.SCOPE_PATH:
            segment_start = getattr(evidence, "path_start", None)
            segment_end = getattr(evidence, "path_end", None)
            if (int(evidence.task_id) <= 0 or int(evidence.task_version) <= 0
                    or int(evidence.trajectory_id) <= 0
                    or not getattr(evidence, "has_path_segment", False)
                    or segment_start is None or segment_end is None
                    or not all(math.isfinite(value) for value in (
                        segment_start.x, segment_start.y, segment_start.z,
                        segment_end.x, segment_end.y, segment_end.z))):
                self.get_logger().warning(
                    "[FAILURE_EVIDENCE_REJECTED] path evidence lacks task lineage or segment")
                return

        if failure_scope == FailureEvidence.SCOPE_VIEWPOINT:
            request_id = int(evidence.request_id)
            constraint = {
                "request_id": request_id,
                "task_id": int(evidence.task_id),
                "task_version": int(evidence.task_version),
                "failed_object_id": int(evidence.failed_object_id),
                "trajectory_id": int(evidence.trajectory_id),
                "x": float(position.x),
                "y": float(position.y),
                "radius": float(self.viewpoint_constraint_radius),
                "region_id": self.nearest_safe_region(
                    float(position.x), float(position.y)),
                "stage": str(evidence.stage),
                "reason": str(evidence.reason),
                "source_map_revision": int(self.last_map_revision),
            }
            # Repeated path requests for one rejected task/viewpoint must not
            # create independent constraints or let request IDs define identity.
            object_key = ((int(evidence.task_id), int(evidence.task_version),
                           int(evidence.failed_object_id))
                          if evidence.task_id and evidence.failed_object_id
                          else request_id)
            previous = self.viewpoint_constraints.get(object_key)
            semantic_keys = tuple(key for key in constraint
                                  if key not in ("request_id", "trajectory_id",
                                                 "source_map_revision"))
            if (previous is None or any(previous.get(key) != constraint[key]
                                        for key in semantic_keys)):
                self.viewpoint_constraints[object_key] = constraint
                while (len(self.viewpoint_constraints)
                       > self.max_viewpoint_constraints):
                    oldest = next(iter(self.viewpoint_constraints))
                    del self.viewpoint_constraints[oldest]
                self.route_constraint_revision += 1
                self.publish_safe_region_snapshot()
                self.get_logger().warning(
                    "[GLOBAL_VIEWPOINT_CONSTRAINT] "
                    f"revision={self.route_constraint_revision} "
                    f"request_id={request_id} "
                    f"region_id={constraint['region_id']} "
                    f"position=({position.x:.2f},{position.y:.2f}) "
                    f"reason={evidence.reason}")

        self.failure_evidence_pub.publish(evidence)
        self.failure_evidence_forwarded += 1

    def queue_patch(self, patch):
        """Bound work to one latest patch; never accumulate a historical queue."""
        self.received_patches += 1
        if self._pending_patch is not None:
            self.replaced_pending_patches += 1
        self._pending_patch = patch

    def process_pending_patch(self):
        patch, self._pending_patch = self._pending_patch, None
        if patch is not None:
            self.patch_callback(patch)

    def patch_work_key(self, patch, raw):
        # Arrival/content counters and last_scan are freshness, not new terrain.
        # A bounded 2 s refresh still applies expiry and experience decay. Origin
        # XY is essential: the producer's content hash does not encode it.
        fields = ("tile_x", "tile_y", "tile_size", "ground_min", "ground_median",
                  "ground_max", "slope", "roughness", "support_cells", "rise_cells",
                  "drop_cells", "overhead_cells", "measured_columns")
        normalized = lambda value: value if math.isfinite(value) else None
        terrain = tuple(sorted(tuple(normalized(float(getattr(tile, name)))
                                     for name in fields) for tile in patch.terrain_tiles))
        return (float(patch.origin.x), float(patch.origin.y), float(patch.resolution),
                int(patch.width), int(patch.height), raw.tobytes(), terrain,
                getattr(self, "_traversal_cost_revision", 0),
                int(patch.header.stamp.sec) // 2)

    def publish_graph_snapshot(self, patch):
        if (self.graph_revision == 1 or getattr(self, "_force_graph_snapshot", False)
                or self.graph_revision % self.snapshot_period_revisions == 0):
            snapshot = TopoGraphDelta()
            snapshot.header = patch.header
            snapshot.source_map_revision = patch.map_revision
            snapshot.graph_revision = self.graph_revision
            snapshot.added_nodes = list(self.global_node_messages.values())
            snapshot.added_edges = list(self.global_edge_messages.values())
            self.snapshot_pub.publish(snapshot)
            self._force_graph_snapshot = False

    def patch_callback(self, patch: LocalMapPatch):
        expected = int(patch.width) * int(patch.height)
        if (expected == 0 or len(patch.occupancy) != expected
                or patch.header.frame_id != "world"
                or not math.isfinite(patch.resolution) or patch.resolution <= 0
                or not all(math.isfinite(v) for v in
                           (patch.origin.x, patch.origin.y, patch.origin.z))
                or any(v not in (-1, 0, 100) for v in patch.occupancy)):
            self.get_logger().warning(
                f"Rejected malformed LocalMapPatch revision={patch.map_revision}")
            return
        if (self.last_map_revision and 0 < patch.map_revision
                < self.last_map_revision):
            # Local restarted with a fresh sliding map; keep the graph epoch
            # monotonic for consumers but drop every stale world fact.
            self.reset_world(
                f"map_revision {self.last_map_revision}->{patch.map_revision}")
        if patch.map_revision <= self.last_map_revision:
            return

        started = time.perf_counter()
        raw_occupancy = np.asarray(patch.occupancy, dtype=np.int8).reshape(
            (patch.height, patch.width))
        raw_free = int(np.count_nonzero(raw_occupancy == 0))
        # Global preserves raw connection evidence. Body certification is Local.
        occupancy = raw_occupancy.copy()
        terrain_changed, terrain_tiles_observed = self.ingest_terrain(
            patch, raw_occupancy)
        work_key = self.patch_work_key(patch, raw_occupancy)
        if (getattr(self, "skip_unchanged_patches", False)
                and work_key == getattr(self, "_last_patch_key", None)):
            self.last_map_revision = patch.map_revision
            self.graph_revision += 1  # transport sequence; geometry stays cached
            self.unchanged_patches = getattr(self, "unchanged_patches", 0) + 1
            delta = TopoGraphDelta()
            delta.header = patch.header
            delta.source_map_revision = patch.map_revision
            delta.graph_revision = self.graph_revision
            self.delta_pub.publish(delta)
            self.publish_graph_snapshot(patch)
            if self.graph_revision % self.snapshot_period_revisions == 0:
                self.publish_safe_region_snapshot(patch.map_revision)
            metrics = dict(getattr(self, "_last_metrics", {}))
            metrics.update(map_revision=int(patch.map_revision),
                           graph_revision=self.graph_revision,
                           patch_scan=int(patch.scan),
                           patch_content_revision=int(patch.content_revision),
                           input_stamp=patch.header.stamp.sec + patch.header.stamp.nanosec * 1e-9,
                           unchanged_patch=True, unchanged_patches=self.unchanged_patches,
                           wavefront_changed_cells=0, wavefront_dirty_tiles=0,
                           topology_oracle_sampled=False, topology_oracle_ms=0.0,
                           received_patches=getattr(self, "received_patches", 0),
                           replaced_pending_patches=getattr(self, "replaced_pending_patches", 0),
                           processing_ms=(time.perf_counter() - started) * 1000.)
            for name in ("clearance_ms", "wavefront_ms", "graph_compression_ms",
                         "graph_delta_update_ms", "safe_region_extraction_ms", "safe_region_oracle_ms"):
                metrics[name] = 0.0
            metrics["safe_region_shadow_sampled"] = False
            metrics["safe_region_oracle_sampled"] = False
            message = String()
            message.data = json.dumps(metrics, separators=(",", ":"))
            self.metrics_pub.publish(message)
            return
        dense_free = int(np.count_nonzero(occupancy == 0))
        destructive_updates_allowed = (
            dense_free >= self.min_free_cells_for_destructive_update)
        clearance_started = time.perf_counter()
        shared_clearance = (clearance_field(occupancy == 0, float(patch.resolution))
                            if self.spatial_engine != "wavefront" or self.safe_region_shadow_enabled
                            else None)
        stage_timings = {
            "clearance_ms": (time.perf_counter() - clearance_started) * 1000.0}
        wave_update = None
        if self.spatial_engine == "wavefront":
            self.traversal_frame = patch.header.frame_id
            wave_started = time.perf_counter()
            try:
                wave_update = self.wavefront.update(
                    occupancy, float(patch.resolution), (patch.origin.x, patch.origin.y))
            except ValueError as error:
                self.get_logger().warning(f"[WAVEFRONT_PATCH_REJECTED] {error}")
                return
            local_graph = self.wavefront.local_view(
                occupancy.shape, (patch.origin.x, patch.origin.y))
            stage_timings.update(wave_update.metrics)
            stage_timings.update(wavefront_ms=(time.perf_counter()-wave_started)*1000.,
                                 thinning_ms=0., graph_compression_ms=0.)
            compression_started = time.perf_counter()
            query_graph = (self.wavefront.routing_view() if self.wavefront_compact_routes
                           else TopologyGraph(self.wavefront.nodes,self.wavefront.edges,set()))
            stage_timings['graph_compression_ms'] = (time.perf_counter()-compression_started)*1000.
            stage_timings.update(self.wavefront.spatial_view.metrics)
        else:
            local_graph = extract_topology(
                occupancy, float(patch.resolution),
                (patch.origin.x, patch.origin.y), shared_clearance, stage_timings)
        oracle_started = time.perf_counter()
        sample_topology_oracle = (getattr(self, "topology_oracle_enabled", True) and
            (not self.last_topology_oracle or
             (self.graph_revision+1) % self.topology_oracle_period_revisions == 0))
        if sample_topology_oracle:
            if wave_update is not None:
                # Compare against the same accumulated world facts as the
                # published graph, including routes outside the rolling patch.
                facts = self.wavefront.occupancy
                x0, y0 = min((c[0] for c in facts),default=0), min((c[1] for c in facts),default=0)
                x1, y1 = max((c[0] for c in facts),default=0), max((c[1] for c in facts),default=0)
                dense = np.full((y1-y0+1,x1-x0+1), -1, dtype=np.int8)
                for (x,y), value in facts.items():
                    dense[y-y0,x-x0] = value
                self.last_topology_oracle = routing_oracle_metrics(
                    dense, float(patch.resolution), query_graph, self.max_oracle_queries,
                    (x0*patch.resolution,y0*patch.resolution),
                    getattr(self,"query_attachment_radius",3.0))
            else:
                self.last_topology_oracle = oracle_metrics(
                    occupancy, float(patch.resolution), local_graph, self.max_oracle_queries)
            self.last_topology_oracle.update(oracle_graph_revision=self.graph_revision+1,
                                            oracle_source_map_revision=int(patch.map_revision))
        metrics = dict(self.last_topology_oracle)
        metrics['topology_oracle_sampled'] = sample_topology_oracle
        stage_timings["topology_oracle_ms"] = (
            time.perf_counter() - oracle_started) * 1000.0
        sample_safe_region = (
            self.safe_region_shadow_enabled
            and (self.graph_revision == 0
                 or (self.graph_revision + 1)
                 % self.safe_region_update_period_revisions == 0))
        if sample_safe_region:
            safe_region_started = time.perf_counter()
            safe_graph = extract_safe_region_graph(
                occupancy, float(patch.resolution),
                (patch.origin.x, patch.origin.y),
                self.safe_region_tile_size, self.portal_bottleneck_width,
                shared_clearance)
            safe_graph = self.safe_region_identity.update(
                safe_graph, (patch.origin.x, patch.origin.y),
                float(patch.resolution))
            safe_region_extraction_ms = (
                time.perf_counter() - safe_region_started) * 1000.0
            sample_oracle = (
                self.graph_revision == 0
                or (self.graph_revision + 1)
                % self.safe_region_oracle_period_revisions == 0)
            if sample_oracle:
                safe_oracle_started = time.perf_counter()
                self.last_safe_oracle_metrics = safe_region_oracle_metrics(
                    occupancy, float(patch.resolution), safe_graph,
                    self.max_oracle_queries)
                self.last_safe_oracle_metrics.update({
                    "safe_region_oracle_ms": (
                        time.perf_counter() - safe_oracle_started) * 1000.0,
                    "safe_region_oracle_sampled": True,
                })
            safe_metrics = dict(self.last_safe_oracle_metrics)
            if not sample_oracle:
                safe_metrics["safe_region_oracle_sampled"] = False
                safe_metrics["safe_region_oracle_ms"] = 0.0
            safe_metrics.update(self.safe_region_stability.update(safe_graph))
            safe_metrics.update({
                "safe_region_shadow_sampled": True,
                "safe_region_extraction_ms": safe_region_extraction_ms,
                "safe_regions": len(safe_graph.regions),
                "portals": len(safe_graph.portals),
                "bottleneck_portals": sum(
                    portal.bottleneck for portal in safe_graph.portals.values()),
                "safe_region_coverage": (
                    len(safe_graph.cell_to_region) / safe_graph.free_cell_count
                    if safe_graph.free_cell_count else 1.0),
                "safe_region_compression_ratio": (
                    1.0 - len(safe_graph.regions) / safe_graph.free_cell_count
                    if safe_graph.free_cell_count else 0.0),
            })
            self.last_safe_shadow_metrics = dict(safe_metrics)
            metrics.update(safe_metrics)
            patch_min_x = float(patch.origin.x)
            patch_min_y = float(patch.origin.y)
            patch_max_x = patch_min_x + patch.width * patch.resolution
            patch_max_y = patch_min_y + patch.height * patch.resolution
            current_safe_ids = set(safe_graph.regions)
            previously_local_ids = {
                region_id
                for region_id, item in self.global_safe_regions.items()
                if (patch_min_x <= item["x"] < patch_max_x
                    and patch_min_y <= item["y"] < patch_max_y)}
            touched_safe_ids = previously_local_ids | current_safe_ids
            for region_id in previously_local_ids - current_safe_ids:
                del self.global_safe_regions[region_id]
            # Rebuild every portal incident to the observed patch.  UNKNOWN
            # outside the patch remains non-destructive, while an explicitly
            # re-observed split/closure cannot leave a stale option edge.
            self.global_safe_portals = {
                portal_id: item
                for portal_id, item in self.global_safe_portals.items()
                if (item["source"] not in touched_safe_ids
                    and item["target"] not in touched_safe_ids)}
            for region in safe_graph.regions.values():
                x = patch.origin.x + (region.anchor[0] + 0.5) * patch.resolution
                y = patch.origin.y + (region.anchor[1] + 0.5) * patch.resolution
                self.global_safe_regions[int(region.region_id)] = {
                    "id": int(region.region_id), "x": x, "y": y,
                    "area": float(region.area)}
            for portal in safe_graph.portals.values():
                source = safe_graph.regions[portal.source_id].anchor
                target = safe_graph.regions[portal.target_id].anchor
                self.global_safe_portals[int(portal.portal_id)] = {
                    "id": int(portal.portal_id),
                    "source": int(portal.source_id),
                    "target": int(portal.target_id),
                    "length": max(float(patch.resolution), math.hypot(
                        target[0] - source[0], target[1] - source[1])
                        * float(patch.resolution)),
                    "width": float(portal.width),
                    "bottleneck": bool(portal.bottleneck)}
            self.publish_safe_region_snapshot(patch.map_revision)
        elif self.safe_region_shadow_enabled and self.last_safe_shadow_metrics:
            safe_metrics = dict(self.last_safe_shadow_metrics)
            safe_metrics.update({
                "safe_region_shadow_sampled": False,
                "safe_region_extraction_ms": 0.0,
                "safe_region_oracle_sampled": False,
                "safe_region_oracle_ms": 0.0,
            })
            metrics.update(safe_metrics)

        if wave_update is not None:
            compression_started = time.perf_counter()
            self.commit_wavefront_update(patch, wave_update)
            stage_timings.update(
                graph_delta_update_ms=(time.perf_counter()-compression_started)*1000.,
                wavefront_fine_nodes=len(self.wavefront.nodes),
                wavefront_fine_edges=len(self.wavefront.edges),
                wavefront_compacted_nodes=len(self.global_nodes),
                wavefront_compacted_edges=len(self.global_edges),
                wavefront_route_vertices=sum(len(e.cells) for e in self.global_edges.values()))
        else:
            self.commit_skeleton_update(patch, occupancy, local_graph, destructive_updates_allowed)
        self.graph_revision += 1
        self.last_map_revision = patch.map_revision
        stamp_seconds = (patch.header.stamp.sec
                         + patch.header.stamp.nanosec * 1e-9)
        terrain_nodes, terrain_edges = self.apply_terrain_attributes(
            int(patch.scan), stamp_seconds)

        delta = TopoGraphDelta()
        delta.header = patch.header
        delta.source_map_revision = patch.map_revision
        delta.graph_revision = self.graph_revision
        added_nodes, updated_nodes, removed_node_ids, added_edges, updated_edges, removed_edge_ids = self.graph_changes
        # Terrain/experience changes re-publish the same geometry at a new cost.
        updated_nodes = (set(updated_nodes)
                         | (terrain_nodes - set(added_nodes)
                            - set(removed_node_ids)))
        updated_edges = (set(updated_edges)
                         | (terrain_edges - set(added_edges)
                            - set(removed_edge_ids)))
        delta.added_nodes = [self.global_node_messages[k] for k in added_nodes]
        delta.updated_nodes = [self.global_node_messages[k] for k in updated_nodes]
        delta.removed_node_ids = sorted(removed_node_ids)
        delta.added_edges = [self.global_edge_messages[k] for k in added_edges]
        delta.updated_edges = [self.global_edge_messages[k] for k in updated_edges]
        delta.removed_edge_ids = sorted(removed_edge_ids)
        self.delta_pub.publish(delta)
        self.publish_graph_markers(patch.header)
        self.publish_graph_snapshot(patch)
        spatial_revision = self.wavefront.spatial_view.revision
        if (getattr(self, "_published_spatial_revision", -1) != spatial_revision
                or self.graph_revision % self.snapshot_period_revisions == 0):
            self.publish_safe_region_snapshot(patch.map_revision)
            self._published_spatial_revision = spatial_revision

        processing_ms = (time.perf_counter() - started) * 1000.0
        element_states = {}
        for message in list(self.global_node_messages.values()) + list(
                self.global_edge_messages.values()):
            name = STATE_NAMES[int(message.terrain_state)]
            element_states[name] = element_states.get(name, 0) + 1
        metrics.update({
            "map_revision": int(patch.map_revision),
            "graph_revision": self.graph_revision,
            "spatial_engine": self.spatial_engine,
            "patch_generation_ms": float(patch.generation_ms),
            "processing_ms": processing_ms,
            "dense_free_cells": dense_free,
            "skeleton_cells": len(local_graph.skeleton_cells),
            "local_nodes": len(local_graph.nodes),
            "local_edges": len(local_graph.edges),
            "global_nodes": len(self.global_nodes),
            "global_edges": len(self.global_edges),
            "traversal_transformed_samples": getattr(self, "traversal_transformed_samples", 0),
            "traversal_transform_failures": getattr(self, "traversal_transform_failures", 0),
            "destructive_updates_allowed": destructive_updates_allowed,
            "pending_node_blockages": self.node_blockage.pending,
            "pending_edge_blockages": self.edge_blockage.pending,
            "route_constraint_revision": self.route_constraint_revision,
            "viewpoint_constraints": len(self.viewpoint_constraints),
            "failure_evidence_received": self.failure_evidence_received,
            "failure_evidence_forwarded": self.failure_evidence_forwarded,
            "node_compression_ratio": (
                1.0 - len(local_graph.nodes) / dense_free if dense_free else 0.0),
            "raw_free_cells": raw_free,
            "patch_scan": int(patch.scan),
            "patch_content_revision": int(patch.content_revision),
            "patch_inflation_radius": 0.0,
            "geometry_input": "raw",
            "unchanged_patch": False,
            "unchanged_patches": getattr(self, "unchanged_patches", 0),
            "input_stamp": stamp_seconds,
            "received_patches": getattr(self, "received_patches", 0),
            "replaced_pending_patches": getattr(self, "replaced_pending_patches", 0),
            "terrain_tiles": len(self.terrain.tiles),
            "terrain_tiles_observed": terrain_tiles_observed,
            "terrain_changed_tiles": int(terrain_changed),
            "terrain_dropped_tiles": self.terrain.dropped_tiles,
            "terrain_revision": self.terrain.revision,
            "terrain_reset_count": self.terrain_reset_count,
            "terrain_element_states": element_states,
            "experience_half_life_seconds": self.experience_half_life,
        })
        metrics.update(stage_timings)
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics, separators=(",", ":"))
        self.metrics_pub.publish(metrics_msg)
        self._last_patch_key = work_key
        self._last_metrics = metrics
        self.get_logger().info(
            f"[SPATIAL_GRAPH_UPDATE] engine={self.spatial_engine} map_revision={patch.map_revision} "
            f"local=({len(local_graph.nodes)},{len(local_graph.edges)}) "
            f"global=({len(self.global_nodes)},{len(self.global_edges)}) "
            f"changed_cells={stage_timings.get('wavefront_changed_cells', 0)} "
            f"dirty_tiles={stage_timings.get('wavefront_dirty_tiles', 0)} "
            f"experience_revision={stage_timings.get('wavefront_experience_revision', 0)} "
            f"fine=({len(self.wavefront.nodes)},{len(self.wavefront.edges)}) "
            f"added=({len(added_nodes)},{len(added_edges)}) "
            f"updated=({len(updated_nodes)},{len(updated_edges)}) "
            f"removed=({len(removed_node_ids)},{len(removed_edge_ids)}) "
            f"terrain_tiles={len(self.terrain.tiles)} "
            f"terrain_changed={int(terrain_changed)} "
            f"content_revision={int(patch.content_revision)} "
            f"processing={processing_ms:.1f}ms",
            throttle_duration_sec=2.0)

    def commit_wavefront_update(self, patch, update):
        # The engine owns world-indexed geometry. Evidence stays in the graph's
        # separate spatial overlay, so split/merge never erases that experience.
        before_nodes, before_edges = self.global_nodes, self.global_edges
        view = (self.wavefront.routing_view() if getattr(self, "wavefront_compact_routes", True)
                else TopologyGraph(self.wavefront.nodes, self.wavefront.edges, set()))
        self.global_nodes, self.global_edges = dict(view.nodes), dict(view.edges)
        added_nodes = {k: n for k, n in view.nodes.items() if k not in before_nodes}
        updated_nodes = {k: n for k, n in view.nodes.items() if k in before_nodes and n != before_nodes[k]}
        added_edges = {k: e for k, e in view.edges.items() if k not in before_edges}
        updated_edges = {k: e for k, e in view.edges.items() if k in before_edges and e != before_edges[k]}
        removed_nodes = before_nodes.keys()-view.nodes.keys()
        removed_edges = before_edges.keys()-view.edges.keys()
        world_patch = LocalMapPatch(resolution=patch.resolution)
        world_patch.origin.z = patch.origin.z
        for key in removed_edges:
            self.global_edge_messages.pop(key, None)
        for key in removed_nodes:
            self.global_node_messages.pop(key, None)
        for key, node in {**added_nodes, **updated_nodes}.items():
            self.global_node_messages[key] = self.node_message(node, world_patch)
        for key, edge in {**added_edges, **updated_edges}.items():
            self.global_edge_messages[key] = self.edge_message(edge, world_patch)
        self.graph_changes = (added_nodes, updated_nodes, removed_nodes,
                              added_edges, updated_edges, removed_edges)

    def commit_skeleton_update(self, patch, occupancy, local_graph, destructive_updates_allowed):
        min_x, min_y = patch.origin.x, patch.origin.y
        max_x = min_x + patch.width * patch.resolution
        max_y = min_y + patch.height * patch.resolution
        def node_id_inside(node_id):
            x, y = node_world_xy(node_id, float(patch.resolution))
            return min_x <= x < max_x and min_y <= y < max_y
        patch_origin = (float(patch.origin.x), float(patch.origin.y))
        removed_node_ids = set()
        for node_id in self.global_nodes:
            if not node_id_inside(node_id):
                continue
            state = occupancy_at_world(
                occupancy, patch_origin, float(patch.resolution),
                node_world_xy(node_id, float(patch.resolution)))
            if self.node_blockage.observe(
                    node_id, node_id in local_graph.nodes, [state],
                    destructive_updates_allowed):
                removed_node_ids.add(node_id)

        removed_edge_ids = set()
        for edge_id, edge in self.global_edges.items():
            message = self.global_edge_messages.get(edge_id)
            states = [] if message is None else [
                occupancy_at_world(
                    occupancy, patch_origin, float(patch.resolution),
                    (point.x, point.y))
                for point in message.polyline]
            if (edge.source_id in removed_node_ids or
                    edge.target_id in removed_node_ids or
                    self.edge_blockage.observe(
                        edge_id, edge_id in local_graph.edges, states,
                        destructive_updates_allowed)):
                removed_edge_ids.add(edge_id)
        added_nodes = {
            node_id: node for node_id, node in local_graph.nodes.items()
            if node_id not in self.global_nodes}
        updated_nodes = {
            node_id: node for node_id, node in local_graph.nodes.items()
            if node_id in self.global_nodes and node != self.global_nodes[node_id]}
        added_edges = {
            edge_id: edge for edge_id, edge in local_graph.edges.items()
            if edge_id not in self.global_edges}
        updated_edges = {
            edge_id: edge for edge_id, edge in local_graph.edges.items()
            if edge_id in self.global_edges and edge != self.global_edges[edge_id]}

        for edge_id in removed_edge_ids:
            self.global_edges.pop(edge_id, None)
            self.global_edge_messages.pop(edge_id, None)
        for node_id in removed_node_ids:
            self.global_nodes.pop(node_id, None)
            self.global_node_messages.pop(node_id, None)
        self.node_blockage.forget(tuple(removed_node_ids))
        self.edge_blockage.forget(tuple(removed_edge_ids))
        self.global_nodes.update(local_graph.nodes)
        self.global_edges.update(local_graph.edges)
        local_node_messages = {
            node_id: self.node_message(node, patch)
            for node_id, node in local_graph.nodes.items()}
        local_edge_messages = {
            edge_id: self.edge_message(edge, patch)
            for edge_id, edge in local_graph.edges.items()}
        self.global_node_messages.update(local_node_messages)
        self.global_edge_messages.update(local_edge_messages)
        self.graph_changes = (added_nodes, updated_nodes, removed_node_ids,
                              added_edges, updated_edges, removed_edge_ids)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRepresentationNode()
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
