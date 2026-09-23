#!/usr/bin/env python3
"""LocalMapPatch to persistent sparse TopoGraphDelta pipeline."""

import json
import math
import time
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import (
    LocalMapPatch, TopoEdge, TopoGraphDelta, TopoNode)
from std_msgs.msg import String

from explorer_core.topology import (
    TopologyEdge, TopologyGraph, TopologyNode, clearance_field,
    extract_topology, oracle_metrics)
from explorer_core.safe_region_graph import (
    PersistentSafeRegionTracker, ShadowStabilityTracker,
    extract_safe_region_graph,
    safe_region_oracle_metrics)
from explorer_core.topology_persistence import (
    ExplicitBlockageTracker, node_world_xy, occupancy_at_world)


class GlobalRepresentationNode(Node):
    """Maintain and publish the sparse global graph used for route estimates."""

    def __init__(self):
        super().__init__("global_representation")
        self.max_oracle_queries = int(
            self.declare_parameter("max_oracle_queries", 8).value)
        self.snapshot_period_revisions = max(1, int(
            self.declare_parameter("snapshot_period_revisions", 10).value))
        self.safe_region_shadow_enabled = bool(self.declare_parameter(
            "safe_region_shadow_enabled", True).value)
        self.safe_region_tile_size = float(self.declare_parameter(
            "safe_region_tile_size", 2.0).value)
        self.portal_bottleneck_width = float(self.declare_parameter(
            "portal_bottleneck_width", 1.2).value)
        self.safe_region_update_period_revisions = max(1, int(
            self.declare_parameter(
                "safe_region_update_period_revisions", 10).value))
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
        self.node_blockage = ExplicitBlockageTracker(
            self.blocked_confirmation_patches)
        self.edge_blockage = ExplicitBlockageTracker(
            self.blocked_confirmation_patches)
        self.graph_revision = 0
        self.last_map_revision = 0

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            LocalMapPatch, "grid_map/local_map_patch",
            self.patch_callback, sensor_qos)
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
        self.metrics_pub = self.create_publisher(
            String, "global_representation/oracle_metrics", 10)
        self.get_logger().info(
            "Sparse global representation enabled for route estimation")

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

    def patch_callback(self, patch: LocalMapPatch):
        if patch.map_revision <= self.last_map_revision:
            return
        expected = int(patch.width) * int(patch.height)
        if expected == 0 or len(patch.occupancy) != expected:
            self.get_logger().warning(
                f"Rejected malformed LocalMapPatch revision={patch.map_revision}")
            return

        started = time.perf_counter()
        occupancy = np.asarray(patch.occupancy, dtype=np.int8).reshape(
            (patch.height, patch.width))
        dense_free = int(np.count_nonzero(occupancy == 0))
        destructive_updates_allowed = (
            dense_free >= self.min_free_cells_for_destructive_update)
        shared_clearance = clearance_field(
            occupancy == 0, float(patch.resolution))
        local_graph = extract_topology(
            occupancy, float(patch.resolution),
            (patch.origin.x, patch.origin.y), shared_clearance)
        metrics = oracle_metrics(
            occupancy, float(patch.resolution), local_graph,
            self.max_oracle_queries)
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
            safe_snapshot = {
                "map_revision": int(patch.map_revision),
                "regions": list(self.global_safe_regions.values()),
                "portals": [item for item in self.global_safe_portals.values()
                            if item["source"] in self.global_safe_regions
                            and item["target"] in self.global_safe_regions],
            }
            safe_msg = String()
            safe_msg.data = json.dumps(safe_snapshot, separators=(",", ":"))
            self.safe_region_pub.publish(safe_msg)
        elif self.safe_region_shadow_enabled and self.last_safe_shadow_metrics:
            safe_metrics = dict(self.last_safe_shadow_metrics)
            safe_metrics.update({
                "safe_region_shadow_sampled": False,
                "safe_region_extraction_ms": 0.0,
                "safe_region_oracle_sampled": False,
                "safe_region_oracle_ms": 0.0,
            })
            metrics.update(safe_metrics)

        min_x, min_y = patch.origin.x, patch.origin.y
        max_x = min_x + patch.width * patch.resolution
        max_y = min_y + patch.height * patch.resolution

        def node_id_inside(node_id: int) -> bool:
            world_x, world_y = node_world_xy(
                node_id, float(patch.resolution))
            return min_x <= world_x < max_x and min_y <= world_y < max_y

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
        self.graph_revision += 1
        self.last_map_revision = patch.map_revision

        delta = TopoGraphDelta()
        delta.header = patch.header
        delta.source_map_revision = patch.map_revision
        delta.graph_revision = self.graph_revision
        delta.added_nodes = [local_node_messages[node_id]
                             for node_id in added_nodes]
        delta.updated_nodes = [local_node_messages[node_id]
                               for node_id in updated_nodes]
        delta.removed_node_ids = sorted(removed_node_ids)
        delta.added_edges = [local_edge_messages[edge_id]
                             for edge_id in added_edges]
        delta.updated_edges = [local_edge_messages[edge_id]
                               for edge_id in updated_edges]
        delta.removed_edge_ids = sorted(removed_edge_ids)
        self.delta_pub.publish(delta)
        if (self.graph_revision == 1
                or self.graph_revision % self.snapshot_period_revisions == 0):
            snapshot = TopoGraphDelta()
            snapshot.header = patch.header
            snapshot.source_map_revision = patch.map_revision
            snapshot.graph_revision = self.graph_revision
            snapshot.added_nodes = list(self.global_node_messages.values())
            snapshot.added_edges = list(self.global_edge_messages.values())
            self.snapshot_pub.publish(snapshot)

        processing_ms = (time.perf_counter() - started) * 1000.0
        metrics.update({
            "map_revision": int(patch.map_revision),
            "graph_revision": self.graph_revision,
            "patch_generation_ms": float(patch.generation_ms),
            "processing_ms": processing_ms,
            "dense_free_cells": dense_free,
            "skeleton_cells": len(local_graph.skeleton_cells),
            "local_nodes": len(local_graph.nodes),
            "local_edges": len(local_graph.edges),
            "global_nodes": len(self.global_nodes),
            "global_edges": len(self.global_edges),
            "destructive_updates_allowed": destructive_updates_allowed,
            "pending_node_blockages": self.node_blockage.pending,
            "pending_edge_blockages": self.edge_blockage.pending,
            "node_compression_ratio": (
                1.0 - len(local_graph.nodes) / dense_free if dense_free else 0.0),
        })
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics, separators=(",", ":"))
        self.metrics_pub.publish(metrics_msg)
        if not destructive_updates_allowed:
            self.get_logger().warning(
                f"[LOCAL_PATCH_INVALID] map_revision={patch.map_revision} "
                f"dense_free={dense_free}; preserving persistent topology",
                throttle_duration_sec=2.0)
        self.get_logger().info(
            f"[TOPOLOGY_SHADOW] map_revision={patch.map_revision} "
            f"dense={dense_free} nodes={len(local_graph.nodes)} "
            f"edges={len(local_graph.edges)} recall="
            f"{metrics['connectivity_recall']:.3f} cost_error="
            f"{metrics['mean_cost_error']:.3f} processing={processing_ms:.1f}ms",
            throttle_duration_sec=2.0)
        if sample_safe_region:
            self.get_logger().info(
                f"[SAFE_REGION_SHADOW] map_revision={patch.map_revision} "
                f"regions={len(safe_graph.regions)} "
                f"portals={len(safe_graph.portals)} "
                f"bottlenecks={safe_metrics['bottleneck_portals']} "
                f"coverage={safe_metrics['safe_region_coverage']:.3f} "
                f"recall={safe_metrics['safe_region_connectivity_recall']:.3f} "
                f"false_positive="
                f"{safe_metrics['safe_region_false_positive_rate']:.3f} "
                f"region_retention="
                f"{safe_metrics['safe_region_id_retention']:.3f} "
                f"portal_retention={safe_metrics['portal_id_retention']:.3f} "
                f"extraction={safe_region_extraction_ms:.1f}ms",
                throttle_duration_sec=2.0)


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
