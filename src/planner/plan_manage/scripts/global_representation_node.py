#!/usr/bin/env python3
"""Shadow-mode LocalMapPatch to persistent TopoGraphDelta pipeline."""

import json
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
    TopologyEdge, TopologyGraph, TopologyNode, extract_topology, oracle_metrics)


class GlobalRepresentationNode(Node):
    """Maintain a shadow global graph without influencing robot control."""

    def __init__(self):
        super().__init__("global_representation")
        self.max_oracle_queries = int(
            self.declare_parameter("max_oracle_queries", 8).value)
        self.global_nodes: Dict[int, TopologyNode] = {}
        self.global_edges: Dict[int, TopologyEdge] = {}
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
        self.metrics_pub = self.create_publisher(
            String, "global_representation/oracle_metrics", 10)
        self.get_logger().info(
            "Shadow global representation enabled; it does not control planning")

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
        local_graph = extract_topology(
            occupancy, float(patch.resolution),
            (patch.origin.x, patch.origin.y))
        metrics = oracle_metrics(
            occupancy, float(patch.resolution), local_graph,
            self.max_oracle_queries)

        min_x, min_y = patch.origin.x, patch.origin.y
        max_x = min_x + patch.width * patch.resolution
        max_y = min_y + patch.height * patch.resolution

        def node_id_inside(node_id: int) -> bool:
            x = (node_id >> 32) & 0xFFFFFFFF
            y = node_id & 0xFFFFFFFF
            x = x - (1 << 32) if x & (1 << 31) else x
            y = y - (1 << 32) if y & (1 << 31) else y
            world_x = x * patch.resolution
            world_y = y * patch.resolution
            return min_x <= world_x < max_x and min_y <= world_y < max_y

        removed_node_ids = {
            node_id for node_id in self.global_nodes
            if node_id_inside(node_id) and node_id not in local_graph.nodes}
        removed_edge_ids = {
            edge_id for edge_id, edge in self.global_edges.items()
            if edge_id not in local_graph.edges and
            (edge.source_id in removed_node_ids or
             edge.target_id in removed_node_ids or
             (node_id_inside(edge.source_id) and
              node_id_inside(edge.target_id)))}
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
        for node_id in removed_node_ids:
            self.global_nodes.pop(node_id, None)
        self.global_nodes.update(local_graph.nodes)
        self.global_edges.update(local_graph.edges)
        self.graph_revision += 1
        self.last_map_revision = patch.map_revision

        delta = TopoGraphDelta()
        delta.header = patch.header
        delta.source_map_revision = patch.map_revision
        delta.graph_revision = self.graph_revision
        delta.added_nodes = [
            self.node_message(node, patch) for node in added_nodes.values()]
        delta.updated_nodes = [
            self.node_message(node, patch) for node in updated_nodes.values()]
        delta.removed_node_ids = sorted(removed_node_ids)
        delta.added_edges = [
            self.edge_message(edge, patch) for edge in added_edges.values()]
        delta.updated_edges = [
            self.edge_message(edge, patch) for edge in updated_edges.values()]
        delta.removed_edge_ids = sorted(removed_edge_ids)
        self.delta_pub.publish(delta)

        processing_ms = (time.perf_counter() - started) * 1000.0
        dense_free = int(np.count_nonzero(occupancy == 0))
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
            "node_compression_ratio": (
                1.0 - len(local_graph.nodes) / dense_free if dense_free else 0.0),
        })
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics, separators=(",", ":"))
        self.metrics_pub.publish(metrics_msg)
        self.get_logger().info(
            f"[TOPOLOGY_SHADOW] map_revision={patch.map_revision} "
            f"dense={dense_free} nodes={len(local_graph.nodes)} "
            f"edges={len(local_graph.edges)} recall="
            f"{metrics['connectivity_recall']:.3f} cost_error="
            f"{metrics['mean_cost_error']:.3f} processing={processing_ms:.1f}ms",
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
