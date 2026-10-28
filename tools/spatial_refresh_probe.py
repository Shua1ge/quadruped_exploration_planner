"""Isolated ROS regression: synthetic scans, no Gazebo and no actuation."""
import json
import os
import signal
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

test_domain = int(os.environ.get('SCAN_TEST_ROS_DOMAIN_ID', '83'))
if not 0 <= test_domain <= 232 or test_domain == 43:
    raise ValueError('Use an isolated test domain, not the normal exploration domain 43')
os.environ['ROS_DOMAIN_ID'] = str(test_domain)

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scan_planner_msgs.msg import LocalMapPatch, LocomotionState, TopoGraphDelta
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray

REPO = Path(__file__).resolve().parents[1]
WS = REPO.parent.parent
sys.path.insert(0, str(REPO / 'src/planner/plan_manage/scripts'))
from frontier_explorer import FrontierExplorer
from global_representation_node import GlobalRepresentationNode


class Probe(Node):
    def __init__(self):
        super().__init__('spatial_refresh_probe')
        self.start = time.monotonic()
        self.times = defaultdict(list)
        self.samples = {}
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose_pubs = [self.create_publisher(Odometry, name, 10)
                          for name in ('sensor_pose', 'body_pose', 'odom')]
        self.cloud_pub = self.create_publisher(PointCloud2, 'cloud', sensor_qos)
        self.health_pub = self.create_publisher(
            LocomotionState, '/robot/locomotion_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for typ, name in (
            (LocalMapPatch, 'grid_map/local_map_patch'),
            (PointCloud2, 'grid_map/occupancy'),
            (PointCloud2, 'grid_map/occupancy_inflate'),
            (PointCloud2, 'grid_map/body_obstacles'),
            (PointCloud2, 'grid_map/low_surface_returns'),
            (TopoGraphDelta, 'global_representation/topology_delta'),
            (String, 'global_representation/safe_region_snapshot'),
            (MarkerArray, 'global_representation/graph_markers')):
            qos = (QoSProfile(depth=1) if typ is PointCloud2 else sensor_qos)
            self.create_subscription(typ, name,
                lambda msg, topic=name: self.record(topic, msg), qos)
        self.create_timer(.01, self.pose)
        self.create_timer(.1, self.cloud)
        self.create_timer(.1, self.health)

    def record(self, topic, msg):
        if time.monotonic() - self.start >= 2.:
            self.times[topic].append(time.monotonic())
        self.samples[topic] = msg

    def pose(self):
        msg = Odometry(header=Header(stamp=self.get_clock().now().to_msg(), frame_id='world'))
        msg.pose.pose.position.z = .3
        msg.pose.pose.orientation.w = 1.
        for pub in self.pose_pubs:
            pub.publish(msg)

    def cloud(self):
        # Known rectangular room. A horizontal lidar hit is not ground evidence.
        angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
        radii = np.minimum(3. / (np.abs(np.cos(angles)) + 1e-12),
                           2. / (np.abs(np.sin(angles)) + 1e-12))
        points = np.column_stack((radii*np.cos(angles), radii*np.sin(angles), np.zeros(720)))
        # Low raw evidence remains recorded; it is not certified ground.
        points = np.vstack((points, [[1., 0., -.28]]))
        msg = point_cloud2.create_cloud_xyz32(
            Header(stamp=self.get_clock().now().to_msg(), frame_id='probe_lidar'), points)
        self.cloud_pub.publish(msg)

    def health(self):
        self.health_pub.publish(LocomotionState(
            stamp=self.get_clock().now().to_msg(), epoch=1,
            mode=LocomotionState.MODE_RL_EFFORT, state=LocomotionState.STATE_STAND,
            actuation_ready=True, perception_pose_valid=True, reason='SYNTHETIC_TEST'))


def main():
    rclpy.init(args=['--ros-args', '-p', 'auto_start:=false', '-p', 'cloud_frame_id:=probe_lidar',
                    '-p', 'cloud_is_world:=false', '-p', 'obstacle_z_relative_to_body:=true',
                    '-p', 'obstacle_min_z:=-0.01',
                    '-p', 'inflation_radius:=0.25', '-p', 'footprint_radius:=0.25'])
    output_path = Path('/tmp/spatial_refresh_local_runtime.log')
    with output_path.open('w') as log:
        command = [str(WS / 'install/scan_planner/lib/scan_planner/scan_planner_node'),
            '--ros-args', '--params-file', str(REPO/'src/planner/plan_manage/config/planner.yaml'),
            '-p', 'grid_map.need_extrinsic:=false', '-p', 'grid_map.cloud_is_world:=false']
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        explorer, graph, probe = FrontierExplorer(), GlobalRepresentationNode(), Probe()
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (explorer, graph, probe):
            executor.add_node(node)
        try:
            deadline = time.monotonic() + 9.
            while time.monotonic() < deadline and child.poll() is None:
                executor.spin_once(timeout_sec=.02)
            assert child.poll() is None, f'Local node exited: inspect {output_path}'
            summary = {}
            for topic in sorted(probe.times):
                stamps = probe.times[topic]
                intervals = np.diff(stamps)
                summary[topic] = {'messages': len(stamps),
                    'median_interval_s': round(float(np.median(intervals)), 4),
                    'max_interval_s': round(float(np.max(intervals)), 4)}
            summary['explorer_fused_scans'] = explorer.map_update_count
            summary['inflation_m'] = explorer.inflation_radius
            summary['latest_patch_free_cells'] = sum(v == 0 for v in
                probe.samples['grid_map/local_map_patch'].occupancy)
            summary['graph_markers'] = [len(m.points) for m in
                probe.samples['global_representation/graph_markers'].markers]
            body = list(point_cloud2.read_points(
                probe.samples['grid_map/body_obstacles'], field_names=('x', 'y', 'z')))
            low = list(point_cloud2.read_points(
                probe.samples['grid_map/low_surface_returns'], field_names=('x', 'y', 'z')))
            summary['body_display_points'] = len(body)
            summary['low_raw_returns'] = len(low)
            print(json.dumps(summary, indent=2))
            for topic in ('grid_map/local_map_patch', 'grid_map/occupancy',
                          'grid_map/occupancy_inflate', 'global_representation/topology_delta',
                          'grid_map/body_obstacles', 'grid_map/low_surface_returns',
                          'global_representation/safe_region_snapshot',
                          'global_representation/graph_markers'):
                assert summary[topic]['messages'] >= 20, (topic, summary)
                assert .23 < summary[topic]['median_interval_s'] < .27, (topic, summary)
            assert 31 <= explorer.map_update_count <= 38, summary
            assert summary['latest_patch_free_cells'] > 100, summary
            assert summary['graph_markers'][0] > 0 and summary['graph_markers'][1] > 0
            assert len(body) > 0 and len(low) > 0, summary
            assert min(float(p[2]) for p in body) >= .18 - 1e-6
            assert max(float(p[2]) for p in low) < .18
            print('PASS: 100 Hz poses do not starve RViz; 10 Hz scans feed the 0.25 s spatial chain.')
        finally:
            # Only the exact child process created by this test is signalled.
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
            executor.shutdown()
            for node in (probe, graph, explorer):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
