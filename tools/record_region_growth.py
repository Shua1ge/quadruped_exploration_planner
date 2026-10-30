#!/usr/bin/env python3
"""Read-only ROS2 recorder for actual Explorer Region/Goal transitions.

Record existing messages, without raycasting, planning, services, or publishing.
The bag is a real run trace; missing map history/provenance is never invented.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True

TOPICS = (
    ('/explorer/map', 'nav_msgs/msg/OccupancyGrid', True),
    ('/map_updates', 'map_msgs/msg/OccupancyGridUpdate', True),
    ('/explorer/frontiers', 'visualization_msgs/msg/Marker', True),
    ('/explorer/region_overlay', 'visualization_msgs/msg/MarkerArray', True),
    ('/explorer/region_sequence', 'std_msgs/msg/String', True),
    ('/explorer/goal', 'visualization_msgs/msg/Marker', True),
    ('/explorer/path', 'nav_msgs/msg/Path', True),
    ('/explorer/status', 'std_msgs/msg/String', False),
    ('/explorer/metrics', 'std_msgs/msg/String', False),
    ('/observation_task', 'scan_planner_msgs/msg/ObservationTask', False),
    ('/planning/observation_feedback', 'scan_planner_msgs/msg/ObservationFeedback', False),
    ('/planning/status', 'std_msgs/msg/String', False),
    ('/planning/trajectory_ack', 'scan_planner_msgs/msg/TrajectoryAck', False),
    ('/planning/local_execution_event', 'std_msgs/msg/String', False),
    ('/quad_0/body_pose', 'nav_msgs/msg/Odometry', False),
    ('/clock', 'rosgraph_msgs/msg/Clock', False),
)


def stamp_ns(value):
    return int(value.sec)*1_000_000_000+int(value.nanosec)


class BagSink:
    def __init__(self, path):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        self.serialize = serialize_message
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        for topic, msg_type, _ in TOPICS:
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=msg_type, serialization_format='cdr'))

    def append(self, topic, message, receipt_wall_ns):
        # Use receipt wall time for monotonic bag ordering across /clock resets;
        # original capture/header times remain unchanged inside each CDR message.
        self.writer.write(topic, self.serialize(message), receipt_wall_ns)

    def close(self):
        self.writer.close()


class RegionRecording:
    def __init__(self, output, domain, odom_rate=5., sink_factory=BagSink):
        self.output = Path(output).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.sink = sink_factory(self.output/'bag')
        self.events = (self.output/'events.jsonl').open('x', encoding='utf-8')
        self.received, self.written, self.skipped = Counter(), Counter(), Counter()
        self.clock_ns = 0
        self.clock_resets = 0
        self.last_sample = {}
        self.last_odom_stamp = None
        self.odom_period_ns = int(1e9/odom_rate)
        self.latest_region_ids = []
        self.latest_sequence = None
        self.latest_published_task = None
        self.map_info = None
        self.started_ns = time.time_ns()
        self.closed = False
        self.manifest = dict(schema_version=1, format='rosbag2_sqlite3_cdr_with_receipt_events',
            ros_domain_id=domain, started_wall_ns=self.started_ns,
            topics=[dict(name=t, type=m, transient_subscription=q) for t,m,q in TOPICS],
            configured_sampling=dict(body_pose_hz=odom_rate, clock_hz=2., other_topics='every received message'),
            raw_map_history_complete=False,
            raw_map_history_reason='Not provable: raw full map is published once; incremental topic has no sequence/history contract',
            measured_free_provenance_available=False,
            measured_free_reason='Existing public topics do not expose per-cell measured-free vs seed transitions',
            baseline_region_source='actual published information_regions marker IDs, not a reconstructed tracker',
            publication_vs_execution='ObservationTask means published intent; ACK/feedback record actual acceptance separately',
            bag_timestamp='receipt wall nanoseconds; original message capture/header time preserved',
            source_snapshot_schema_compatible=False,
            scope='Read-only subscribed run trace; not exact measured-growth replay input')
        self.flush()

    def append(self, topic, message, wall_ns=None, monotonic_ns=None):
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        mono = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        self.received[topic] += 1
        if topic == '/clock':
            value = stamp_ns(message.clock)
            if value < self.clock_ns:
                self.clock_resets += 1
            self.clock_ns = value
        interval = self.odom_period_ns if topic == '/quad_0/body_pose' else (
            500_000_000 if topic == '/clock' else 0)
        if interval and topic in self.last_sample and mono-self.last_sample[topic] < interval:
            self.skipped[topic+':rate'] += 1
            return False
        header = getattr(message, 'header', None)
        source_ns = stamp_ns(header.stamp) if header is not None else None
        if topic == '/quad_0/body_pose' and source_ns == self.last_odom_stamp:
            self.skipped[topic+':duplicate_stamp'] += 1
            return False
        self.sink.append(topic, message, wall_ns)
        self.written[topic] += 1
        self.last_sample[topic] = mono
        if topic == '/quad_0/body_pose':
            self.last_odom_stamp = source_ns
        payload = self.describe(topic, message)
        self.events.write(json.dumps(dict(topic=topic, receipt_wall_ns=wall_ns,
            receipt_monotonic_ns=mono, observed_clock_ns=self.clock_ns,
            message_header_stamp_ns=source_ns, payload=payload),ensure_ascii=False)+'\n')
        return True

    def describe(self, topic, message):
        if topic == '/explorer/map':
            info = message.info
            self.map_info = dict(width=info.width,height=info.height,resolution=info.resolution,
                origin=[info.origin.position.x,info.origin.position.y],
                frame_id=message.header.frame_id, header_stamp_ns=stamp_ns(message.header.stamp))
            return dict(self.map_info, cells=len(message.data), history_complete=False)
        if topic == '/map_updates':
            return dict(x=message.x,y=message.y,width=message.width,height=message.height,
                        cells=len(message.data), full_map_received=self.map_info is not None)
        if topic == '/explorer/region_overlay':
            regions = [dict(id=int(m.id),cells=len(m.points),color=[m.color.r,m.color.g,m.color.b,m.color.a])
                for m in message.markers if m.ns == 'information_regions' and m.action == 0]
            self.latest_region_ids = sorted(r['id'] for r in regions)
            return dict(regions=regions,markers=len(message.markers),
                observation='published snapshot, not fusion revision-synchronized')
        if topic == '/observation_task':
            data = {name:int(getattr(message,name)) for name in
                    ('task_id','task_version','request_id','lifecycle','region_id')}
            data.update(has_region=message.has_region,goal=[message.goal.x,message.goal.y],
                target_cell_count=message.target_cell_count,representatives=len(message.target_cells),
                route_points=len(message.coarse_route))
            self.latest_published_task = data
            return data
        if hasattr(message, 'data') and isinstance(message.data, str):
            try:
                data = json.loads(message.data)
            except (ValueError,TypeError):
                data = message.data
            if topic == '/explorer/region_sequence':
                self.latest_sequence = data
            return data
        if topic == '/quad_0/body_pose':
            p,v=message.pose.pose.position,message.twist.twist.linear
            return dict(position=[p.x,p.y,p.z],velocity=[v.x,v.y,v.z])
        if topic == '/explorer/frontiers':
            return dict(points=len(message.points),ns=message.ns,id=message.id,action=message.action)
        if topic == '/explorer/goal':
            p=message.pose.position
            return dict(position=[p.x,p.y,p.z],ns=message.ns,id=message.id,action=message.action)
        if topic == '/explorer/path':
            return dict(points=len(message.poses))
        if topic == '/clock':
            return dict(clock_ns=self.clock_ns)
        return {name:int(getattr(message,name)) for name in
                ('task_id','task_version','request_id','trajectory_id','status')
                if hasattr(message,name)}

    def flush(self):
        self.events.flush()
        value = dict(self.manifest, status='closed' if self.closed else 'recording',
            received=dict(self.received),written=dict(self.written),deliberately_skipped=dict(self.skipped),
            observed_clock_ns=self.clock_ns,clock_resets=self.clock_resets,
            map_info=self.map_info,latest_published_region_ids=self.latest_region_ids,
            latest_region_sequence=self.latest_sequence,latest_published_task=self.latest_published_task,
            updated_wall_ns=time.time_ns())
        (self.output/'session.json').write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')

    def close(self):
        if not self.closed:
            self.sink.close()
            self.closed = True
            self.flush()
            self.events.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing directory is rejected')
    parser.add_argument('--domain', type=int, help='Same ROS_DOMAIN_ID as Campus')
    parser.add_argument('--duration', type=float, default=0., help='Wall seconds; 0 until Ctrl+C')
    parser.add_argument('--odom-rate', type=float, default=5.)
    args = parser.parse_args(argv)
    if args.domain is not None and not 0 <= args.domain <= 232:
        parser.error('--domain must be in [0,232]')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not math.isfinite(args.odom_rate) or not 0 < args.odom_rate <= 50:
        parser.error('--odom-rate must be in (0,50]')
    if args.domain is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        parser.exit(2, f'先 source /opt/ros/humble/setup.bash 和 dyn/install/setup.bash：{error}\n')
    session = node = None
    rclpy.init(args=[])
    try:
        session = RegionRecording(args.output, os.environ.get('ROS_DOMAIN_ID','0'),args.odom_rate)
        node = Node('region_growth_readonly_recorder')
        for topic,msg_type,transient in TOPICS:
            qos = QoSProfile(depth=100 if topic == '/map_updates' else 20,
                reliability=ReliabilityPolicy.RELIABLE if transient else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
            node.create_subscription(get_message(msg_type), topic,
                lambda message, topic=topic: session.append(topic,message),qos)
        started = time.monotonic()
        def progress():
            session.flush()
            node.get_logger().info(f"recorded={sum(session.written.values())} "
                f"map_updates={session.written['/map_updates']} "
                f"region_frames={session.written['/explorer/region_overlay']} "
                f"published_regions={session.latest_region_ids}; output={session.output}")
            missing=[t for t in ('/explorer/map','/map_updates','/explorer/region_overlay')
                     if not session.received[t]]
            if missing:
                node.get_logger().warning(f'尚未收到 {missing}；核对ROS_DOMAIN_ID、节点和话题。')
        node.create_timer(5.,progress)
        node.get_logger().info(f"只读记录开始 domain={os.environ.get('ROS_DOMAIN_ID','0')}；Ctrl+C结束；不发控制命令。")
        node.get_logger().warning('晚加入时原始地图历史未必完整，公开话题缺少逐格实测FREE来源；已在session.json标记，不伪造延续证据。')
        while rclpy.ok() and (args.duration == 0 or time.monotonic()-started < args.duration):
            rclpy.spin_once(node,timeout_sec=.2)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if session is not None:
            session.close()
            print(f'记录完成：{session.output}',flush=True)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
