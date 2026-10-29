#!/usr/bin/env python3
"""Read-only GO2 odometry recorder. CSV is complete; /quad_0/path is RViz's tail.

python3 tools/record_actual_route.py --output /tmp/go_actual.csv --ros-args -r __ns:=/
Set ROS_DOMAIN_ID to the running simulation's domain before starting separately.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

FIELDS = ('stamp_ns', 'elapsed_s', 'wall_time_s', 'segment', 'frame_id',
          'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
          'vx', 'vy', 'vz', 'speed_mps', 'distance_xy_m', 'distance_3d_m')


class RouteWriter:
    """Bounded O(1) state; timestamps belong to measured odometry, not receipt."""
    def __init__(self, output, world='', topic='/quad_0/body_pose', rate=5.):
        self.output = Path(output).expanduser().resolve()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if self.output.with_suffix('.json').exists():
            raise FileExistsError(self.output.with_suffix('.json'))
        self.stream = self.output.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(FIELDS)
        self.stream.flush()
        self.count = 0
        self.last = None
        self.origin_ns = None
        self.elapsed = 0.
        self.segment = 0
        self.distance_xy = self.distance_3d = 0.
        self.metadata = dict(schema=1, source_topic=topic, world_file=world,
                             sample_rate_hz=rate, started_wall_time_s=time.time(),
                             coordinates='unmodified odometry pose (body reference, not ground)',
                             csv=str(self.output))
        self._metadata()

    def _metadata(self):
        self.output.with_suffix('.json').write_text(
            json.dumps(dict(self.metadata, samples=self.count,
                            distance_xy_m=self.distance_xy,
                            distance_3d_m=self.distance_3d), ensure_ascii=False, indent=2),
            encoding='utf-8')

    def append(self, msg, wall_time=None):
        pose, vel = msg.pose.pose, msg.twist.twist.linear
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        v = (vel.x, vel.y, vel.z)
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        frame = msg.header.frame_id
        if not frame or not all(math.isfinite(n) for n in (*xyz, *q, *v)):
            return False
        if self.last and ns == self.last[0]:
            return False  # paused simulator / duplicate callback
        if self.origin_ns is None:
            self.origin_ns = ns
        if self.last:
            old_ns, old_frame, old_xyz = self.last
            dt = (ns - old_ns) * 1e-9
            # Don't draw across a clock reset, changed coordinate frame or input outage.
            if dt < 0 or frame != old_frame or dt > 2.:
                self.segment += 1
            else:
                self.elapsed += dt
                self.distance_xy += math.hypot(xyz[0]-old_xyz[0], xyz[1]-old_xyz[1])
                self.distance_3d += math.dist(xyz, old_xyz)
        self.writer.writerow((ns, self.elapsed, time.time() if wall_time is None else wall_time,
                              self.segment, frame, msg.child_frame_id, *xyz, *q, *v,
                              math.sqrt(sum(n*n for n in v)), self.distance_xy, self.distance_3d))
        self.last = ns, frame, xyz
        self.count += 1
        return True

    def flush(self):
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            self.flush()
            self.stream.close()
            self._metadata()


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--topic', default='/quad_0/body_pose')
    parser.add_argument('--world', default='')
    parser.add_argument('--rate', type=float, default=5., help='maximum wall-clock samples/s')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.rate) or args.rate <= 0 or args.rate > 50:
        parser.error('--rate must be in (0, 50]')
    rclpy.init(args=ros_args)
    writer = None
    node = None
    try:
        writer = RouteWriter(args.output, args.world, args.topic, args.rate)
        node = Node('actual_route_recorder')
        latest = [None]
        node.create_subscription(Odometry, args.topic,
                                 lambda msg: latest.__setitem__(0, msg), qos_profile_sensor_data)
        def sample():
            if latest[0] is not None:
                writer.append(latest[0])
        node.create_timer(1./args.rate, sample)
        node.create_timer(1., writer.flush)
        node.get_logger().info(f'ACTUAL_ROUTE_RECORDING topic={args.topic} csv={writer.output} rate={args.rate:g}Hz')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if writer:
            if node and latest[0] is not None:
                writer.append(latest[0])
            writer.close()
            print(f'ACTUAL_ROUTE_SAVED samples={writer.count} distance_xy_m={writer.distance_xy:.3f} csv={writer.output}', flush=True)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
