"""Real ROS processes, synthetic odometry, no Gazebo or physical actuation.

The ACK proxy deliberately withholds retirement confirmation. Tests run in
their own ROS domain and never publish to the user's simulation domain.
"""
import os
import pathlib
import subprocess
import tempfile
import time
import math
import copy
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from scan_planner_msgs.msg import Bspline, ObservationTask, ObservationFeedback, TrajectoryAck, ExecutionState, LocomotionState, RoutePortal
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


class TestTaskRuntime(unittest.TestCase):
    def setUp(self):
        cls=self  # Each scenario gets fresh processes and odometry filters.
        cls.context=Context()
        cls.domain=150+os.getpid()%50
        rclpy.init(context=cls.context,domain_id=cls.domain)
        cls.node=rclpy.create_node("observation_task_contract_probe",context=cls.context)
        cls.executor=SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.children=[]
        cls.logs=[]
        env=dict(os.environ,ROS_DOMAIN_ID=str(cls.domain))
        root=pathlib.Path(__file__).resolve().parents[1]
        binary=pathlib.Path(get_package_prefix("scan_planner"))/"lib"/"scan_planner"
        for executable,extra in (
            ("scan_planner_node",["--params-file",str(root/"config"/"planner.yaml"),
                                  "-p","fsm.task_mode:=true","-p","fsm.navi_mode:=3"]),
            ("closed_loop_controller",["-r","planning/trajectory_ack:=controller_ack_proxy"]),
        ):
            log=tempfile.TemporaryFile(mode="w+")
            cls.logs.append(log)
            cls.children.append(subprocess.Popen([str(binary/executable),"--ros-args"]+extra,
                                                 env=env,stdout=log,stderr=log))
        cls.acks=[]; cls.states=[]; cls.statuses=[]; cls.splines=[]; cls.feedback=[]
        cls.node.create_subscription(ObservationFeedback,"planning/observation_feedback",cls.feedback.append,20)
        cls.node.create_subscription(TrajectoryAck,"controller_ack_proxy",cls.acks.append,20)
        cls.node.create_subscription(ExecutionState,"planning/execution_state",cls.states.append,20)
        cls.node.create_subscription(String,"planning/status",lambda m:cls.statuses.append(m.data),20)
        cls.node.create_subscription(Bspline,"planning/bspline",cls.splines.append,20)
        cls.task_pub=cls.node.create_publisher(ObservationTask,"observation_task",20)
        cls.spline_pub=cls.node.create_publisher(Bspline,"planning/bspline",20)
        cls.ack_pub=cls.node.create_publisher(TrajectoryAck,"planning/trajectory_ack",20)
        cls.odom_pub=cls.node.create_publisher(Odometry,"body_pose",20)
        cls.sensor_pose_pub=cls.node.create_publisher(Odometry,"sensor_pose",20)
        cls.cloud_pub=cls.node.create_publisher(PointCloud2,"cloud",20)
        cls.ready_pub=cls.node.create_publisher(LocomotionState,"/robot/locomotion_state",
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.pose_x=0.0
        cls.pose_yaw=0.0
        cls.pose_vx=0.0
        cls.cloud_counter=0

    def tearDown(self):
        cls=self
        for child in cls.children:
            child.terminate()
        for child in cls.children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=5)
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)
        for log in cls.logs: log.close()

    def spin_until(self,predicate,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            odom=Odometry()
            odom.header.stamp=self.node.get_clock().now().to_msg()
            odom.header.frame_id="world"
            odom.pose.pose.position.x=self.pose_x
            odom.pose.pose.position.z=.4
            odom.pose.pose.orientation.w=math.cos(self.pose_yaw/2)
            odom.pose.pose.orientation.z=math.sin(self.pose_yaw/2)
            odom.twist.twist.linear.x=self.pose_vx
            self.odom_pub.publish(odom)
            self.sensor_pose_pub.publish(odom)
            self.cloud_counter+=1
            if self.cloud_counter%5==0:
                points=[(4.*math.cos(i*math.pi/32),4.*math.sin(i*math.pi/32),z)
                        for i in range(64) for z in (.4,.7)]
                self.cloud_pub.publish(create_cloud_xyz32(odom.header,points))
            ready=LocomotionState()
            ready.stamp=odom.header.stamp
            ready.actuation_ready=True
            ready.perception_pose_valid=True
            self.ready_pub.publish(ready)
            self.executor.spin_once(timeout_sec=.02)
            if predicate(): return
            for child in self.children:
                if child.poll() is not None:
                    outputs=[]
                    for log in self.logs:
                        log.seek(0); outputs.append(log.read()[-5000:])
                    self.fail("ROS child exited: "+"\n".join(outputs))
        outputs=[]
        for log in self.logs:
            log.seek(0); outputs.append(log.read()[-7000:])
        self.fail("Timed out waiting for ROS contract event; statuses="+str(self.statuses[-10:])+"\n"+"\n".join(outputs))

    def task(self,task_id):
        message=ObservationTask()
        message.header.frame_id="world"
        message.header.stamp=self.node.get_clock().now().to_msg()
        message.request_id=self.node.get_clock().now().nanoseconds
        message.task_id=task_id; message.task_version=1; message.lifecycle=ObservationTask.ACTIVE
        message.goal=Point(x=2.,y=0.,z=0.)
        message.coarse_route=[Point(x=0.,y=0.,z=0.),message.goal]
        message.corridor_radius=.8; message.observation_radius=3.; message.resolution=.2
        message.target_cells=[Point(x=3.,y=0.,z=0.)]
        message.target_cell_count=1; message.representative_selection_version=1
        message.max_station_adjustments=3
        return message

    def stationary(self,task_id,request_id,traj_id):
        message=Bspline()
        message.order=3; message.task_id=task_id; message.task_version=1
        message.request_id=request_id; message.traj_id=traj_id; message.candidate_id=9
        message.start_time=self.node.get_clock().now().to_msg()
        message.pos_pts=[Point(x=0.,y=0.,z=.4) for _ in range(4)]
        message.knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.]
        return message

    def test_autonomous_candidate_ack_and_frozen_topology_contract(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        task=self.task(2001); task.local_geometry_autonomy=True; task.route_graph_revision=10
        portal=RoutePortal(connection_id=11,source_node_id=1,target_node_id=2,
                           center=Point(x=.8,y=0.,z=0.),radius=.8)
        portal.direction.x=1.;task.route_portals=[portal]
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==2001 and a.request_id==task.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        ack=next(a for a in self.acks if a.task_id==2001 and a.request_id==task.request_id and
                 a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.assertFalse(any(f.task_id==2001 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==2001 and f.request_id==task.request_id and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        changed=copy.deepcopy(task);changed.request_id=self.node.get_clock().now().nanoseconds
        changed.route_graph_revision=11
        self.task_pub.publish(changed)
        self.spin_until(lambda:any("field=route_contract_changed_without_version" in s and
            f"request_id={changed.request_id}" in s for s in self.statuses))
        self.assertFalse(any(f.request_id==changed.request_id and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))

    def test_retirement_fences_delayed_ack_and_candidate(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
                        self.spline_pub.get_subscription_count()>0)
        invalid = self.task(99)
        invalid.target_cell_count = 0
        self.task_pub.publish(invalid)
        self.spin_until(lambda:any("REFERENCE_PATH_REJECTED" in s and
            f"request_id={invalid.request_id}" in s and "task_id=99" in s and
            "field=target_cells_size" in s for s in self.statuses))
        self.assertFalse(any("PATH_ACCEPTED" in s and
            f"request_id={invalid.request_id}" in s for s in self.statuses))
        first=self.task(100)
        self.task_pub.publish(first)
        self.spin_until(lambda:any("PATH_ACCEPTED" in s and f"request_id={first.request_id}" in s
                                   for s in self.statuses))
        self.spin_until(lambda:any(a.task_id==100 and a.request_id==first.request_id and
                                  a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        planned=next(a for a in self.acks if a.task_id==100 and a.request_id==first.request_id and
                     a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.ack_pub.publish(planned)
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and f"request_id={first.request_id}" in s
                                   for s in self.statuses))
        # Replay a robot turning in place. The acknowledged execution remains
        # owned while Controller aligns; FSM may not replace it with siblings.
        self.pose_yaw=math.pi
        self.spin_until(lambda:any(s.request_id==first.request_id and
            s.trajectory_id==planned.trajectory_id and
            s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        # The same forward candidate cannot be ACKed as yaw=0 while moving
        # backwards-facing, then demand a pi turn after installation.
        self.pose_vx=.2
        until=time.monotonic()+.2
        self.spin_until(lambda:time.monotonic()>=until)
        reversed_handoff=copy.deepcopy(next(s for s in self.splines
            if s.traj_id==planned.trajectory_id))
        reversed_handoff.traj_id=900
        self.spline_pub.publish(reversed_handoff)
        self.spin_until(lambda:any(a.trajectory_id==900 and
            a.status==TrajectoryAck.STATUS_REJECTED and
            a.reason=="HANDOFF_MOVING_HEADING_ERROR" and a.yaw_error>2.5
            for a in self.acks))
        self.assertFalse(any(s.trajectory_id==900 for s in self.states))
        self.pose_vx=0.
        # Once frozen, errors in the hysteresis band must not resume walking.
        self.pose_yaw=-.7
        until=time.monotonic()+.3
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(self.states[-1].state,ExecutionState.STATE_HEADING_FROZEN)
        aligned_acks=len([a for a in self.acks if a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED])
        until=time.monotonic()+1.2
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(aligned_acks,len([a for a in self.acks if a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED]))
        self.pose_yaw=0.
        old=self.stationary(100,first.request_id,1000)
        self.spline_pub.publish(old)
        self.spin_until(lambda:any(a.task_id==100 and a.trajectory_id==1000 and
                                  a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks))
        next_task=self.task(101)
        self.task_pub.publish(next_task)
        self.spin_until(lambda:any(a.task_id==100 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        # Even though Controller has stopped, FSM must wait for its explicit
        # confirmation rather than installing the queued task immediately.
        self.assertFalse(any("PATH_ACCEPTED" in s and f"request_id={next_task.request_id}" in s
                             for s in self.statuses))
        stale=next(a for a in self.acks if a.task_id==100 and a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.ack_pub.publish(stale)
        retirement=next(a for a in self.acks if a.task_id==100 and a.status==TrajectoryAck.STATUS_TASK_RETIRED)
        self.ack_pub.publish(retirement)
        self.spin_until(lambda:any("PATH_ACCEPTED" in s and f"request_id={next_task.request_id}" in s
                                   for s in self.statuses))
        delayed=self.stationary(100,next_task.request_id+1,1001)
        self.spline_pub.publish(delayed)
        self.spin_until(lambda:any(a.task_id==100 and a.trajectory_id==1001 and
                                  a.status==TrajectoryAck.STATUS_REJECTED and
                                  a.reason=="TASK_RETIRED_OR_INVALID" for a in self.acks))
        fresh=self.stationary(101,next_task.request_id+2,1002)
        self.spline_pub.publish(fresh)
        self.spin_until(lambda:any(a.task_id==101 and a.trajectory_id==1002 and
                                  a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks))
        self.spin_until(lambda:any(s.task_id==101 and s.task_version==1 for s in self.states))
        station_task=self.task(102)
        station_task.max_station_adjustments=1
        self.task_pub.publish(station_task)
        self.spin_until(lambda:any(a.task_id==101 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==101 and
                                 a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(a.task_id==102 and a.request_id==station_task.request_id and
                                  a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==102 and
                                 a.request_id==station_task.request_id and
                                 a.status==TrajectoryAck.STATUS_ACCEPTED))
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
                                  f"request_id={station_task.request_id}" in s for s in self.statuses))
        self.pose_x=2.0
        self.spin_until(lambda:any(f.task_id==102 and f.state==ObservationFeedback.STATION_REACHED
                                  for f in self.feedback))
        reached=next(f for f in self.feedback if f.task_id==102 and
                     f.state==ObservationFeedback.STATION_REACHED)
        self.assertEqual(reached.station_adjustments,1)
        self.assertAlmostEqual(reached.station.x,2.0)
        self.assertFalse(any("REACHED request_id="+str(station_task.request_id) in s
                             for s in self.statuses))
        no_gain=self.task(102)
        no_gain.coarse_route = [no_gain.goal]
        no_gain.max_station_adjustments=3
        self.task_pub.publish(no_gain)
        self.spin_until(lambda:any(a.task_id==102 and a.request_id==no_gain.request_id and
                                  a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks))
        self.assertFalse(any(f.task_id==102 and f.request_id==no_gain.request_id and
                             f.reason=="NO_NEW_OBSERVATION_STATION" for f in self.feedback))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==102 and
                                 a.request_id==no_gain.request_id and
                                 a.status==TrajectoryAck.STATUS_ACCEPTED))
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
                                  f"request_id={no_gain.request_id}" in s for s in self.statuses))
        retry=self.task(102)
        retry.max_station_adjustments=1
        self.task_pub.publish(retry)
        self.spin_until(lambda:any(f.task_id==102 and f.request_id==retry.request_id and
                                  f.state==ObservationFeedback.TASK_EXHAUSTED for f in self.feedback))

        # Controller-only mid-segment probe: measured pose remains exactly on
        # the curve, spatial progress stays near zero, duration is ten seconds.
        # The FSM has no pending candidate for this deliberately newer version.
        moving=self.stationary(102,retry.request_id+1,2000)
        moving.pos_pts=[Point(x=x,y=0.,z=.4) for x in (1.5,2.,2.5,3.)]
        moving.knots=[-30.,-20.,-10.,0.,10.,20.,30.,40.]
        moving.execution_mode=Bspline.MODE_CLEARANCE_LIMITED
        moving.max_forward_speed=.3
        moving.min_forward_command=.3
        moving.min_forward_effective_speed=.165
        self.spline_pub.publish(moving)
        self.spin_until(lambda:any(a.trajectory_id==2000 and a.status==TrajectoryAck.STATUS_ACCEPTED
                                  for a in self.acks))
        self.spin_until(lambda:any(s.trajectory_id==2000 and s.state==ExecutionState.STATE_STALLED
                                  and s.reason=="EXECUTION_NO_PROGRESS" for s in self.states),timeout=5)
        stalled=next(s for s in self.states if s.trajectory_id==2000 and
                     s.reason=="EXECUTION_NO_PROGRESS")
        self.assertLess(stalled.execution_time,stalled.duration)
        self.assertAlmostEqual(stalled.duration,10.)

        # Regression: the coarse goal is inside the old 0.25 m arrival band,
        # but the selected, acknowledged local station has not been reached.
        # A stall must generate a replacement, not loop REPLAN->GOAL_REACHED
        # until Explorer's 120 s timeout. Only this isolated domain is used.
        nearby=self.task(103)
        nearby.goal=Point(x=self.pose_x+.24,y=0.,z=0.)
        nearby.coarse_route=[Point(x=self.pose_x,y=0.,z=0.),nearby.goal]
        self.task_pub.publish(nearby)
        self.spin_until(lambda:any(a.task_id==102 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==102 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(a.task_id==103 and a.request_id==nearby.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        near_ack=next(a for a in self.acks if a.task_id==103 and
            a.request_id==nearby.request_id and a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.ack_pub.publish(near_ack)
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
            f"request_id={nearby.request_id}" in s for s in self.statuses))
        state_pub=self.node.create_publisher(ExecutionState,"planning/execution_state",20)
        self.spin_until(lambda:state_pub.get_subscription_count()>0)
        state_pub.publish(ExecutionState(task_id=103,task_version=1,
            request_id=nearby.request_id,trajectory_id=near_ack.trajectory_id,
            state=ExecutionState.STATE_STALLED,reason="EXECUTION_NO_PROGRESS",
            terminal_error=.5))
        self.spin_until(lambda:any(a.task_id==103 and a.request_id==nearby.request_id and
            a.trajectory_id>near_ack.trajectory_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=5)
        self.assertFalse(any(f.task_id==103 and f.state==ObservationFeedback.STATION_REACHED
            for f in self.feedback))


    def test_z_rolling_task_swap_waits_for_candidate_ack_not_retirement(self):
        self.pose_x=0.; self.pose_yaw=0.; self.pose_vx=0.
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
                        self.spline_pub.get_subscription_count()>0)
        first=self.task(10000)
        self.task_pub.publish(first)
        self.spin_until(lambda:any("PATH_ACCEPTED" in s and
                       f"request_id={first.request_id}" in s for s in self.statuses))
        self.spin_until(lambda:any(a.task_id==10000 and a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        first_ack=next(a for a in self.acks if a.task_id==10000 and
            a.request_id==first.request_id and a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.ack_pub.publish(first_ack)
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
            f"request_id={first.request_id}" in s for s in self.statuses))
        old_curve=copy.deepcopy(next(s for s in self.splines if s.traj_id==first_ack.trajectory_id))
        # At rest, rolling candidates cannot buy a pi turn or an instantaneous
        # speed jump merely by passing the position gate.
        for traj_id,points,reason in (
            (9000,[.2,0.,-.2,-.4],"HANDOFF_ROLLING_HEADING_ERROR"),
            (9001,[-.3,0.,.3,.6],"HANDOFF_ROLLING_VELOCITY_ERROR"),
        ):
            discontinuous=self.stationary(10000,first.request_id,traj_id)
            discontinuous.rolling_task_handoff=True
            discontinuous.pos_pts=[Point(x=x,y=0.,z=.4) for x in points]
            self.spline_pub.publish(discontinuous)
            self.spin_until(lambda:any(a.trajectory_id==traj_id and
                a.status==TrajectoryAck.STATUS_REJECTED and a.reason==reason for a in self.acks))
            self.assertFalse(any(s.trajectory_id==traj_id for s in self.states))
        state_start=len(self.states)
        successor=self.task(10001)
        successor.predecessor_task_id=10000; successor.predecessor_task_version=1
        self.task_pub.publish(successor)
        self.spin_until(lambda:any(a.task_id==10001 and a.request_id==successor.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        next_ack=next(a for a in self.acks if a.task_id==10001 and
            a.request_id==successor.request_id and a.status==TrajectoryAck.STATUS_ACCEPTED)
        self.assertFalse(any(a.task_id==10000 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                             for a in self.acks))
        self.assertFalse(any(s.reason=="TASK_RETIRED" for s in self.states[state_start:]))
        # ACK and spline observers are independent subscriptions; receipt
        # order in this probe is not the Controller's installation order.
        self.spin_until(lambda:any(s.task_id==10001 and s.rolling_task_handoff for s in self.splines))
        # Controller fences old candidates at installation, even before SCAN
        # has received the successor ACK and sent its retirement command.
        old_curve.traj_id=100000
        self.spline_pub.publish(old_curve)
        self.spin_until(lambda:any(a.trajectory_id==100000 and
            a.status==TrajectoryAck.STATUS_REJECTED and a.reason=="TASK_RETIRED_OR_INVALID"
            for a in self.acks))
        self.ack_pub.publish(first_ack)  # Delayed predecessor ACK cannot commit G2.
        self.assertFalse(any("PATH_TRAJECTORY_READY" in s and
            f"request_id={successor.request_id}" in s for s in self.statuses))
        self.assertFalse(any(f.task_id==10001 and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.ack_pub.publish(next_ack)
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
            f"request_id={successor.request_id}" in s for s in self.statuses))
        self.spin_until(lambda:any(f.task_id==10001 and f.task_version==1 and
            f.request_id==successor.request_id and f.trajectory_id==next_ack.trajectory_id and
            f.candidate_id==next_ack.candidate_id and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.spin_until(lambda:any(a.task_id==10000 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        self.assertFalse(any(s.task_id==10001 and s.reason=="TASK_RETIRED"
                             for s in self.states[state_start:]))
        # Cancellation while the next candidate ACK is withheld must stop
        # whichever task is physically installed, and fence a late ACK.
        cancelled=self.task(10002)
        cancelled.predecessor_task_id=10001; cancelled.predecessor_task_version=1
        self.task_pub.publish(cancelled)
        self.spin_until(lambda:any(a.task_id==10002 and a.request_id==cancelled.request_id and
            a.status==TrajectoryAck.STATUS_ACCEPTED for a in self.acks),timeout=8)
        cancelled_ack=next(a for a in self.acks if a.task_id==10002 and
            a.request_id==cancelled.request_id and a.status==TrajectoryAck.STATUS_ACCEPTED)
        cancelled.lifecycle=ObservationTask.CANCELLED
        self.task_pub.publish(cancelled)
        self.spin_until(lambda:any(a.task_id==10002 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==10002 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(f.task_id==10002 and f.task_version==1 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        self.ack_pub.publish(cancelled_ack)
        until=time.monotonic()+.15
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertFalse(any("PATH_TRAJECTORY_READY" in s and
            f"request_id={cancelled.request_id}" in s for s in self.statuses))
        self.assertFalse(any(f.task_id==10002 and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.assertTrue(any(s.task_id==10002 and s.state==ExecutionState.STATE_SOFT_HOLD
            and s.reason=="TASK_RETIRED" for s in self.states))
        # A contract rejected before activation must still finish cancellation.
        invalid=self.task(10003)
        invalid.target_cells=[]
        self.task_pub.publish(invalid)
        self.spin_until(lambda:any("REFERENCE_PATH_REJECTED" in s and
            f"task_id={invalid.task_id}" in s for s in self.statuses))
        invalid.lifecycle=ObservationTask.CANCELLED
        self.task_pub.publish(invalid)
        self.spin_until(lambda:any(a.task_id==10003 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==10003 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(f.task_id==10003 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))


if __name__=="__main__": unittest.main()
