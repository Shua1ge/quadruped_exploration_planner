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
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from scan_planner_msgs.msg import LocalMapPatch, Bspline, ObservationTask, ObservationFeedback, TrajectoryAck, ExecutionState, ExecutionCommand, LocomotionState, RoutePortal
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, create_cloud


class TestTaskRuntime(unittest.TestCase):
    def setUp(self):
        cls=self  # Each scenario gets fresh processes and odometry filters.
        cls.context=Context()
        cls.domain=int(os.environ.get("SCAN_TEST_ROS_DOMAIN",150+os.getpid()%50))
        rclpy.init(context=cls.context,domain_id=cls.domain)
        cls.node=rclpy.create_node("observation_task_contract_probe",context=cls.context)
        cls.executor=SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.children=[]
        cls.logs=[]
        env=dict(os.environ,ROS_DOMAIN_ID=str(cls.domain))
        root=pathlib.Path(__file__).resolve().parents[1]
        binary=pathlib.Path(os.environ.get("SCAN_TEST_BINARY_DIR",
            str(pathlib.Path(get_package_prefix("scan_planner"))/"lib"/"scan_planner")))
        processes = (
            ("scan_planner_node",["--params-file",str(root/"config"/"planner.yaml"),
                                  "-p","fsm.task_mode:=true","-p","fsm.navi_mode:=3"]),
            ("closed_loop_controller",["-r","planning/trajectory_ack:=controller_ack_proxy"]),
        )
        if self._testMethodName=="test_original_laser_no_return_updates_explorer_map":
            processes += (("frontier_explorer.py",["-p","auto_start:=false",
                "-p","use_laser_scan:=true","-p","cloud_frame_id:=lidar",
                "-p","cloud_is_world:=false","-p","need_extrinsic:=true",
                "-p","lidar_extrinsic_z:=0.12","-p","obstacle_z_relative_to_body:=true",
                # Explicit fixture band; sensor origin is evaluated at capture
                # time rather than assumed to coincide with body zero.
                "-p","obstacle_min_z:=-0.25","-p","obstacle_max_z:=0.25"]),)
        for executable,extra in processes:
            if (executable == "scan_planner_node" and
                    self._testMethodName == "test_cancel_confirmation_timeout_is_explicit_and_recoverable"):
                extra += ["-p", "fsm.task_cancel_timeout:=0.5"]
            if (executable == "closed_loop_controller" and
                    self._testMethodName == "test_predecessor_cancel_needs_actual_confirmation"):
                extra += ["-r", "planning/bspline:=controller_spline_proxy",
                          "-r", "planning/execution_command:=controller_command_proxy"]
            if (executable == "closed_loop_controller" and
                    self._testMethodName in ("test_rolling_handoff_velocity_check_is_frame_consistent",
                        "test_rolling_handoff_rejects_world_velocity_jump",
                        "test_rolling_rejection_waits_for_confirmed_rest_before_retry",
                        "test_failed_recovery_rejoin_rejection_is_terminal",
                        "test_failed_recovery_rejoin_ack_timeout_does_not_restart_escape",
                        "test_installation_rejection_retries_same_station_without_blacklist")):
                extra += ["-r", "planning/bspline:=controller_spline_proxy"]
            if executable=="scan_planner_node" and self._testMethodName=="test_original_laser_no_return_updates_local_patch":
                extra += ["-p","grid_map.use_laser_scan:=true","-p","grid_map.need_extrinsic:=false"]
            if executable=="scan_planner_node" and self._testMethodName.startswith("test_measured_ground_slope_"):
                extra += ["-p","grid_map.use_ray_cloud:=true","-p","grid_map.cloud_is_world:=true",
                          "-p","grid_map.need_extrinsic:=false"]
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
        cls.controller_spline_pub=cls.node.create_publisher(Bspline,"controller_spline_proxy",20)
        cls.ack_pub=cls.node.create_publisher(TrajectoryAck,"planning/trajectory_ack",20)
        cls.odom_pub=cls.node.create_publisher(Odometry,"body_pose",20)
        cls.sensor_pose_pub=cls.node.create_publisher(Odometry,"sensor_pose",20)
        cls.cloud_pub=cls.node.create_publisher(PointCloud2,"cloud",20)
        cls.scan_pub=cls.node.create_publisher(LaserScan,"scan",20)
        cls.patches=[];cls.explorer_maps=[]
        cls.node.create_subscription(OccupancyGrid,"explorer/planning_map",cls.explorer_maps.append,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.node.create_subscription(LocalMapPatch,"grid_map/local_map_patch",cls.patches.append,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.ready_pub=cls.node.create_publisher(LocomotionState,"/robot/locomotion_state",
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.commands=[]
        cls.node.create_subscription(Twist,"cmd_vel",cls.commands.append,50)
        cls.pose_y=0.0
        cls.pose_vy=0.0
        cls.pose_last_wall=time.monotonic()
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
            if getattr(self,'advance_commands',False) and self.commands:
                now=time.monotonic(); dt=min(.05,now-self.pose_last_wall)
                command=self.commands[-1]
                self.pose_vx=command.linear.x;self.pose_vy=command.linear.y
                if self.splines and self.splines[-1].execution_mode==Bspline.MODE_FORWARD_TURN_STAGE:
                    # Use the measured legacy forward gain, rather than the
                    # old ideal actuator that equated command and velocity.
                    self.pose_vx*=self.splines[-1].min_forward_effective_speed/self.splines[-1].min_forward_command
                c,s=math.cos(self.pose_yaw),math.sin(self.pose_yaw)
                self.pose_x+=(c*self.pose_vx-s*self.pose_vy)*dt
                self.pose_y+=(s*self.pose_vx+c*self.pose_vy)*dt
                self.pose_last_wall=now
            if getattr(self,'advance_pose',False):
                self.pose_x+=self.pose_vx*.02
            if hasattr(self, 'yaw_response_deadband') and self.commands:
                now=time.monotonic()
                cmd=self.commands[-1].angular.z
                self.pose_yaw_rate=cmd if abs(cmd)>self.yaw_response_deadband else 0.
                self.pose_yaw+=self.pose_yaw_rate*min(.05,now-self.yaw_response_wall)
                self.yaw_response_wall=now
            if getattr(self,'advance_yaw',False):
                now=time.monotonic()
                self.pose_yaw+=self.pose_yaw_rate*min(.05,now-self.pose_yaw_last_wall)
                self.pose_yaw_last_wall=now
            odom=Odometry()
            odom.header.stamp=self.node.get_clock().now().to_msg()
            odom.header.frame_id="world"
            odom.pose.pose.position.x=self.pose_x
            odom.pose.pose.position.y=self.pose_y
            odom.pose.pose.position.z=.4
            odom.pose.pose.orientation.w=math.cos(self.pose_yaw/2)
            odom.pose.pose.orientation.z=math.sin(self.pose_yaw/2)
            odom.twist.twist.linear.x=self.pose_vx
            odom.twist.twist.linear.y=self.pose_vy
            odom.twist.twist.angular.z=getattr(self,"pose_yaw_rate",0.)
            self.odom_pub.publish(odom)
            self.sensor_pose_pub.publish(odom)
            self.cloud_counter+=1
            if self.cloud_counter%5==0:
                points=getattr(self,"cloud_points",None) or [(4.*math.cos(i*math.pi/32),4.*math.sin(i*math.pi/32),z)
                        for i in range(64) for z in (.4,.7)]
                factory=getattr(self,"cloud_message_factory",create_cloud_xyz32)
                self.cloud_pub.publish(factory(odom.header,points))
                if getattr(self,'use_original_scan',False):
                    scan=LaserScan();scan.header=copy.deepcopy(odom.header)
                    scan.header.frame_id="lidar"
                    scan.angle_min=-math.pi;scan.angle_max=math.pi
                    scan.angle_increment=2*math.pi/720;scan.range_min=.2;scan.range_max=7.5
                    scan.ranges=[float('inf')]*720
                    self.scan_pub.publish(scan)
            ready=LocomotionState()
            ready.stamp=odom.header.stamp
            ready.actuation_ready=True
            ready.perception_pose_valid=True
            self.ready_pub.publish(ready)
            self.executor.spin_once(timeout_sec=.02)
            # Only the end-to-end station leg enables this proxy. Earlier
            # identity-fencing checks deliberately withhold selected ACKs.
            auto_request=getattr(self,'auto_ack_request',None)
            if auto_request is not None:
                forwarded=getattr(self,'forwarded_acks',set())
                for ack in self.acks:
                    key=(ack.request_id,ack.trajectory_id,ack.candidate_id,ack.status)
                    if (ack.request_id==auto_request and
                            ack.status==TrajectoryAck.STATUS_ACTIVATED and key not in forwarded):
                        self.ack_pub.publish(ack);forwarded.add(key)
                self.forwarded_acks=forwarded
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

    def read_child_log(self,index):
        log=self.logs[index]
        return os.pread(log.fileno(),os.fstat(log.fileno()).st_size,0).decode()

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

    def prepare_boundary_source(self, deep=False):
        # The hit is above the self-filter cap and inside the vertical body
        # sweep. It remains a real measured source at its original XYZ.
        self.cloud_points=[(.18,.10 if deep else .24,.65)]
        self.cloud_points += [(4.*math.cos(i*math.pi/32),4.*math.sin(i*math.pi/32),z)
                              for i in range(64) for z in (.4,.7)]
        self.spin_until(lambda:"reason=pose_occupied" in self.read_child_log(0),6.)

    def test_boundary_escape_is_submitted_and_ack_activates_fixed_yaw_exit(self):
        self.prepare_boundary_source()
        task=self.task(1401);task.local_geometry_autonomy=True;self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==1401 and
            s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),6.)
        candidate=next(s for s in self.splines if s.task_id==1401 and
            s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE)
        self.assertGreater(candidate.terminal_braking_deceleration,0.)
        self.assertGreater(candidate.terminal_command_latency,0.)
        self.assertEqual(len(candidate.yaw_pts),1)
        distance=math.hypot(candidate.pos_pts[-1].x-candidate.pos_pts[0].x,
                            candidate.pos_pts[-1].y-candidate.pos_pts[0].y)
        stop=candidate.max_lateral_speed*candidate.terminal_command_latency+candidate.max_lateral_speed**2/(2*candidate.terminal_braking_deceleration)
        self.assertGreaterEqual(distance,stop+.04)
        ack=TrajectoryAck();ack.request_id=candidate.request_id;ack.trajectory_id=candidate.traj_id
        ack.task_id=candidate.task_id;ack.task_version=candidate.task_version;ack.candidate_id=candidate.candidate_id
        ack.status=TrajectoryAck.STATUS_ACTIVATED
        self.ack_pub.publish(ack)
        self.spin_until(lambda:"[CANDIDATE_TRAJECTORY_COMMITTED]" in self.read_child_log(0),3.)
        self.spin_until(lambda:any(c.linear.y<-.05 for c in self.commands),3.)
        self.assertFalse(any(f.state==ObservationFeedback.LOCAL_MOTION_EXHAUSTED for f in self.feedback))

    def test_boundary_escape_rejects_deep_overlap(self):
        self.prepare_boundary_source(deep=True)
        task=self.task(1402);task.local_geometry_autonomy=True;self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==1402 and
            f.state==ObservationFeedback.LOCAL_MOTION_EXHAUSTED for f in self.feedback),6.)
        self.assertFalse(any(s.task_id==1402 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines))
        self.assertIn("[MOTION_REJECTION_SOURCE]",self.read_child_log(0))
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))

    def test_boundary_escape_new_obstacle_revokes_acknowledged_exit(self):
        self.prepare_boundary_source()
        task=self.task(1403);task.local_geometry_autonomy=True;self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==1403 and
            s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),6.)
        candidate=next(s for s in self.splines if s.task_id==1403 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE)
        ack=TrajectoryAck();ack.request_id=candidate.request_id;ack.trajectory_id=candidate.traj_id
        ack.task_id=candidate.task_id;ack.task_version=candidate.task_version;ack.candidate_id=candidate.candidate_id
        ack.status=TrajectoryAck.STATUS_ACTIVATED;self.ack_pub.publish(ack)
        self.spin_until(lambda:"[CANDIDATE_TRAJECTORY_COMMITTED]" in self.read_child_log(0),3.)
        self.cloud_points.append((-.18,-.19,.65))
        self.spin_until(lambda:"[BOUNDARY_ESCAPE_STOP]" in self.read_child_log(0),5.)
        self.spin_until(lambda:any(f.task_id==1403 and
            "BOUNDARY_CERTIFICATE_REVOKED" in f.reason for f in self.feedback),3.)

    def stationary(self,task_id,request_id,traj_id):
        message=Bspline()
        message.order=3; message.task_id=task_id; message.task_version=1
        message.request_id=request_id; message.traj_id=traj_id; message.candidate_id=9
        message.start_time=self.node.get_clock().now().to_msg()
        # Controller-only fixtures explicitly supply the mock Local grant.
        # Planner-produced splines must carry their own measured certificate.
        message.handoff_certified=True
        message.handoff_position=Point(x=self.pose_x,y=self.pose_y,z=.4)
        message.handoff_body_yaw=self.pose_yaw
        message.handoff_target_yaw=self.pose_yaw
        message.handoff_matched_time=0.
        message.pos_pts=[Point(x=0.,y=0.,z=.4) for _ in range(4)]
        message.knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.]
        return message


    def rolling_probe(self, yaw, direction, request_id, traj_id, speed=.5):
        """Inject one rolling candidate moving along `direction` (world, rad).

        The robot is commanded at 0.5 m/s along its own body +x. Spline speed is
        chosen so the world-frame motion is continuous with that: a correct
        handoff check must accept it at any yaw.
        """
        self.pose_yaw = yaw
        self.pose_vx = 0.5
        # Publish repeated fresh source states before handing over the candidate.
        # A single spin is not an acknowledgement of the new orientation.
        state_count=len(self.states)
        self.spin_until(lambda:len(self.states)>=state_count+3,timeout=2.)
        msg = Bspline()
        msg.order = 3; msg.task_id = request_id; msg.task_version = 1
        msg.request_id = request_id; msg.traj_id = traj_id; msg.candidate_id = 5
        msg.start_time = self.node.get_clock().now().to_msg()
        msg.rolling_task_handoff = True
        msg.handoff_certified = True
        msg.handoff_position = Point(x=self.pose_x, y=self.pose_y, z=.4)
        msg.handoff_body_yaw = yaw
        msg.handoff_target_yaw = direction
        msg.handoff_matched_time = 0.
        # Cubic B-spline through control points -s,0,s,2s starts exactly at the
        # robot (B(0) = (P0+4P1+P2)/6 = 0) with speed 3s ~= 0.5 m/s.
        step = speed   # uniform one-second knots: derivative equals spacing
        msg.pos_pts = [Point(x=(d) * math.cos(direction),
                             y=(d) * math.sin(direction), z=.4)
                       for d in (-step, 0., step, 2 * step)]
        msg.knots = [-3., -2., -1., 0., 1., 2., 3., 4.]
        self.controller_spline_pub.publish(msg)
        self.spin_until(lambda: any(a.trajectory_id == traj_id for a in self.acks),
                        timeout=4)
        ack = next((a for a in self.acks if a.trajectory_id == traj_id), None)
        print(f"[PROBE] yaw={yaw:.2f} dir={direction:.2f} status={getattr(ack,'status',None)} "
              f"reason={getattr(ack,'reason','')!r} pos_err={getattr(ack,'position_error',None)} "
              f"yaw_err={getattr(ack,'yaw_error',None)}", flush=True)
        return ack

    def test_rolling_handoff_velocity_check_is_frame_consistent(self):
        """A continuous world motion must not be rejected because of yaw.

        The controller compares the world-frame spline derivative against the
        body-frame odometry twist. At yaw=90 deg a correctly continuous 0.5 m/s
        handoff differs by ~0.7 m/s in that comparison, far above the 0.125 m/s
        limit, so a legal successor is rejected with
        HANDOFF_ROLLING_VELOCITY_ERROR. The yaw=0 case is the control.
        """
        self.spin_until(lambda: self.controller_spline_pub.get_subscription_count() > 0)
        control = self.rolling_probe(0.0, 0.0, 771001, 771001)
        self.assertIsNotNone(control, "control case produced no ACK")
        self.assertEqual(control.status,TrajectoryAck.STATUS_ACTIVATED,control.reason)
        self.assertEqual(control.reason,"HANDOFF_ACCEPTED")
        turned = self.rolling_probe(math.pi / 2., math.pi / 2., 771002, 771002)
        self.assertIsNotNone(turned, "yaw=90 case produced no ACK")
        self.assertEqual(turned.status,TrajectoryAck.STATUS_ACTIVATED,turned.reason)
        self.assertEqual(turned.reason,"HANDOFF_ACCEPTED")
        self.spin_until(lambda:any(s.trajectory_id==771002 for s in self.states))

    def test_rolling_handoff_rejects_world_velocity_jump(self):
        # Independent process: changing yaw between accepted probes must not
        # make a pending alignment mask the velocity gate under test.
        self.spin_until(lambda:self.controller_spline_pub.get_subscription_count()>0)
        rejected=self.rolling_probe(math.pi/2,math.pi/2,771003,771003,speed=.9)
        self.assertEqual(rejected.status,TrajectoryAck.STATUS_REJECTED)
        self.assertEqual(rejected.reason,"HANDOFF_ROLLING_VELOCITY_ERROR")
        self.assertFalse(any(s.trajectory_id==771003 for s in self.states))

    def test_deferred_timeout_expires_in_real_explorer_process(self):
        # No Local terminal reply is sent. Exercise the real subscription,
        # periodic planning callback, timeout cleanup and late-status fence.
        script = """
import sys,rclpy
sys.path.insert(0,sys.argv[1])
from frontier_explorer import FrontierExplorer
rclpy.init(args=['--ros-args','-p','auto_start:=true','-p','deferred_path_timeout:=0.7',
                '-r','planning/status:=deferred_probe_status'])
e=FrontierExplorer()
e.position=(0.,0.);e.map_update_count=2;e.locomotion_ready=lambda:True
e.pending_path_publish_ns=e.get_clock().now().nanoseconds
e.pending_path_request_generation=e.active_path_request_generation=404
e.blacklist=[(9.,9.)]
print('DEFERRED_PROBE_READY',flush=True)
def report():
    if e.last_active_goal_clear_reason=='DEFERRED_REQUEST_TIMEOUT' or e.pending_path_request_generation is None:
        print('DEFERRED_PROBE_RELEASED blacklist='+repr(e.blacklist),flush=True)
e.create_timer(.3,report)
rclpy.spin(e)
"""
        log=tempfile.TemporaryFile(mode='w+');self.logs.append(log)
        scripts=str(pathlib.Path(__file__).resolve().parents[1]/'scripts')
        child=subprocess.Popen(['/usr/bin/python3','-c',script,scripts],
            env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain)),stdout=log,stderr=log)
        self.children.append(child);index=len(self.logs)-1
        pub=self.node.create_publisher(String,'deferred_probe_status',20)
        self.spin_until(lambda:pub.get_subscription_count()>0 and
            'DEFERRED_PROBE_READY' in self.read_child_log(index))
        pub.publish(String(data='PATH_DEFERRED request_id=404'))
        self.spin_until(lambda:'[PATH_REQUEST_DEFERRED_TIMEOUT]' in self.read_child_log(index),timeout=5)
        self.spin_until(lambda:'DEFERRED_PROBE_RELEASED blacklist=[(9.0, 9.0)]' in self.read_child_log(index))
        pub.publish(String(data='PATH_ACCEPTED request_id=404'))
        self.spin_until(lambda:'[STALE_PLANNING_STATUS_IGNORED]' in self.read_child_log(index))

    def test_stopped_handoff_finishes_small_rotation_before_translation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8810,self.node.get_clock().now().nanoseconds,8810)
        yaw=.5
        msg.pos_pts=[Point(x=d*math.cos(yaw),y=d*math.sin(yaw),z=.4)
                     for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=yaw
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(s.trajectory_id==8810 and
            s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.pose_yaw=.44;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.pose_yaw=.48
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))

    def test_replacement_cannot_release_unfinished_turn_during_yaw_motion(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        old=self.stationary(8820,self.node.get_clock().now().nanoseconds,8820)
        old.pos_pts=[Point(x=d*math.cos(.5),y=d*math.sin(.5),z=.4) for d in (-.3,0.,.3,.6)]
        old.handoff_target_yaw=.5;self.spline_pub.publish(old)
        self.spin_until(lambda:any(s.trajectory_id==8820 and s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        self.pose_yaw=.135;self.pose_vx=.06;self.pose_yaw_rate=.21
        self.spin_until(lambda:len(self.commands)>10)
        new=copy.deepcopy(old);new.traj_id=8821;new.handoff_body_yaw=self.pose_yaw
        self.spline_pub.publish(new)
        self.spin_until(lambda:any(a.trajectory_id==8821 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.pose_vx=0.;self.pose_yaw=.48;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.pose_yaw_rate=0.
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))

    def test_initial_turn_waits_for_measured_angular_rest(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        self.pose_yaw_rate=.21
        # Deliver angular odometry before the grant so the certificate and
        # Controller inspect the same measured state, rather than a callback race.
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        message=self.stationary(8822,self.node.get_clock().now().nanoseconds,8822)
        message.handoff_target_yaw=.5
        message.pos_pts=[Point(x=d*math.cos(.5),y=d*math.sin(.5),z=.4) for d in (-.3,0.,.3,.6)]
        self.spline_pub.publish(message)
        self.spin_until(lambda:any(s.trajectory_id==8822 and s.reason=='HEADING_ALIGNMENT_WAIT_REST' for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.pose_yaw_rate=0.
        self.spin_until(lambda:any(c.angular.z>.1 for c in self.commands))

    def test_completed_normal_spline_never_resumes_tracking_after_pose_drift(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8850,self.node.get_clock().now().nanoseconds,8850)
        msg.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.3,0.,.3,.6)]
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8850 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.pose_x=.3;self.pose_vx=.12
        self.spin_until(lambda:any(s.trajectory_id==8850 and s.reason=='TERMINAL_SETTLING' for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>20)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.assertFalse(any(s.trajectory_id==8850 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_vx=0.
        self.spin_until(lambda:any(s.trajectory_id==8850 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_x=.5;self.pose_yaw=.3;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>20)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        replacement=self.stationary(8850,msg.request_id,8851)
        replacement.pos_pts=[Point(x=d,y=0.,z=.4) for d in (.2,.5,.8,1.1)]
        replacement.handoff_target_yaw=0.
        self.spline_pub.publish(replacement)
        self.spin_until(lambda:any(a.trajectory_id==8851 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.pose_yaw=0.
        self.commands.clear();self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))

    def test_certified_alignment_overcomes_small_command_deadband_before_translation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8860,self.node.get_clock().now().nanoseconds,8860)
        yaw=.208
        msg.pos_pts=[Point(x=d*math.cos(yaw),y=d*math.sin(yaw),z=.4) for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=yaw;msg.handoff_alignment_tolerance=.12
        self.yaw_response_deadband=.35;self.yaw_response_wall=time.monotonic()
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(c.angular.z>.35 for c in self.commands),timeout=3)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands),timeout=5)
        self.assertLessEqual(abs(yaw-self.pose_yaw),.12)
        self.assertFalse(any(s.reason=='HEADING_STALLED' for s in self.states))
        self.assertLessEqual(max(abs(c.angular.z) for c in self.commands),1.)

    def test_ineffective_alignment_reports_typed_terminal_instead_of_same_spline_retry(self):
        self.pose_yaw=.5
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8861);task.local_geometry_autonomy=True
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==8861 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=12)
        ack=next(a for a in self.acks if a.task_id==8861 and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==8861 and f.state==ObservationFeedback.LOCAL_MOTION_EXHAUSTED
            and f.reason=='HEADING_ALIGNMENT_INEFFECTIVE' for f in self.feedback),timeout=7)
        self.spin_until(lambda:any(s.trajectory_id==ack.trajectory_id and s.state==ExecutionState.STATE_SOFT_HOLD for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.assertEqual(len({s.traj_id for s in self.splines if s.task_id==8861}),1)

    def test_local_yaw_band_allows_certified_residual_but_rejects_unbounded_grant(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8852,self.node.get_clock().now().nanoseconds,8852)
        msg.pos_pts=[Point(x=d*math.cos(.5),y=d*math.sin(.5),z=.4) for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=.5;msg.handoff_alignment_tolerance=.12
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(s.trajectory_id==8852 and s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        self.pose_yaw=.42;self.commands.clear()
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        invalid=copy.deepcopy(msg);invalid.traj_id=8853
        invalid.handoff_body_yaw=self.pose_yaw;invalid.handoff_alignment_tolerance=.121
        self.spline_pub.publish(invalid)
        self.spin_until(lambda:any(a.trajectory_id==8853 and a.reason=='HANDOFF_CERTIFICATE_YAW_BAND_INVALID' for a in self.acks))
        self.assertFalse(any(s.trajectory_id==8853 for s in self.states))

    def test_normal_moving_candidate_keeps_continuous_translation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        self.pose_vx=.25
        message=self.stationary(8823,self.node.get_clock().now().nanoseconds,8823)
        message.use_measured_progress=True
        message.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.3,0.,.3,.6)]
        self.spline_pub.publish(message)
        self.spin_until(lambda:any(a.trajectory_id==8823 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear();self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        self.assertFalse(any(s.trajectory_id==8823 and s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))

    def test_normal_progress_is_measured_without_minimum_command_or_whole_route_limit(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8880,self.node.get_clock().now().nanoseconds,8880)
        msg.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.5,0.,.5,1.)]
        msg.use_measured_progress=True
        msg.max_forward_speed=.25
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8880 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        end=time.monotonic()+1.3  # Longer than the entire reference duration.
        self.spin_until(lambda:time.monotonic()>end)
        states=[s for s in self.states if s.trajectory_id==8880]
        self.assertTrue(states)
        self.assertLess(max(s.execution_time for s in states),.05)
        self.assertFalse(any(s.state==ExecutionState.STATE_FINISHED for s in states))
        self.assertTrue(any(.05<c.linear.x<=.25 for c in self.commands))
        self.pose_x=.5
        self.spin_until(lambda:any(s.trajectory_id==8880 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_x=.7;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_measured_progress_successor_swaps_while_moving_without_terminal_stop(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        self.pose_vx=.2
        old=self.stationary(8881,self.node.get_clock().now().nanoseconds,8881)
        # Keep well outside the legitimate terminal-capture band while swapping.
        old.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-1.,0.,1.,2.)]
        old.knots=[-15.,-10.,-5.,0.,5.,10.,15.,20.]
        old.use_measured_progress=True
        self.spline_pub.publish(old)
        self.spin_until(lambda:any(a.trajectory_id==8881 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.pose_x=.1
        self.spin_until(lambda:any(s.trajectory_id==8881 and s.execution_time>.4 for s in self.states))
        new=self.stationary(8882,old.request_id+1,8882)
        new.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.9,.1,1.1,2.1)]
        new.knots=list(old.knots)
        new.use_measured_progress=True;new.rolling_task_handoff=True
        self.commands.clear();self.spline_pub.publish(new)
        self.spin_until(lambda:any(a.trajectory_id==8882 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(c.linear.x>.05 for c in self.commands))
        self.assertFalse(any(s.trajectory_id==8882 and s.state in
            (ExecutionState.STATE_HEADING_FROZEN,ExecutionState.STATE_SOFT_HOLD,ExecutionState.STATE_FINISHED) for s in self.states))

    def test_fixed_yaw_stage_waits_for_residual_turn_to_stop(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        self.pose_yaw_rate=.21
        # Deliver angular odometry before the grant so the certificate and
        # Controller inspect the same measured state, rather than a callback race.
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        message=self.stationary(8824,self.node.get_clock().now().nanoseconds,8824)
        message.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.3,0.,.3,.6)]
        message.execution_mode=Bspline.MODE_CLEARANCE_LIMITED
        message.yaw_pts=[0.];message.yaw_dt=1.
        message.use_measured_progress=True
        message.min_forward_command=.30;message.min_forward_effective_speed=.165
        message.max_forward_speed=.30;message.max_lateral_speed=.08
        message.max_yaw_rate=.10;message.max_tracking_correction=.08
        self.spline_pub.publish(message)
        self.spin_until(lambda:any(s.trajectory_id==8824 and s.reason=='HEADING_ALIGNMENT_WAIT_REST' for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.pose_yaw_rate=0.
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))

    def test_departure_wait_does_not_reuse_pose_changed_by_residual_rotation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        self.pose_yaw_rate=.21
        # Deliver angular odometry before the grant so the certificate and
        # Controller inspect the same measured state, rather than a callback race.
        self.commands.clear();self.spin_until(lambda:len(self.commands)>5)
        message=self.stationary(8825,self.node.get_clock().now().nanoseconds,8825)
        message.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.3,0.,.3,.6)]
        self.spline_pub.publish(message)
        self.spin_until(lambda:any(s.trajectory_id==8825 and s.reason=='HEADING_ALIGNMENT_WAIT_REST' for s in self.states))
        self.pose_yaw=.03
        self.spin_until(lambda:any(s.trajectory_id==8825 and s.reason=='HANDOFF_REST_STATE_CHANGED' for s in self.states))
        self.pose_yaw_rate=0.;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))

    def test_certified_turn_envelope_keeps_one_spline_through_gait_translation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8830,self.node.get_clock().now().nanoseconds,8830)
        msg.pos_pts=[Point(x=d*math.cos(.5),y=d*math.sin(.5),z=.4) for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=.5;msg.handoff_turn_position_allowance=.10
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(c.angular.z>.1 for c in self.commands))
        self.pose_x=.06;self.pose_yaw=.3;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(any(c.angular.z>.1 for c in self.commands))
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)<1e-9 for c in self.commands))
        self.assertFalse(any(s.trajectory_id==8830 and s.state==ExecutionState.STATE_STALLED for s in self.states))
        self.pose_yaw=.48;self.commands.clear()
        self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        self.assertEqual(sum(a.trajectory_id==8830 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),1)

    def test_terminal_speed_limit_keeps_wide_prefix_fast(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8891,self.node.get_clock().now().nanoseconds,8891)
        msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (-5.,0.,5.,10.)]
        msg.terminal_speed_limit=.30
        msg.terminal_braking_deceleration=.32
        msg.terminal_command_latency=.55
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8891 and a.status==TrajectoryAck.STATUS_ACTIVATED
                                  for a in self.acks))
        self.spin_until(lambda:any(c.linear.x>.50 for c in self.commands))
        self.commands.clear();self.pose_x=4.8
        self.spin_until(lambda:len(self.commands)>=5)
        moving=[c.linear.x for c in self.commands if c.linear.x>.01]
        self.assertTrue(moving)
        self.assertLessEqual(max(moving[-3:]),.50)

    def test_fresh_turn_grants_continue_alignment_but_stale_grants_do_not(self):
        refresh_pub=self.node.create_publisher(ExecutionCommand,"planning/execution_command",20)
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0 and refresh_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        yaw=.8
        msg=self.stationary(8890,self.node.get_clock().now().nanoseconds,8890)
        msg.pos_pts=[Point(x=d*math.cos(yaw),y=d*math.sin(yaw),z=.4)
                     for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=yaw;msg.handoff_turn_position_allowance=.10
        msg.handoff_alignment_tolerance=.10
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(s.trajectory_id==8890 and s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        self.spin_until(lambda:any(abs(c.angular.z)>.1 for c in self.commands))
        for i in range(1,37):
            self.pose_x=i*.005
            deadline=time.monotonic()+.025
            while time.monotonic()<deadline:self.executor.spin_once(timeout_sec=.005)
            grant=ExecutionCommand()
            grant.command=ExecutionCommand.COMMAND_REFRESH_TURN
            grant.request_id=msg.request_id;grant.task_id=msg.task_id
            grant.task_version=msg.task_version;grant.trajectory_id=8890
            grant.certificate_stamp=self.node.get_clock().now().to_msg()
            grant.certificate_map_revision=i
            grant.certificate_position=Point(x=self.pose_x,y=0.,z=.4)
            grant.certificate_target_yaw=yaw;grant.certificate_position_allowance=.10
            refresh_pub.publish(grant)
        self.assertFalse(any(s.trajectory_id==8890 and s.reason=='HANDOFF_ALIGNMENT_DEVIATED' for s in self.states))
        self.assertTrue(any(abs(c.angular.z)>.1 for c in self.commands))
        self.pose_x=.32
        grant.certificate_position.x=.32
        grant.certificate_stamp.sec-=1
        refresh_pub.publish(grant)
        self.spin_until(lambda:any(s.trajectory_id==8890 and s.reason=='HANDOFF_ALIGNMENT_DEVIATED' for s in self.states))

    def test_controller_cannot_invent_turn_envelope_or_cross_its_boundary(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        for i,allowance in enumerate((-.01,.101,float('nan'))):
            msg=self.stationary(8831,self.node.get_clock().now().nanoseconds,8831+i)
            msg.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.3,0.,.3,.6)]
            msg.handoff_turn_position_allowance=allowance;self.spline_pub.publish(msg)
            self.spin_until(lambda:any(a.trajectory_id==msg.traj_id and a.reason=='HANDOFF_CERTIFICATE_TURN_ENVELOPE_INVALID' for a in self.acks))
        msg=self.stationary(8834,self.node.get_clock().now().nanoseconds,8834)
        msg.pos_pts=[Point(x=d*math.cos(.5),y=d*math.sin(.5),z=.4) for d in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=.5;msg.handoff_turn_position_allowance=.10
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(c.angular.z>.1 for c in self.commands))
        self.pose_x=.11
        self.spin_until(lambda:any(s.trajectory_id==8834 and s.reason=='HANDOFF_ALIGNMENT_DEVIATED' for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def begin_turn_fault_on_committed_task(self,task_id,reason):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(task_id);task.local_geometry_autonomy=True
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==task_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        ack=next(a for a in self.acks if a.task_id==task_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==task_id and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        first=next(s for s in self.splines if s.task_id==task_id and s.traj_id==ack.trajectory_id)
        self.assertGreaterEqual(first.handoff_turn_position_allowance,.02)
        self.assertLessEqual(first.handoff_turn_position_allowance,.10)
        self.pose_yaw_rate=.25;self.advance_yaw=True
        self.pose_yaw_last_wall=time.monotonic();self.commands.clear()
        # Fill the pose-history estimator before injecting the terminal event.
        until=time.monotonic()+.7;self.spin_until(lambda:time.monotonic()>=until)
        publisher=self.node.create_publisher(ExecutionState,'planning/execution_state',20)
        self.spin_until(lambda:publisher.get_subscription_count()>0)
        fault=copy.deepcopy(next(s for s in reversed(self.states) if s.task_id==task_id and s.trajectory_id==ack.trajectory_id))
        fault.state=ExecutionState.STATE_STALLED;fault.reason=reason;fault.terminal_error=.025
        publisher.publish(fault)
        self.spin_until(lambda:'TURN_RECERTIFICATION_WAIT_REST' in self.read_child_log(0))
        self.spin_until(lambda:any(s.task_id==task_id and s.reason=='TURN_RECERTIFICATION_REQUIRES_STOP' for s in self.states))
        return task,first

    def test_turn_fault_waits_for_rest_without_rebuilding_splines(self):
        task,first=self.begin_turn_fault_on_committed_task(8835,'HANDOFF_REST_STATE_CHANGED')
        count=sum(s.task_id==task.task_id for s in self.splines)
        until=time.monotonic()+.6;self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(sum(s.task_id==task.task_id for s in self.splines),count)
        self.pose_yaw_rate=0.;self.advance_yaw=False
        self.spin_until(lambda:any(s.task_id==task.task_id and s.traj_id>first.traj_id for s in self.splines),timeout=8)
        successor=next(s for s in self.splines if s.task_id==task.task_id and s.traj_id>first.traj_id)
        self.assertEqual(successor.request_id,task.request_id)
        self.assertEqual(successor.candidate_id,first.candidate_id)
        self.assertFalse(any(f.task_id==task.task_id and f.state in (ObservationFeedback.VIEWPOINT_REJECTED,ObservationFeedback.EXECUTION_BLOCKED) for f in self.feedback))
        self.assertEqual(self.read_child_log(0).count('TURN_RECERTIFICATION_WAIT_REST'),1)

    def test_turn_rest_timeout_reports_terminal_instead_of_interpolation_loop(self):
        task,first=self.begin_turn_fault_on_committed_task(8836,'HANDOFF_REST_UNCONFIRMED')
        count=sum(s.task_id==task.task_id for s in self.splines)
        self.spin_until(lambda:any(f.task_id==task.task_id and f.reason=='TURN_STOP_UNCONFIRMED' for f in self.feedback),timeout=8)
        self.assertEqual(sum(s.task_id==task.task_id for s in self.splines),count)
        self.assertIn('OBSERVATION_LOCAL_REJECTED',self.read_child_log(0))

    def test_original_laser_no_return_updates_local_patch(self):
        self.use_original_scan=True
        self.spin_until(lambda:self.scan_pub.get_subscription_count()>0)
        def at(patch,x,y):
            cx=math.floor((x-patch.origin.x)/patch.resolution)
            cy=math.floor((y-patch.origin.y)/patch.resolution)
            return patch.occupancy[cy*patch.width+cx] if 0<=cx<patch.width and 0<=cy<patch.height else None
        self.spin_until(lambda:any(at(p,2.,0.)==0 for p in self.patches),timeout=8)
        patch=self.patches[-1]
        self.assertEqual(at(patch,4.,0.),-1)
        self.assertNotIn(100,patch.occupancy)
        self.assertIn('[LASER_RAY_EVIDENCE]',self.read_child_log(0))

    def test_original_laser_no_return_updates_explorer_map(self):
        self.use_original_scan=True
        self.spin_until(lambda:self.scan_pub.get_subscription_count()>0)
        def at(grid,x,y):
            cx=math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
            cy=math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
            return grid.data[cy*grid.info.width+cx]
        self.spin_until(lambda:any(at(g,2.,0.)==0 for g in self.explorer_maps),timeout=8)
        grid=self.explorer_maps[-1]
        self.assertEqual(at(grid,4.,0.),-1)
        self.assertNotIn(100,grid.data)
        self.assertIn('[MAP_FUSION_STARTED]',self.read_child_log(2))

    def forward_turn_task(self,task_id):
        # The body fits heading zero here; the next goal's rotation does not.
        # A wider chamber starts beyond x=.5 and fits the whole capture/turn region.
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,11)]
            for y in (-.55,.55) for z in (.4,.7)]
        self.cloud_points += [(x,y,z) for x in [i*.05 for i in range(10,81)]
            for y in (-2.,2.) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-40,41)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>35)
        task=self.task(task_id);task.local_geometry_autonomy=True
        task.goal=Point(x=2.,y=1.,z=0.)
        task.coarse_route=[Point(x=0.,y=0.),Point(x=1.2,y=0.),task.goal]
        task.corridor_radius=1.2;task.target_cells=[Point(x=2.5,y=1.)]
        self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==task_id and s.yaw_pts and
            s.execution_mode==Bspline.MODE_FORWARD_TURN_STAGE for s in self.splines),timeout=12)
        return task,next(s for s in self.splines if s.task_id==task_id and s.yaw_pts)

    def test_narrow_observation_station_keeps_certified_forward_exit(self):
        # Body fits at yaw zero, but a full in-place return turn does not.
        # A station with a coherent forward stopping corridor remains usable.
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.50,.50) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-10,11)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>35)
        task=self.task(8883);task.local_geometry_autonomy=True
        self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8883 for s in self.splines),timeout=12)
        candidate=next(s for s in self.splines if s.task_id==8883)
        endpoint=(candidate.pos_pts[-3].x+4*candidate.pos_pts[-2].x+candidate.pos_pts[-1].x)/6
        self.assertGreaterEqual(endpoint,1.19)  # Inside the actual dwell zone.
        self.assertTrue(candidate.use_measured_progress)
        self.assertFalse(candidate.yaw_pts)
        self.assertNotIn('TASK_TERMINAL_EXIT_REJECTED',self.read_child_log(0))

    def test_installation_rejection_retries_same_station_without_blacklist(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8884);task.local_geometry_autonomy=True
        self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8884 for s in self.splines),timeout=8)
        first=next(s for s in self.splines if s.task_id==8884)
        reject=TrajectoryAck(request_id=first.request_id,task_id=first.task_id,
            task_version=first.task_version,candidate_id=first.candidate_id,
            trajectory_id=first.traj_id,status=TrajectoryAck.STATUS_REJECTED,
            reason='HANDOFF_POSITION_ERROR')
        self.ack_pub.publish(reject)
        self.spin_until(lambda:any(s.task_id==8884 and s.traj_id>first.traj_id for s in self.splines),timeout=8)
        second=next(s for s in self.splines if s.task_id==8884 and s.traj_id>first.traj_id)
        def endpoint(s):
            return ((s.pos_pts[-3].x+4*s.pos_pts[-2].x+s.pos_pts[-1].x)/6,
                    (s.pos_pts[-3].y+4*s.pos_pts[-2].y+s.pos_pts[-1].y)/6)
        a,b=endpoint(first),endpoint(second)
        self.assertAlmostEqual(a[0],b[0],delta=.02)
        self.assertAlmostEqual(a[1],b[1],delta=.02)
        self.assertEqual(first.candidate_id,second.candidate_id)
        self.assertEqual(first.request_id,second.request_id)
        self.assertIn('reason=CONTROLLER_REJECTED_HANDOFF_POSITION_ERROR',self.read_child_log(0))
        self.controller_spline_pub.publish(second)
        self.spin_until(lambda:any(a.trajectory_id==second.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))

    def test_local_transit_extends_before_terminal_capture(self):
        self.cloud_points=[(float(x),y,z) for x in range(-4,16)
            for y in (-2.,2.) for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8885);task.local_geometry_autonomy=True
        task.goal=Point(x=10.,y=0.);task.coarse_route=[Point(x=0.,y=0.),task.goal]
        task.target_cells=[Point(x=11.,y=0.)]
        self.auto_ack_request=task.request_id;self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==8885 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback),timeout=8)
        first=next(s for s in self.splines if s.task_id==8885)
        first_end=(first.pos_pts[-3].x+4*first.pos_pts[-2].x+first.pos_pts[-1].x)/6
        self.advance_commands=True;self.pose_last_wall=time.monotonic()
        self.spin_until(lambda:any(s.task_id==8885 and
            (s.pos_pts[-3].x+4*s.pos_pts[-2].x+s.pos_pts[-1].x)/6>first_end+.5
            for s in self.splines),timeout=15)
        self.assertGreater(first_end-self.pose_x,.20)
        self.assertFalse(any(s.task_id==8885 and s.trajectory_id==first.traj_id and
            s.reason in ('TERMINAL_SETTLING','TERMINAL_REACHED') for s in self.states))

    def forward_stage_fixture(self, identity):
        msg=self.stationary(identity,self.node.get_clock().now().nanoseconds,identity)
        msg.pos_pts=[Point(x=d,y=0.,z=.4) for d in (-.4,0.,.4,.8)]
        msg.execution_mode=Bspline.MODE_FORWARD_TURN_STAGE
        msg.yaw_pts=[0.];msg.yaw_dt=1.
        msg.use_measured_progress=True
        msg.min_forward_command=.30;msg.min_forward_effective_speed=.165
        msg.max_forward_speed=.30;msg.max_lateral_speed=.08
        msg.max_yaw_rate=.10;msg.max_tracking_correction=.08
        msg.forward_stage_stop_latency=.55;msg.forward_stage_deceleration=.32
        return msg

    def test_short_forward_stage_brakes_before_point_and_finishes_at_certified_stopped_pose(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.forward_stage_fixture(8860);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8860 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear();self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        self.pose_x=.23;self.pose_vx=.2
        self.spin_until(lambda:any(s.trajectory_id==8860 and s.reason=='FORWARD_STAGE_BRAKING' for s in self.states))
        self.commands.clear();self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.pose_x=.29;self.pose_vx=0.
        self.spin_until(lambda:any(s.trajectory_id==8860 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_x=.52;self.commands.clear()
        self.spin_until(lambda:any(s.trajectory_id==8860 and s.reason=='FORWARD_STAGE_OVERSHOOT' for s in self.states))
        self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_forward_stage_cross_track_drift_latches_zero_before_reaching_wall(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.forward_stage_fixture(8861);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8861 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear();self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        self.pose_x=.25;self.pose_y=.09;self.commands.clear()
        self.spin_until(lambda:any(s.trajectory_id==8861 and s.reason=='FORWARD_STAGE_CROSS_TRACK_DRIFT' for s in self.states))
        self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        self.pose_y=0.;self.commands.clear();self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_early_stage_stop_outside_capture_is_failure_instead_of_fake_arrival(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.forward_stage_fixture(8862);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8862 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear();self.spin_until(lambda:any(c.linear.x>.05 for c in self.commands))
        self.pose_x=.1;self.pose_vx=.3
        self.spin_until(lambda:any(s.trajectory_id==8862 and s.reason=='FORWARD_STAGE_BRAKING' for s in self.states))
        self.pose_vx=0.;self.commands.clear()
        self.spin_until(lambda:any(s.trajectory_id==8862 and s.reason=='FORWARD_STAGE_STOP_SHORT' for s in self.states))
        self.assertFalse(any(s.trajectory_id==8862 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.spin_until(lambda:len(self.commands)>10)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_stage_failure_feedback_exits_stage_ownership_and_waits_for_actual_rest(self):
        task,stage=self.forward_turn_task(8863)
        self.auto_ack_request=task.request_id
        self.spin_until(lambda:any('PATH_TRAJECTORY_READY' in s and f'request_id={task.request_id}' in s for s in self.statuses))
        self.pose_yaw_rate=.21
        self.spin_until(lambda:len(self.commands)>10)
        fault=ExecutionState(request_id=task.request_id,task_id=task.task_id,task_version=1,
            trajectory_id=stage.traj_id,state=ExecutionState.STATE_STALLED,
            reason='FORWARD_STAGE_CROSS_TRACK_DRIFT',terminal_error=.09)
        pub=self.node.create_publisher(ExecutionState,'planning/execution_state',20)
        pub.publish(fault)
        self.spin_until(lambda:'reason=FORWARD_STAGE_CROSS_TRACK_DRIFT' in self.read_child_log(0))
        before=len(self.splines);end=time.monotonic()+.3
        self.spin_until(lambda:time.monotonic()>=end)
        self.assertEqual(len(self.splines),before)
        self.assertNotIn('TASK_TURN_STAGE_REACHED',self.read_child_log(0))
        self.pose_yaw_rate=0.
        self.spin_until(lambda:'[TASK_RESTART_FROM_REST]' in self.read_child_log(0),timeout=8)

    def test_task_drives_to_turn_stage_and_replans_only_after_arrival_and_rest(self):
        task,stage=self.forward_turn_task(8811)
        self.assertAlmostEqual(stage.yaw_pts[0],0.,places=6)
        self.assertTrue(stage.handoff_certified)
        stage_end_x=(stage.pos_pts[-3].x+4*stage.pos_pts[-2].x+stage.pos_pts[-1].x)/6
        self.assertGreater(stage_end_x-.15,.5)  # Entire capture interval enters the chamber.
        # No exact ACK: a staging flag must not bypass pending handoff handling.
        until=time.monotonic()+.25
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertNotIn('TASK_TURN_STAGE_REACHED',self.read_child_log(0))
        self.auto_ack_request=task.request_id
        self.advance_commands=True;self.pose_last_wall=time.monotonic()
        self.spin_until(lambda:'TASK_TURN_STAGE_REACHED' in self.read_child_log(0),timeout=12)
        self.advance_commands=False;self.pose_vx=self.pose_vy=0.
        captured_x=self.pose_x
        self.assertLessEqual(abs(captured_x-stage_end_x),.15)
        self.assertGreater(captured_x,.5)
        self.assertAlmostEqual(self.pose_yaw,0.)
        self.assertFalse(any(abs(c.angular.z)>.01 for c in self.commands))
        # The original information task persists; its next path is recertified
        # from the actual stopped chamber pose, not counted as a station visit.
        self.spin_until(lambda:any(s.task_id==8811 and s.traj_id!=stage.traj_id and
            not s.yaw_pts for s in self.splines),timeout=12)
        following=next(s for s in self.splines if s.task_id==8811 and
            s.traj_id!=stage.traj_id and not s.yaw_pts)
        self.assertEqual(following.request_id,task.request_id)
        self.assertAlmostEqual(following.handoff_position.x,captured_x,delta=.02)
        self.assertGreater(abs(following.handoff_target_yaw),.1)
        self.assertFalse(any(f.request_id==task.request_id and f.state in
            (ObservationFeedback.STATION_REACHED,ObservationFeedback.VIEWPOINT_REJECTED) for f in self.feedback))

    def test_prepared_successor_cannot_interrupt_straight_turn_stage(self):
        task,stage=self.forward_turn_task(8813)
        self.auto_ack_request=task.request_id
        self.spin_until(lambda:any('PATH_TRAJECTORY_READY' in s and
            f'request_id={task.request_id}' in s for s in self.statuses))
        successor=self.task(8814)
        successor.goal=Point(x=2.,y=1.)
        successor.coarse_route=[Point(x=0.,y=0.),Point(x=1.2,y=0.),successor.goal]
        successor.corridor_radius=1.2;successor.local_geometry_autonomy=True
        successor.target_cells=[Point(x=2.5,y=1.)]
        successor.predecessor_task_id=task.task_id;successor.predecessor_task_version=1
        self.task_pub.publish(successor)
        self.advance_commands=True;self.pose_last_wall=time.monotonic()
        until=time.monotonic()+.6
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertFalse(any(s.task_id==8814 for s in self.splines))
        self.assertTrue(any(f.task_id==8814 and f.request_id==successor.request_id and
            f.state==ObservationFeedback.TRANSITION_WAIT_STOP and
            f.reason=='WAITING_FOR_TURN_STAGE' for f in self.feedback))
        self.assertAlmostEqual(self.pose_yaw,0.)
        self.spin_until(lambda:'TASK_TURN_STAGE_REACHED' in self.read_child_log(0),timeout=12)
        self.advance_commands=False;self.pose_vx=self.pose_vy=0.
        self.auto_ack_request=successor.request_id
        self.spin_until(lambda:any(s.task_id==8814 for s in self.splines),timeout=12)
        candidate=next(s for s in self.splines if s.task_id==8814)
        stage_end_x=(stage.pos_pts[-3].x+4*stage.pos_pts[-2].x+stage.pos_pts[-1].x)/6
        self.assertLessEqual(abs(candidate.handoff_position.x-stage_end_x),.17)
        self.assertGreater(candidate.handoff_position.x,.5)
        self.assertNotEqual(candidate.traj_id,stage.traj_id)

    def test_turn_stage_ack_timeout_keeps_reconciliation_runnable(self):
        task,stage=self.forward_turn_task(8812)
        self.spin_until(lambda:'TRAJECTORY_ACK_TIMEOUT' in self.read_child_log(0),timeout=8)
        self.assertNotIn('TASK_TURN_STAGE_REACHED',self.read_child_log(0))
        self.assertFalse(any(f.request_id==task.request_id and f.state==ObservationFeedback.STATION_REACHED
                             for f in self.feedback))

    def test_retimed_prefix_drift_does_not_invent_sideways_turn(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8400,self.node.get_clock().now().nanoseconds,8400)
        msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (-.001,0.,.001,0.,0.,1.,2.,3.)]
        msg.knots=[float(15*i) for i in range(-3,9)]
        msg.execution_mode=Bspline.MODE_CLEARANCE_LIMITED
        msg.use_measured_progress=True
        msg.min_forward_command=.3;msg.min_forward_effective_speed=.165
        msg.max_forward_speed=.3;msg.max_lateral_speed=.08;msg.max_yaw_rate=.5
        msg.max_tracking_correction=.08
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8400 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks))
        self.pose_y=.015;self.commands.clear()
        until=time.monotonic()+.7
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertFalse(any(s.trajectory_id==8400 and s.state==ExecutionState.STATE_HEADING_FROZEN
            for s in self.states))
        self.assertTrue(self.commands)
        self.assertTrue(all(abs(c.angular.z)<.3 for c in self.commands))

    def test_later_large_turn_stops_before_any_uncertified_rotation(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8401,self.node.get_clock().now().nanoseconds,8401)
        msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (-.3,0.,.3,.6)]
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8401 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks))
        self.pose_yaw=math.pi;self.commands.clear()
        self.spin_until(lambda:any(s.trajectory_id==8401 and s.reason=='UNCERTIFIED_TURN_REQUIRED'
            for s in self.states))
        self.commands.clear()
        self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.angular.z)+abs(c.linear.x)+abs(c.linear.y)<1e-9
            for c in self.commands))

    def side_step_fixture(self,traj_id):
        msg=self.stationary(traj_id,self.node.get_clock().now().nanoseconds,traj_id)
        msg.pos_pts=[Point(x=0.,y=y,z=.4) for y in (-.15,0.,.15,.30)]
        msg.knots=[float(10*i) for i in range(-3,5)]
        msg.execution_mode=Bspline.MODE_RECOVERY_PRIMITIVE
        msg.yaw_pts=[0.]*4;msg.yaw_dt=10.
        msg.max_forward_speed=.3;msg.max_lateral_speed=.3;msg.max_yaw_rate=.5
        return msg

    def test_side_capture_finishes_by_measured_progress_and_stays_stopped(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.side_step_fixture(8402);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8402 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.spin_until(lambda:any(c.linear.y>.29 for c in self.commands))
        self.pose_x=.08;self.pose_y=.15
        self.spin_until(lambda:any(s.trajectory_id==8402 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_y=.05;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_side_stop_rollback_is_corrected_before_reporting_finished(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.side_step_fixture(8404);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8404 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear()
        self.spin_until(lambda:any(abs(c.linear.y)>.05 for c in self.commands))
        self.pose_y=.15
        self.spin_until(lambda:any(s.trajectory_id==8404 and s.reason=='LATERAL_CAPTURE_SETTLING' for s in self.states))
        self.pose_y=.07;self.commands.clear()
        self.spin_until(lambda:any(c.linear.y>.29 for c in self.commands),timeout=4)
        self.assertFalse(any(s.trajectory_id==8404 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        before=len(self.states);self.pose_y=.24
        self.spin_until(lambda:any(s.trajectory_id==8404 and s.reason=='LATERAL_CAPTURE_SETTLING' for s in self.states[before:]))
        self.pose_y=.15
        self.spin_until(lambda:any(s.trajectory_id==8404 and s.state==ExecutionState.STATE_FINISHED for s in self.states))

    def test_lateral_deviation_latches_until_a_new_certificate(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.side_step_fixture(8403);self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8403 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.commands.clear()
        self.spin_until(lambda:any(abs(c.linear.y)>.05 for c in self.commands))
        self.pose_x=.11
        self.spin_until(lambda:any(s.trajectory_id==8403 and s.reason=='LATERAL_RECOVERY_LONGITUDINAL_DRIFT' for s in self.states))
        self.pose_x=0.;self.commands.clear()
        self.spin_until(lambda:len(self.commands)>8)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
        expected=math.hypot(self.pose_x, self.pose_y-.15)
        self.spin_until(lambda:any(s.trajectory_id==8403 and abs(s.terminal_error-expected)<.005
            for s in self.states))
        self.assertAlmostEqual(self.states[-1].terminal_error,expected,delta=.005)

    def test_controller_uses_certified_match_instead_of_rematching(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8300,self.node.get_clock().now().nanoseconds,8300)
        msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (-.3,0.,.3,.6)]
        msg.handoff_matched_time=.4  # Independent closest match would be t=0.
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8300 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==8300)
        self.assertAlmostEqual(ack.matched_time,.4,places=9)

    def test_invalid_handoff_certificate_preserves_installed_execution(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        hold=self.stationary(8301,self.node.get_clock().now().nanoseconds,8301)
        self.spline_pub.publish(hold)
        self.spin_until(lambda:any(a.trajectory_id==8301 and a.status==TrajectoryAck.STATUS_HOLD_INSTALLED
            for a in self.acks))
        for index,reason in enumerate(('HANDOFF_CERTIFICATE_MISSING',
                'HANDOFF_CERTIFICATE_STATE_CHANGED','HANDOFF_CERTIFICATE_HEADING_CHANGED')):
            msg=self.stationary(8301,hold.request_id+1,8302+index)
            msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (-.3,0.,.3,.6)]
            if index==0:msg.handoff_certified=False
            if index==1:msg.handoff_position.y=.021
            if index==2:msg.handoff_target_yaw=math.pi
            self.spline_pub.publish(msg)
            self.spin_until(lambda:any(a.trajectory_id==msg.traj_id and
                a.status==TrajectoryAck.STATUS_REJECTED and a.reason==reason for a in self.acks))
            self.assertFalse(any(s.trajectory_id==msg.traj_id for s in self.states))
        self.assertEqual(self.states[-1].trajectory_id,hold.traj_id)

    def test_certified_initial_turn_stops_when_position_deviates(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8310,self.node.get_clock().now().nanoseconds,8310)
        msg.pos_pts=[Point(x=0.,y=y,z=.4) for y in (-.3,0.,.3,.6)]
        msg.handoff_target_yaw=math.pi/2
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(s.trajectory_id==8310 and s.state==ExecutionState.STATE_HEADING_FROZEN
            for s in self.states))
        self.spin_until(lambda:any(c.angular.z>0 and abs(c.linear.x)<1e-9 and abs(c.linear.y)<1e-9
            for c in self.commands))
        self.pose_x=.03
        self.spin_until(lambda:any(s.trajectory_id==8310 and s.reason=='HANDOFF_ALIGNMENT_DEVIATED'
            for s in self.states))
        self.commands.clear()
        self.spin_until(lambda:len(self.commands)>5)
        self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))

    def test_latest_map_vetoes_remaining_turn_without_linear_motion(self):
        refreshes=[]
        self.node.create_subscription(ExecutionCommand,"planning/execution_command",refreshes.append,20)
        self.pose_yaw=math.pi
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8312)
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==8312 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks),timeout=12)
        ack=next(a for a in self.acks if a.task_id==8312 and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.spin_until(lambda:any(s.traj_id==ack.trajectory_id for s in self.splines))
        candidate=next(s for s in self.splines if s.traj_id==ack.trajectory_id)
        self.assertTrue(candidate.handoff_certified)
        self.assertAlmostEqual(ack.matched_time,candidate.handoff_matched_time,places=8)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(s.trajectory_id==ack.trajectory_id and
            s.state==ExecutionState.STATE_HEADING_FROZEN for s in self.states))
        self.spin_until(lambda:any(c.command==ExecutionCommand.COMMAND_REFRESH_TURN and
            c.trajectory_id==ack.trajectory_id for c in refreshes))
        # Horizontal body and straight path still fit. Intermediate vertical
        # body in the pending pi turn does not. No linear braking is present.
        self.cloud_points=[(x,-.45,z) for x in [i*.05 for i in range(-40,81)]
            for z in (.4,.7)]
        self.spin_until(lambda:'reason=remaining_handoff_rotation' in self.read_child_log(0),timeout=8)
        self.assertIn('HEADING_ALIGNMENT_SWEEP_BLOCKED',self.read_child_log(0))
        self.spin_until(lambda:any(s.trajectory_id==ack.trajectory_id and s.state==ExecutionState.STATE_SOFT_HOLD
            for s in self.states))

    def test_replan_failure_keeps_scheduled_lateral_recovery(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8311)
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==8311 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks),timeout=10)
        ack=next(a for a in self.acks if a.task_id==8311 and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==8311 and f.state==ObservationFeedback.EXECUTION_READY
            for f in self.feedback))
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,1.20) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,25)] for z in (.4,.7)]
        boundary=self.cloud_counter+35
        self.spin_until(lambda:self.cloud_counter>boundary)
        # The new wall triggers the actual preferred-clearance rolling
        # replan. Do not overwrite its outcome with a synthetic stale stall.
        self.spin_until(lambda:any(s.task_id==8311 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE
            and len(s.yaw_pts)==1 for s in self.splines),timeout=12)
        self.assertFalse(any(f.task_id==8311 and f.state==ObservationFeedback.LOCAL_MOTION_EXHAUSTED
            for f in self.feedback))
        log=self.read_child_log(0)
        self.assertIn('from EXEC_TRAJ to REPLAN_TRAJ',log)
        self.assertIn('TASK_LATERAL_RECOVERY_WAIT_STOP',log)
        self.assertNotIn('OBSERVATION_LOCAL_REJECTED',log)

    def test_short_recovery_does_not_finish_after_only_two_centimetres(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8201,self.node.get_clock().now().nanoseconds,8201)
        msg.pos_pts=[Point(x=0.,y=y,z=.4) for y in (-.15,0.,.15,.30)]
        msg.execution_mode=Bspline.MODE_RECOVERY_PRIMITIVE
        msg.yaw_pts=[0.];msg.yaw_dt=1.
        msg.max_forward_speed=.30;msg.max_lateral_speed=.15
        msg.max_yaw_rate=.50;msg.max_tracking_correction=.08
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==8201 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.pose_y=.02
        self.spin_until(lambda:any(s.trajectory_id==8201 and s.state==ExecutionState.STATE_STALLED for s in self.states))
        self.assertFalse(any(s.trajectory_id==8201 and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.pose_y=.15
        self.spin_until(lambda:any(s.trajectory_id==8201 and s.state==ExecutionState.STATE_FINISHED for s in self.states))

    def test_fixed_yaw_recovery_keeps_pure_lateral_command(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.pose_yaw=0.0
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8101,self.node.get_clock().now().nanoseconds,801)
        msg.pos_pts=[Point(x=0.,y=y,z=.4) for y in (-.1,0.,.1,.2)]
        msg.execution_mode=Bspline.MODE_RECOVERY_PRIMITIVE
        msg.yaw_pts=[0.];msg.yaw_dt=1.
        msg.max_forward_speed=.30;msg.max_lateral_speed=.15
        msg.max_yaw_rate=.50;msg.max_tracking_correction=.08
        self.commands.clear()
        self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==801 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks))
        self.spin_until(lambda:any(c.linear.y>.02 for c in self.commands))
        moving=[c for c in self.commands if abs(c.linear.y)>.02]
        self.assertTrue(moving)
        self.assertTrue(all(abs(c.linear.x)<1e-9 for c in moving))
        self.assertTrue(all(abs(c.angular.z)<1e-6 for c in moving))
        self.assertTrue(all(abs(c.linear.y)<=.15+1e-9 for c in moving))

    def test_fixed_yaw_recovery_uses_body_lateral_axis_at_rotated_heading(self):
        self.pose_yaw=math.pi/2
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        msg=self.stationary(8102,self.node.get_clock().now().nanoseconds,802)
        msg.pos_pts=[Point(x=x,y=0.,z=.4) for x in (.1,0.,-.1,-.2)]
        msg.execution_mode=Bspline.MODE_RECOVERY_PRIMITIVE
        msg.yaw_pts=[self.pose_yaw];msg.yaw_dt=1.
        msg.max_forward_speed=.30;msg.max_lateral_speed=.15
        msg.max_yaw_rate=.50;msg.max_tracking_correction=.08
        self.commands.clear();self.spline_pub.publish(msg)
        self.spin_until(lambda:any(a.trajectory_id==802 and a.status==TrajectoryAck.STATUS_ACTIVATED
            for a in self.acks))
        self.spin_until(lambda:any(c.linear.y>.02 for c in self.commands))
        moving=[c for c in self.commands if c.linear.y>.02]
        self.assertTrue(all(abs(c.linear.x)<1e-9 for c in moving))
        self.assertTrue(all(abs(c.angular.z)<1e-6 for c in moving))

    def test_task_departure_failure_prepares_lateral_recovery_without_exhausting_task(self):
        # A narrow wall on the right forbids rotate-then-drive departures;
        # fixed-yaw translation to the left has room and a straight rejoin.
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,1.20) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,25)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8103)
        self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8103 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE
            and len(s.yaw_pts)==1 for s in self.splines),timeout=12)
        recovery=next(s for s in self.splines if s.task_id==8103 and
            s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE and len(s.yaw_pts)==1)
        self.assertEqual(recovery.request_id,task.request_id)
        self.assertEqual(recovery.min_forward_command,0.)
        self.assertEqual(recovery.min_forward_effective_speed,0.)
        self.assertAlmostEqual(recovery.yaw_pts[0],0.,places=4)
        self.assertAlmostEqual(recovery.max_lateral_speed,.30)
        self.assertGreater(recovery.pos_pts[-1].y,recovery.pos_pts[0].y)
        self.assertFalse(any(f.task_id==8103 and f.state in (
            ObservationFeedback.LOCAL_MOTION_EXHAUSTED,ObservationFeedback.STATION_REACHED)
            for f in self.feedback))
        # The planner must still wait for this exact controller activation.
        self.assertFalse(any(f.task_id==8103 and f.state==ObservationFeedback.EXECUTION_READY
            for f in self.feedback))
        self.spin_until(lambda:any(a.trajectory_id==recovery.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==recovery.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==8103 and f.state==ObservationFeedback.EXECUTION_READY
            for f in self.feedback))
        self.pose_last_wall=time.monotonic()
        self.advance_commands=True
        self.spin_until(lambda:any(s.task_id==8103 and s.traj_id>recovery.traj_id and
            s.execution_mode!=Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),timeout=10)
        rejoin=next(s for s in self.splines if s.task_id==8103 and s.traj_id>recovery.traj_id and
            s.execution_mode!=Bspline.MODE_RECOVERY_PRIMITIVE)
        self.assertEqual(rejoin.request_id,task.request_id)
        self.assertEqual(rejoin.task_version,task.task_version)
        self.assertFalse(rejoin.yaw_pts)
        self.assertGreater(self.pose_y,.05)
        self.assertFalse(any(f.task_id==8103 and f.state in (
            ObservationFeedback.LOCAL_MOTION_EXHAUSTED,ObservationFeedback.STATION_REACHED)
            for f in self.feedback))
        self.spin_until(lambda:any(a.trajectory_id==rejoin.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        rejoin_ack=next(a for a in self.acks if a.trajectory_id==rejoin.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(rejoin_ack)
        self.spin_until(lambda:'STRUCTURED_LOCAL_REPAIR_COMPLETE' in self.read_child_log(0))


    def activate_recovery_for_failed_rejoin(self,task_id,manual_controller=False):
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,1.20) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,25)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(task_id);self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==task_id and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),timeout=12)
        recovery=next(s for s in self.splines if s.task_id==task_id and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE)
        if manual_controller:
            self.spin_until(lambda:self.controller_spline_pub.get_subscription_count()>0)
            self.controller_spline_pub.publish(recovery)
        self.spin_until(lambda:any(a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==task_id and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.commands.clear()
        self.spin_until(lambda:any(abs(c.linear.y)>.05 for c in self.commands))
        # Inject drift only after the side step has actually begun; ACTIVATED
        # acknowledges installation, not physical departure.
        # Synthetic measured pose models useful transverse movement followed
        # by the field's ~0.11 rad body-yaw deviation. Local must wait for its
        # own fresh rest evidence, not treat these inputs as a physics test.
        self.pose_y=.15;self.pose_yaw=.11
        self.spin_until(lambda:any(s.trajectory_id==recovery.traj_id and s.reason=='LATERAL_RECOVERY_HEADING_DRIFT' for s in self.states))
        return task,recovery

    def stopped_rejoin_candidate(self,task,recovery):
        self.spin_until(lambda:any(s.task_id==task.task_id and s.traj_id>recovery.traj_id and s.execution_mode!=Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),timeout=8)
        return next(s for s in self.splines if s.task_id==task.task_id and s.traj_id>recovery.traj_id and s.execution_mode!=Bspline.MODE_RECOVERY_PRIMITIVE)

    def test_failed_side_step_certifies_actual_pose_and_waits_for_exact_ack(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8601)
        rejoin=self.stopped_rejoin_candidate(task,recovery)
        self.assertEqual(rejoin.request_id,task.request_id)
        self.assertEqual(rejoin.task_version,task.task_version)
        self.assertAlmostEqual(rejoin.handoff_body_yaw,.11,places=4)
        self.assertAlmostEqual(rejoin.handoff_position.y,.15,places=4)
        self.assertTrue(rejoin.handoff_certified)
        self.assertFalse(any(s.trajectory_id==recovery.traj_id and s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.assertFalse(any(f.task_id==task.task_id and f.state in (ObservationFeedback.STATION_REACHED,ObservationFeedback.LOCAL_MOTION_EXHAUSTED) for f in self.feedback))
        self.assertNotIn('STRUCTURED_LOCAL_REPAIR_COMPLETE',self.read_child_log(0))
        self.spin_until(lambda:any(a.trajectory_id==rejoin.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==rejoin.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
        stale=copy.deepcopy(ack);stale.request_id+=1;self.ack_pub.publish(stale)
        until=time.monotonic()+.15;self.spin_until(lambda:time.monotonic()>=until)
        self.assertNotIn('STRUCTURED_LOCAL_REPAIR_COMPLETE',self.read_child_log(0))
        self.ack_pub.publish(ack)
        self.spin_until(lambda:'STRUCTURED_LOCAL_REPAIR_COMPLETE' in self.read_child_log(0))
        self.assertEqual(sum(s.task_id==task.task_id and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),1)
        self.assertIn('reason=LATERAL_RECOVERY_HEADING_DRIFT',self.read_child_log(1))
        self.assertIn('yaw_error=',self.read_child_log(1))

    def test_failed_side_step_waits_for_actual_rest_before_rejoin(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8602)
        self.pose_vx=.08;self.advance_pose=True
        until=time.monotonic()+.6;self.spin_until(lambda:time.monotonic()>=until)
        self.assertFalse(any(s.task_id==task.task_id and s.traj_id>recovery.traj_id for s in self.splines))
        self.advance_pose=False;self.pose_vx=0.
        rejoin=self.stopped_rejoin_candidate(task,recovery)
        self.assertAlmostEqual(rejoin.handoff_position.x,self.pose_x,places=3)
        self.assertIn('FAILED_RECOVERY_STOPPED_REJOIN',self.read_child_log(0))

    def test_failed_recovery_rejoin_rejection_is_terminal(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8603,True)
        rejoin=self.stopped_rejoin_candidate(task,recovery)
        rejection=TrajectoryAck(task_id=task.task_id,task_version=task.task_version,
            request_id=task.request_id,trajectory_id=rejoin.traj_id,candidate_id=rejoin.candidate_id,
            status=TrajectoryAck.STATUS_REJECTED,reason='HANDOFF_CERTIFICATE_POSITION_ERROR')
        self.ack_pub.publish(rejection)
        self.spin_until(lambda:any(f.task_id==task.task_id and f.reason=='LATERAL_RECOVERY_REJOIN_REJECTED' for f in self.feedback))
        until=time.monotonic()+.4;self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(sum(s.task_id==task.task_id and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),1)
        self.assertEqual(sum(s.task_id==task.task_id and s.traj_id>recovery.traj_id for s in self.splines),1)

    def test_failed_recovery_rejoin_ack_timeout_does_not_restart_escape(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8604,True)
        self.stopped_rejoin_candidate(task,recovery)
        self.spin_until(lambda:any(f.task_id==task.task_id and f.reason in (
            'LATERAL_RECOVERY_REJOIN_ACK_TIMEOUT','LATERAL_RECOVERY_REJOIN_ACK_UNCONFIRMED') for f in self.feedback),timeout=8)
        self.assertEqual(sum(s.task_id==task.task_id and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines),1)
        self.assertEqual(sum(s.task_id==task.task_id and s.traj_id>recovery.traj_id for s in self.splines),1)

    def test_failed_side_step_without_confirmed_rest_has_no_rejoin(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8605)
        self.pose_vx=.015;self.advance_pose=True
        self.spin_until(lambda:any(f.task_id==task.task_id and f.reason=='LATERAL_RECOVERY_STOP_UNCONFIRMED' for f in self.feedback),timeout=8)
        self.advance_pose=False;self.pose_vx=0.
        self.assertFalse(any(s.task_id==task.task_id and s.traj_id>recovery.traj_id for s in self.splines))
        self.assertNotIn('FAILED_RECOVERY_STOPPED_REJOIN',self.read_child_log(0))

    def test_cancelled_failed_rejoin_releases_ownership_and_ignores_late_ack(self):
        task,recovery=self.activate_recovery_for_failed_rejoin(8606)
        rejoin=self.stopped_rejoin_candidate(task,recovery)
        self.spin_until(lambda:any(a.trajectory_id==rejoin.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        late=copy.deepcopy(next(a for a in self.acks if a.trajectory_id==rejoin.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED))
        cancel=copy.deepcopy(task);cancel.lifecycle=ObservationTask.CANCELLED;self.task_pub.publish(cancel)
        self.spin_until(lambda:any(a.task_id==task.task_id and a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        retired=next(a for a in self.acks if a.task_id==task.task_id and a.status==TrajectoryAck.STATUS_TASK_RETIRED)
        self.ack_pub.publish(retired)
        self.spin_until(lambda:any(f.task_id==task.task_id and f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        self.ack_pub.publish(late)
        successor=self.task(8607);self.task_pub.publish(successor)
        self.spin_until(lambda:any(s.task_id==successor.task_id for s in self.splines),timeout=8)
        next_spline=next(s for s in self.splines if s.task_id==successor.task_id)
        self.assertNotEqual(next_spline.execution_mode,Bspline.MODE_RECOVERY_PRIMITIVE)
        self.assertNotIn('STRUCTURED_LOCAL_REPAIR_COMPLETE',self.read_child_log(0))

    def test_lateral_first_stop_logs_specific_limit_and_keeps_reason(self):
        self.spin_until(lambda:self.spline_pub.get_subscription_count()>0)
        self.spin_until(lambda:len(self.states)>0)
        for traj_id,x,y,yaw,reason in (
            (8701,0.,.26,0.,'LATERAL_RECOVERY_OVERSHOOT'),
            (8702,.11,0.,0.,'LATERAL_RECOVERY_LONGITUDINAL_DRIFT'),
            (8703,0.,.15,.11,'LATERAL_RECOVERY_HEADING_DRIFT')):
            self.pose_x=0.;self.pose_y=0.;self.pose_yaw=0.
            self.spin_until(lambda:len(self.commands)>0)
            msg=self.side_step_fixture(traj_id);self.spline_pub.publish(msg)
            self.spin_until(lambda:any(a.trajectory_id==traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
            self.commands.clear()
            self.spin_until(lambda:any(abs(c.linear.y)>.05 for c in self.commands))
            self.pose_x=x;self.pose_y=y;self.pose_yaw=yaw
            self.spin_until(lambda:any(s.trajectory_id==traj_id and s.reason==reason for s in self.states))
            state=next(s for s in self.states if s.trajectory_id==traj_id and s.reason==reason)
            self.assertTrue(math.isfinite(state.terminal_error))
            self.pose_x=0.;self.pose_y=0.;self.pose_yaw=0.;self.commands.clear()
            self.spin_until(lambda:len(self.commands)>8)
            self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-9 for c in self.commands))
            self.assertEqual(self.read_child_log(1).count('reason='+reason+' request='),1)

    def test_ineffective_side_step_is_not_reopened_by_a_sibling_task(self):
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,1.20) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,25)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8202)
        self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8202 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE
            for s in self.splines),timeout=12)
        recovery=next(s for s in self.splines if s.task_id==8202 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE)
        self.spin_until(lambda:any(a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==8202 and f.reason=='LATERAL_RECOVERY_REJOIN_FAILED'
            for f in self.feedback),timeout=15)
        feedback=next(f for f in self.feedback if f.task_id==8202 and f.reason=='LATERAL_RECOVERY_REJOIN_FAILED')
        self.assertTrue(feedback.trigger_reason)
        self.assertEqual(feedback.recovery_reason,'EXECUTION_NO_PROGRESS')
        self.assertEqual(feedback.planning_reason,'DEPARTURE_UNSAFE')
        self.assertIn('cause=DEPARTURE_UNSAFE terminal=LATERAL_RECOVERY_REJOIN_FAILED',self.read_child_log(0))
        self.assertEqual(feedback.failure_scope,ObservationFeedback.FAILURE_ACTION)
        self.assertFalse(any(s.task_id==8202 and s.traj_id>recovery.traj_id for s in self.splines))
        self.assertEqual(self.read_child_log(0).count('[FAILED_RECOVERY_STOPPED_REJOIN]'),1)
        self.assertIn('FAILED_RECOVERY_REJOIN_EXHAUSTED',self.read_child_log(0))
        self.assertNotIn('[FSM]: from WAIT_TARGET to REPLAN_TRAJ',self.read_child_log(0))
        # Install a nearby successor after exact retirement; actual pose and map
        # stay fixed. This must not purchase a second side-step family.
        sibling=self.task(8203);sibling.goal.x=2.2;sibling.coarse_route[-1]=sibling.goal
        self.task_pub.publish(sibling)
        self.spin_until(lambda:any(a.task_id==8202 and a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        retired=next(a for a in self.acks if a.task_id==8202 and a.status==TrajectoryAck.STATUS_TASK_RETIRED)
        self.ack_pub.publish(retired)
        self.spin_until(lambda:any(f.task_id==8203 and f.state in (
            ObservationFeedback.EXECUTION_BLOCKED,ObservationFeedback.PATH_REJECTED,
            ObservationFeedback.LOCAL_MOTION_EXHAUSTED) for f in self.feedback),timeout=12)
        self.assertFalse(any(s.task_id==8203 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines))

    def test_recovery_finished_feedback_does_not_hide_measured_rollback(self):
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,1.20) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,25)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8204);self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8204 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE
            for s in self.splines),timeout=12)
        recovery=next(s for s in self.splines if s.task_id==8204 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE)
        self.spin_until(lambda:any(a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.trajectory_id==recovery.traj_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==8204 and f.state==ObservationFeedback.EXECUTION_READY
            for f in self.feedback))
        # Fault injection: terminal feedback describes a transient arrival,
        # while fresh settled odometry is back at the original departure.
        # This tests Local's independent effect gate without teleporting pose
        # history or claiming to simulate Gazebo's physical rollback.
        state_pub=self.node.create_publisher(ExecutionState,"planning/execution_state",20)
        self.spin_until(lambda:state_pub.get_subscription_count()>0)
        duration=recovery.knots[-recovery.order-1]-recovery.knots[recovery.order]
        state_pub.publish(ExecutionState(task_id=8204,task_version=task.task_version,
            request_id=task.request_id,trajectory_id=recovery.traj_id,
            state=ExecutionState.STATE_FINISHED,reason='TERMINAL_REACHED',
            execution_time=duration,duration=duration,terminal_error=.01))
        self.spin_until(lambda:any(f.task_id==8204 and f.reason=='LATERAL_RECOVERY_REJOIN_FAILED'
            for f in self.feedback),timeout=5)
        self.assertFalse(any(s.task_id==8204 and s.traj_id>recovery.traj_id for s in self.splines))
        self.assertNotIn('STRUCTURED_LOCAL_REPAIR_COMPLETE',self.read_child_log(0))

    def test_lateral_capture_extension_is_rejected_when_braking_space_is_insufficient(self):
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,.90) for z in (.4,.7)]
        self.cloud_points += [(4.,y,z) for y in [i*.05 for i in range(-6,19)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8405);self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==8405 and f.state==ObservationFeedback.EXECUTION_BLOCKED and f.reason=="DEPARTURE_UNSAFE" for f in self.feedback),timeout=12)
        self.assertFalse(any(s.task_id==8405 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE for s in self.splines))
        self.assertIn('LATERAL_SWEEP_REJECTED',self.read_child_log(0))

    def test_lateral_recovery_cannot_cross_task_gate_or_wall(self):
        self.cloud_points=[(x,y,z) for x in [i*.05 for i in range(-40,81)]
            for y in (-.34,.90) for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8104)
        # Right translation enters the wall; left translation exits the task's
        # allowed domain. Recovery must exhaust without authorizing either.
        task.has_directional_gate=True
        task.gate_origin.y=.05;task.gate_direction.y=-1.
        self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==8104 and f.state in (
            ObservationFeedback.EXECUTION_BLOCKED,ObservationFeedback.PATH_REJECTED,
            ObservationFeedback.LOCAL_MOTION_EXHAUSTED) for f in self.feedback),timeout=8)
        self.assertFalse(any(s.task_id==8104 and s.execution_mode==Bspline.MODE_RECOVERY_PRIMITIVE
            for s in self.splines))
        self.assertFalse(any(f.task_id==8104 and f.state==ObservationFeedback.STATION_REACHED
            for f in self.feedback))

    def test_autonomous_candidate_ack_and_frozen_topology_contract(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        task=self.task(2001); task.local_geometry_autonomy=True; task.route_graph_revision=10
        portal=RoutePortal(connection_id=11,source_node_id=1,target_node_id=2,
                           center=Point(x=.8,y=0.,z=0.),radius=.8)
        portal.direction.x=1.;task.route_portals=[portal]
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==2001 and a.request_id==task.request_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        ack=next(a for a in self.acks if a.task_id==2001 and a.request_id==task.request_id and
                 a.status==TrajectoryAck.STATUS_ACTIVATED)
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

    def test_cancel_confirmation_timeout_is_explicit_and_recoverable(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        task = self.task(9001)
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==9001 and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks), timeout=8)
        task.lifecycle = ObservationTask.CANCELLED
        self.task_pub.publish(task)
        self.spin_until(lambda:any(a.task_id==9001 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        # The proxy withholds retirement ACK. Resends do not renew the deadline.
        self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==9001 and
            f.state==ObservationFeedback.CANCEL_UNCONFIRMED for f in self.feedback))
        self.assertFalse(any(f.task_id==9001 and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==9001 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(f.task_id==9001 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        count = len([f for f in self.feedback if f.task_id==9001 and
                     f.state==ObservationFeedback.CANCEL_ACK])
        self.task_pub.publish(task)
        self.spin_until(lambda:len([f for f in self.feedback if f.task_id==9001 and
            f.state==ObservationFeedback.CANCEL_ACK]) > count)

    def test_predecessor_cancel_needs_actual_confirmation(self):
        spline_forward = self.node.create_publisher(Bspline, "controller_spline_proxy", 20)
        command_forward = self.node.create_publisher(ExecutionCommand, "controller_command_proxy", 20)
        commands = []
        self.node.create_subscription(ExecutionCommand, "planning/execution_command", commands.append, 20)
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
            spline_forward.get_subscription_count()>0 and command_forward.get_subscription_count()>0)
        first = self.task(11000)
        self.task_pub.publish(first)
        self.spin_until(lambda:any(s.task_id==11000 for s in self.splines), timeout=8)
        spline_forward.publish(next(s for s in self.splines if s.task_id==11000))
        self.spin_until(lambda:any(a.task_id==11000 and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==11000 and
            a.status==TrajectoryAck.STATUS_ACTIVATED))
        self.spin_until(lambda:any(f.task_id==11000 and
            f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        successor = self.task(11001)
        successor.predecessor_task_id=11000; successor.predecessor_task_version=1
        self.task_pub.publish(successor)
        self.spin_until(lambda:any(s.task_id==11001 for s in self.splines), timeout=8)
        # Neither G2's candidate nor either retirement command reaches Controller.
        successor.lifecycle=ObservationTask.CANCELLED
        self.task_pub.publish(successor)
        self.spin_until(lambda:any(c.reason=="ROLLING_SUCCESSOR_CANCELLED" for c in commands))
        first.lifecycle=ObservationTask.CANCELLED
        self.task_pub.publish(first)
        self.task_pub.publish(first)
        self.spin_until(lambda:any(c.task_id==11000 and
            c.reason=="TASK_RETIREMENT_CONFIRMATION_PENDING" for c in commands))
        self.assertFalse(any(f.task_id==11000 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        self.assertFalse(any(a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        self.assertEqual(self.states[-1].task_id, 11000)
        # First confirm only the predecessor. G2 must remain fenced/pending.
        command_forward.publish(next(c for c in commands if c.task_id==11000 and
            c.command==ExecutionCommand.COMMAND_RETIRE_TASK))
        self.spin_until(lambda:any(a.task_id==11000 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        first_ack=next(a for a in self.acks if a.task_id==11000 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED)
        self.ack_pub.publish(first_ack)
        self.spin_until(lambda:any(f.task_id==11000 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        self.assertFalse(any(f.task_id==11001 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))
        count=sum(f.task_id==11000 and f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback)
        self.task_pub.publish(first)
        self.spin_until(lambda:sum(f.task_id==11000 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback)>count)
        command_forward.publish(next(c for c in commands if c.task_id==11001 and
            c.command==ExecutionCommand.COMMAND_RETIRE_TASK))
        self.spin_until(lambda:any(a.task_id==11001 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        # A later predecessor ACK must not overwrite the successor ACK.
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==11001 and
            a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.ack_pub.publish(first_ack)
        self.spin_until(lambda:any(f.task_id==11001 and
            f.state==ObservationFeedback.CANCEL_ACK for f in self.feedback))

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
        # Start with a real certified pi alignment; a later arbitrary jump
        # now requires a new Local grant instead of an unvalidated turn.
        self.pose_yaw=math.pi
        first=self.task(100)
        self.task_pub.publish(first)
        self.spin_until(lambda:any("PATH_ACCEPTED" in s and f"request_id={first.request_id}" in s
                                   for s in self.statuses))
        self.spin_until(lambda:any(a.task_id==100 and a.request_id==first.request_id and
                                  a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        planned=next(a for a in self.acks if a.task_id==100 and a.request_id==first.request_id and
                     a.status==TrajectoryAck.STATUS_ACTIVATED)
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
        reversed_handoff.handoff_body_yaw=0.  # Explicit stale certification.
        self.spline_pub.publish(reversed_handoff)
        self.spin_until(lambda:any(a.trajectory_id==900 and
            a.status==TrajectoryAck.STATUS_REJECTED and
            a.reason=="HANDOFF_CERTIFICATE_STATE_CHANGED"
            for a in self.acks))
        self.assertFalse(any(s.trajectory_id==900 for s in self.states))
        # Even a renewed mock Local certificate cannot authorize a pi jump
        # while moving: the original velocity/heading gate still applies.
        reversed_handoff.traj_id=901
        reversed_handoff.handoff_body_yaw=self.pose_yaw
        self.spline_pub.publish(reversed_handoff)
        self.spin_until(lambda:any(a.trajectory_id==901 and
            a.status==TrajectoryAck.STATUS_REJECTED and
            a.reason=="HANDOFF_MOVING_HEADING_ERROR" and a.yaw_error>2.5
            for a in self.acks))
        self.assertFalse(any(s.trajectory_id==901 for s in self.states))
        self.pose_vx=0.
        # Once frozen, errors in the hysteresis band must not resume walking.
        self.pose_yaw=-.7
        until=time.monotonic()+.3
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(self.states[-1].state,ExecutionState.STATE_HEADING_FROZEN)
        aligned_acks=len([a for a in self.acks if a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED])
        until=time.monotonic()+1.2
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertEqual(aligned_acks,len([a for a in self.acks if a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED]))
        self.pose_yaw=0.
        old=self.stationary(100,first.request_id,1000)
        self.spline_pub.publish(old)
        self.spin_until(lambda:any(a.task_id==100 and a.trajectory_id==1000 and
                                  a.status==TrajectoryAck.STATUS_HOLD_INSTALLED for a in self.acks))
        next_task=self.task(101)
        self.task_pub.publish(next_task)
        self.spin_until(lambda:any(a.task_id==100 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        # Even though Controller has stopped, FSM must wait for its explicit
        # confirmation rather than installing the queued task immediately.
        self.assertFalse(any("PATH_ACCEPTED" in s and f"request_id={next_task.request_id}" in s
                             for s in self.statuses))
        stale=next(a for a in self.acks if a.task_id==100 and a.status==TrajectoryAck.STATUS_ACTIVATED)
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
                                  a.status==TrajectoryAck.STATUS_HOLD_INSTALLED for a in self.acks))
        self.spin_until(lambda:any(s.task_id==101 and s.task_version==1 for s in self.states))
        station_task=self.task(102)
        station_task.max_station_adjustments=1
        self.task_pub.publish(station_task)
        self.spin_until(lambda:any(a.task_id==101 and a.status==TrajectoryAck.STATUS_TASK_RETIRED
                                  for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==101 and
                                 a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(a.task_id==102 and a.request_id==station_task.request_id and
                                  a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==102 and
                                 a.request_id==station_task.request_id and
                                 a.status==TrajectoryAck.STATUS_ACTIVATED))
        self.spin_until(lambda:any("PATH_TRAJECTORY_READY" in s and
                                  f"request_id={station_task.request_id}" in s for s in self.statuses))
        # Follow actual Controller commands continuously. Distance alone is
        # no longer a station certificate: exact FINISHED and fresh measured
        # rest must precede STATION_REACHED.
        self.auto_ack_request=station_task.request_id
        self.pose_last_wall=time.monotonic();self.advance_commands=True
        self.spin_until(lambda:any(f.task_id==102 and f.state==ObservationFeedback.STATION_REACHED
                                  for f in self.feedback),timeout=20)
        self.advance_commands=False;self.auto_ack_request=None
        self.pose_vx=0.;self.pose_vy=0.
        reached=next(f for f in self.feedback if f.task_id==102 and
                     f.state==ObservationFeedback.STATION_REACHED)
        self.assertEqual(reached.station_adjustments,1)
        self.assertLessEqual(abs(reached.station.x-2.0),.20)
        self.assertAlmostEqual(reached.station.x,self.pose_x,delta=.03)
        self.assertTrue(any(s.task_id==102 and s.request_id==station_task.request_id and
            s.state==ExecutionState.STATE_FINISHED for s in self.states))
        self.assertFalse(any("REACHED request_id="+str(station_task.request_id) in s
                             for s in self.statuses))
        no_gain=self.task(102)
        no_gain.coarse_route = [no_gain.goal]
        no_gain.max_station_adjustments=3
        self.task_pub.publish(no_gain)
        self.spin_until(lambda:any(a.task_id==102 and a.request_id==no_gain.request_id and
                                  a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        self.assertFalse(any(f.task_id==102 and f.request_id==no_gain.request_id and
                             f.reason=="NO_NEW_OBSERVATION_STATION" for f in self.feedback))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==102 and
                                 a.request_id==no_gain.request_id and
                                 a.status==TrajectoryAck.STATUS_ACTIVATED))
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
        moving.use_measured_progress=True
        moving.execution_mode=Bspline.MODE_CLEARANCE_LIMITED
        moving.max_forward_speed=.3
        moving.min_forward_command=.3
        moving.min_forward_effective_speed=.165
        self.spline_pub.publish(moving)
        self.spin_until(lambda:any(a.trajectory_id==2000 and a.status==TrajectoryAck.STATUS_ACTIVATED
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
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        near_ack=next(a for a in self.acks if a.task_id==103 and
            a.request_id==nearby.request_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
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
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=5)
        self.assertFalse(any(f.task_id==103 and f.state==ObservationFeedback.STATION_REACHED
            for f in self.feedback))


    def test_rolling_rejection_waits_for_confirmed_rest_before_retry(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
                        self.controller_spline_pub.get_subscription_count()>0)
        first=self.task(14000); self.task_pub.publish(first)
        self.spin_until(lambda:any(s.task_id==14000 for s in self.splines),timeout=8)
        curve=next(s for s in self.splines if s.task_id==14000)
        self.controller_spline_pub.publish(curve)
        self.spin_until(lambda:any(a.task_id==14000 and a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        ack=next(a for a in self.acks if a.task_id==14000 and a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)
        self.spin_until(lambda:any(f.task_id==14000 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        successor=self.task(14001)
        successor.predecessor_task_id=14000; successor.predecessor_task_version=1
        self.task_pub.publish(successor)
        self.spin_until(lambda:any(s.task_id==14001 and s.rolling_task_handoff for s in self.splines),timeout=8)
        rejected=next(s for s in self.splines if s.task_id==14001 and s.rolling_task_handoff)
        failure=TrajectoryAck(task_id=14001,task_version=1,request_id=successor.request_id,
            trajectory_id=rejected.traj_id,candidate_id=rejected.candidate_id,
            status=TrajectoryAck.STATUS_REJECTED,reason='HANDOFF_ROLLING_VELOCITY_ERROR')
        self.pose_vx=.2
        self.advance_pose=True
        # Safety measures motion from pose history, not the raw twist alone.
        end=time.monotonic()+.4
        self.spin_until(lambda:time.monotonic()>=end,timeout=2)
        self.ack_pub.publish(failure)
        self.spin_until(lambda:any(f.task_id==14001 and f.state==ObservationFeedback.TRANSITION_WAIT_STOP for f in self.feedback))
        end=time.monotonic()+.7
        self.spin_until(lambda:time.monotonic()>=end,timeout=2)
        self.assertFalse(any(f.task_id==14001 and f.state==ObservationFeedback.TRANSITION_RESTARTING for f in self.feedback))
        self.assertFalse(any(f.task_id==14001 and f.state==ObservationFeedback.LOCAL_MOTION_EXHAUSTED for f in self.feedback))
        self.assertFalse(any(s.task_id==14001 and not s.rolling_task_handoff for s in self.splines))
        self.pose_vx=0.
        self.spin_until(lambda:any(f.task_id==14001 and f.state==ObservationFeedback.TRANSITION_RESTARTING for f in self.feedback))
        self.spin_until(lambda:any(s.task_id==14001 and not s.rolling_task_handoff for s in self.splines),timeout=8)
        restarted=next(s for s in self.splines if s.task_id==14001 and not s.rolling_task_handoff)
        self.assertEqual(restarted.request_id,successor.request_id)
        self.assertGreater(restarted.traj_id,rejected.traj_id)
        self.ack_pub.publish(failure)  # Rejected rolling ACK cannot match rest retry.
        end=time.monotonic()+.12
        self.spin_until(lambda:time.monotonic()>=end,timeout=1)
        self.controller_spline_pub.publish(restarted)
        self.spin_until(lambda:any(a.task_id==14001 and a.trajectory_id==restarted.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks))
        next_ack=next(a for a in self.acks if a.task_id==14001 and a.trajectory_id==restarted.traj_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED)
        self.ack_pub.publish(ack)  # Old ACK still cannot authorize the successor.
        self.assertFalse(any(f.task_id==14001 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))
        self.ack_pub.publish(next_ack)
        self.spin_until(lambda:any(f.task_id==14001 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))

    def test_z_rolling_task_swap_waits_for_candidate_ack_not_retirement(self):
        self.pose_x=0.; self.pose_yaw=0.; self.pose_vx=0.
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
                        self.spline_pub.get_subscription_count()>0)
        first=self.task(10000)
        self.task_pub.publish(first)
        self.spin_until(lambda:any("PATH_ACCEPTED" in s and
                       f"request_id={first.request_id}" in s for s in self.statuses))
        self.spin_until(lambda:any(a.task_id==10000 and a.request_id==first.request_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        first_ack=next(a for a in self.acks if a.task_id==10000 and
            a.request_id==first.request_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
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
            discontinuous.handoff_target_yaw=math.pi if traj_id==9000 else 0.
            self.spline_pub.publish(discontinuous)
            self.spin_until(lambda:any(a.trajectory_id==traj_id and
                a.status==TrajectoryAck.STATUS_REJECTED and a.reason==reason for a in self.acks))
            self.assertFalse(any(s.trajectory_id==traj_id for s in self.states))
        state_start=len(self.states)
        successor=self.task(10001)
        successor.predecessor_task_id=10000; successor.predecessor_task_version=1
        self.task_pub.publish(successor)
        self.spin_until(lambda:any(a.task_id==10001 and a.request_id==successor.request_id and
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        next_ack=next(a for a in self.acks if a.task_id==10001 and
            a.request_id==successor.request_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
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
            a.status==TrajectoryAck.STATUS_ACTIVATED for a in self.acks),timeout=8)
        cancelled_ack=next(a for a in self.acks if a.task_id==10002 and
            a.request_id==cancelled.request_id and a.status==TrajectoryAck.STATUS_ACTIVATED)
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
        count = len([f for f in self.feedback if f.task_id==10003 and
                     f.state==ObservationFeedback.CANCEL_ACK])
        self.task_pub.publish(invalid)
        self.spin_until(lambda:len([f for f in self.feedback if f.task_id==10003 and
            f.state==ObservationFeedback.CANCEL_ACK]) > count)



    def test_failed_recovery_search_has_one_probe_and_no_physical_charge(self):
        # A closed near-field box has no certified departure. This exercises
        # the real Local timers and terminal feedback, without any actuator.
        self.cloud_points=[(x,y,z) for x in [i*.04 for i in range(-15,16)]
            for y in (-.38,.38) for z in (.4,.7)]
        self.cloud_points += [(x,y,z) for x in (-.60,.60)
            for y in [i*.04 for i in range(-10,11)] for z in (.4,.7)]
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>30)
        task=self.task(8991);self.task_pub.publish(task)
        self.spin_until(lambda:any(f.task_id==8991 and f.reason=="DEPARTURE_EPISODE_EXHAUSTED"
            for f in self.feedback),timeout=16)
        log=self.read_child_log(0)
        self.assertEqual(log.count("[DEPARTURE_SEARCH]"),2)
        self.assertNotIn("[STRUCTURED_LOCAL_REPAIR_BUDGET]",log)
        self.assertNotIn("LATERAL_RECOVERY_ALREADY_USED",log)
        self.assertTrue(any(f.task_id==8991 and f.reason=="DEPARTURE_RETRY_PENDING"
            for f in self.feedback))
        self.assertFalse(any(f.task_id==8991 and f.state==ObservationFeedback.EXECUTION_READY
            for f in self.feedback))
        # New task IDs do not buy a third search. A normal alternative is
        # still evaluated before its recovery phase reaches this same fence.
        sibling=self.task(8992);sibling.goal.y=.5;sibling.coarse_route[-1]=sibling.goal
        self.task_pub.publish(sibling)
        self.spin_until(lambda:any(a.task_id==8991 and a.status==TrajectoryAck.STATUS_TASK_RETIRED for a in self.acks))
        self.ack_pub.publish(next(a for a in self.acks if a.task_id==8991 and a.status==TrajectoryAck.STATUS_TASK_RETIRED))
        self.spin_until(lambda:any(f.task_id==8992 and f.reason=="DEPARTURE_EPISODE_EXHAUSTED"
            for f in self.feedback),timeout=8)
        self.assertEqual(self.read_child_log(0).count("[DEPARTURE_SEARCH]"),2)



    def prepare_measured_ground_slope(self,degrees):
        slope=math.tan(math.radians(degrees))
        self.cloud_points=[(ix*.10,iy*.10,slope*ix*.10) for ix in range(-20,41) for iy in range(-20,21)]
        fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
                PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
                PointField(name='return_type',offset=12,datatype=PointField.UINT8,count=1)]
        # Typed h=0 means an observed support surface, independent of whether
        # the current walk capability permits this surface's inclination.
        self.cloud_message_factory=lambda header,points:create_cloud(header,fields,[(*p,53) for p in points])
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0)
        self.spin_until(lambda:self.cloud_counter>35)

    def test_measured_ground_slope_below_capability_keeps_normal_candidate(self):
        self.prepare_measured_ground_slope(24.)
        task=self.task(8993);self.task_pub.publish(task)
        self.spin_until(lambda:any(s.task_id==8993 and s.execution_mode!=Bspline.MODE_RECOVERY_PRIMITIVE
            for s in self.splines),timeout=8)
        self.assertNotIn('[LOCAL_TERRAIN_LIMIT]',self.read_child_log(0))

    def test_measured_ground_slope_above_capability_is_rejected_without_wall(self):
        self.prepare_measured_ground_slope(28.)
        task=self.task(8994);self.task_pub.publish(task)
        self.spin_until(lambda:'[LOCAL_TERRAIN_LIMIT]' in self.read_child_log(0),timeout=8)
        self.assertIn('reason=GROUND_SLOPE_LIMIT',self.read_child_log(0))
        self.assertFalse(any(f.task_id==8994 and f.state==ObservationFeedback.EXECUTION_READY for f in self.feedback))


if __name__=="__main__": unittest.main()
