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
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from scan_planner_msgs.msg import Bspline, ObservationTask, ObservationFeedback, TrajectoryAck, ExecutionState, LocomotionState
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


class TestTaskRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
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
        cls.cloud_counter=0

    @classmethod
    def tearDownClass(cls):
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
            odom.pose.pose.orientation.w=1.
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

    def test_retirement_fences_delayed_ack_and_candidate(self):
        self.spin_until(lambda:self.task_pub.get_subscription_count()>0 and
                        self.spline_pub.get_subscription_count()>0)
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
        retry=self.task(102)
        retry.max_station_adjustments=1
        self.task_pub.publish(retry)
        self.spin_until(lambda:any(f.task_id==102 and f.request_id==retry.request_id and
                                  f.state==ObservationFeedback.TASK_EXHAUSTED for f in self.feedback))


if __name__=="__main__": unittest.main()
