"""Isolated real ROS mapping processes; synthetic rays, no actuators or Gazebo."""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from scan_planner_msgs.msg import LocomotionState, LocalMapPatch
from std_msgs.msg import Header


def ground_height(value):
    """Encode a measured height above the local ground into the return_type byte.

    Shared wire contract (plan_env/ground_height.hpp): 3..252 carry the height
    at 1 cm resolution over [-0.50, +1.99] m; 1 means measured but unknown.
    """
    return 3 + int(round((max(-0.50, min(1.99, value)) + 0.50) / 0.01))


class TestMid360Mapping(unittest.TestCase):
    def setUp(self):
        self.context=Context()
        self.domain=150+os.getpid()%50
        rclpy.init(context=self.context,domain_id=self.domain)
        self.node=rclpy.create_node('mid360_mapping_contract_probe',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.log=tempfile.TemporaryFile(mode='w+')
        root=Path(__file__).resolve().parents[1]
        binary=Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node'
        separate=self._testMethodName!='test_same_floor_is_an_obstacle_without_low_channel'
        high_separate=self._testMethodName!='test_above_band_beam_is_an_obstacle_without_high_channel'
        self_filter=self._testMethodName!='test_real_obstacle_remains_when_self_filter_disabled'
        args=[str(binary),'--ros-args','--params-file',str(root/'config/planner.yaml'),
              '-p','grid_map.use_ray_cloud:=true','-p','grid_map.cloud_is_world:=true',
              '-p','grid_map.need_extrinsic:=true','-p','grid_map.lidar_extrinsic_x:=0.187',
              '-p','grid_map.lidar_extrinsic_y:=0.0','-p','grid_map.lidar_extrinsic_z:=0.0803',
              '-p','grid_map.lidar_extrinsic_roll:=0.0',
              '-p','grid_map.lidar_extrinsic_pitch:=0.22689280275926285',
              '-p','grid_map.lidar_extrinsic_yaw:=0.0',
              '-p',f'grid_map.separate_low_returns:={str(separate).lower()}',
              '-p',f'grid_map.separate_high_returns:={str(high_separate).lower()}',
              # Show the full band in the display slice so an overhead endpoint
              # can be observed as occupied when the high channel is disabled.
              '-p','grid_map.vis_height:=1.0',
              '-p',f'grid_map.self_filter_enabled:={str(self_filter).lower()}']
        if self._testMethodName=='test_local_profile_changes_constraint_but_preserves_measured_z':
            args += ['-p','motion.terrain.maximum_rise:=0.35','-p','motion.terrain.maximum_drop:=0.35']
        self.child=subprocess.Popen(args,env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain)),
                                    stdout=self.log,stderr=self.log)
        self.explorer_child=None;self.explorer_map=None;self.explorer_log=None
        if self._testMethodName in ('test_typed_ground_keeps_low_rocks_and_drop_hazards',
                'test_reobserved_support_retracts_only_terrain_projection'):
            self.explorer_log=tempfile.TemporaryFile(mode='w+')
            executable=Path(get_package_prefix('scan_planner'))/'lib/scan_planner/frontier_explorer.py'
            self.explorer_child=subprocess.Popen([str(executable),'--ros-args',
                '-p','auto_start:=false','-p','use_ray_cloud:=true',
                '-p','cloud_is_world:=false','-p','cloud_frame_id:=lidar',
                '-p','need_extrinsic:=false','-p','obstacle_z_relative_to_body:=true',
                '-p','obstacle_min_z:=-0.25','-p','obstacle_max_z:=0.25',
                '-r','cloud:=explorer_cloud'],env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain)),
                stdout=self.explorer_log,stderr=self.explorer_log)
            self.node.create_subscription(OccupancyGrid,'explorer/planning_map',
                lambda msg:setattr(self,'explorer_map',msg),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.explorer_rays=self.node.create_publisher(PointCloud2,'explorer_cloud',10)
        self.patch=None
        self.node.create_subscription(LocalMapPatch,'grid_map/local_map_patch',
            lambda msg:setattr(self,'patch',msg),qos_profile_sensor_data)
        self.body=self.node.create_publisher(Odometry,'body_pose',20)
        self.sensor=self.node.create_publisher(Odometry,'sensor_pose',20)
        self.rays=self.node.create_publisher(PointCloud2,'cloud',10)
        self.ready=self.node.create_publisher(LocomotionState,'/robot/locomotion_state',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.terrain_facts=None
        self.occupied=None;self.inflated=None;self.low=None;self.high=None;self.counter=0;self.low_updates=0
        self.node.create_subscription(PointCloud2,'grid_map/terrain_evidence',
            lambda msg:setattr(self,'terrain_facts',np.array(list(read_points(msg,
                field_names=['x','y','z','ground_height','capture_body_z','observation_scan'],
                skip_nans=False).tolist()),dtype=float)),qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2,'grid_map/occupancy',
            lambda msg:setattr(self,'occupied',self.xyz(msg)),qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2,'grid_map/occupancy_inflate',
            lambda msg:setattr(self,'inflated',self.xyz(msg)),qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2,'grid_map/low_surface_returns',
            self.on_low,qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2,'grid_map/high_surface_returns',
            lambda msg:setattr(self,'high',self.xyz(msg)),qos_profile_sensor_data)
        # The rear leg is inside the body envelope but outside the old
        # sensor-centred one. Low obstacles are deliberately above the gate.
        # Body z is 0.30, so the band is [0.05, 0.55]: beam_over is a tunnel
        # timber set above the body and must never occupy; beam_inside is just
        # below the boundary and stays the Local planner's obstacle evidence.
        self.floor=np.array([1.,-1.,0.])
        self.trench=np.array([2.,-1.,-.30])
        self.small_obstacle=np.array([1.,1.,.08])
        self.wall=np.array([1.,1.,.35])
        self.rear_leg=np.array([-.38,0.,.15])
        self.no_return=np.array([2.,2.,-.20])
        self.beam_over=np.array([1.5,0.,.80])
        self.beam_inside=np.array([1.5,.4,.52])
        self.timer=self.node.create_timer(.02,self.tick)

    def on_low(self,message):
        self.low=self.xyz(message);self.low_updates+=1

    @staticmethod
    def xyz(message):
        points=read_points(message,field_names=('x','y','z'),skip_nans=True).reshape(-1)
        return np.column_stack((points['x'],points['y'],points['z']))

    def tick(self):
        odom=Odometry();odom.header.stamp=self.node.get_clock().now().to_msg()
        odom.header.frame_id='world';odom.pose.pose.position.z=.30
        odom.pose.pose.orientation.w=1.
        self.body.publish(odom);self.sensor.publish(odom)
        state=LocomotionState();state.stamp=odom.header.stamp;state.epoch=1
        state.state=LocomotionState.STATE_RUN
        state.actuation_ready=True;state.perception_pose_valid=True
        self.ready.publish(state);self.counter+=1
        if self.counter%5:return
        header=Header(stamp=odom.header.stamp,frame_id='world')
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,name in enumerate(('x','y','z'))]
        fields.append(PointField(name='return_type',offset=12,datatype=PointField.UINT8,count=1))
        points=[(*p,1) for p in (self.floor,self.trench,self.small_obstacle,self.wall,
                                 self.rear_leg,self.beam_over,self.beam_inside)]
        points.append((*self.no_return,2))
        if self._testMethodName in ('test_typed_ground_keeps_low_rocks_and_drop_hazards',
                'test_local_profile_changes_constraint_but_preserves_measured_z'):
            points.extend([(2.,-1.,.12,ground_height(0.0)),(2.,1.,-.30,ground_height(-0.30)),
                           (1.,-2.,.04,ground_height(0.30)),(2.,2.,.30,ground_height(0.10))])
        if self._testMethodName in ('test_reobserved_support_retracts_only_terrain_projection',
                'test_support_cannot_erase_real_wall_sharing_projection'):
            points=[(2.,-1.,0.,getattr(self,'terrain_kind',ground_height(0.30))),(2.,1.,.3,1)]
        if self._testMethodName=='test_flickering_hazard_never_projects_at_body_height':
            # Every third scan reports this cell as a 30 cm rise and the other
            # two report observed ground: the minority flicker a rough floor
            # produces when the per-frame support fit moves. If a minority
            # clearance obstruction could latch a projection, the plan's own
            # body plane would be blocked there forever. The steady obstruction
            # in the same cloud is the positive control.
            self.flicker_scan=getattr(self,'flicker_scan',0)+1
            points=[(2.,-1.,0.,ground_height(0.30) if self.flicker_scan%3==0 else ground_height(0.0)),
                    (-2.,-1.,0.,ground_height(0.30)),(2.,1.,.3,1)]
        if self._testMethodName=='test_unobserved_projection_is_released_without_support':
            points=[(2.,1.,.3,1)]
            if not getattr(self,'hazard_unobserved',False):
                points.insert(0,(2.,-1.,0.,ground_height(0.30)))
        if getattr(self,'wall_shares_projection',False):points.append((2.,-1.,.3,1))
        self.rays.publish(create_cloud(header,fields,points))
        local_header=Header(stamp=odom.header.stamp,frame_id='lidar')
        self.explorer_rays.publish(create_cloud(local_header,fields,
            [(x,y,z-.30,kind) for x,y,z,kind in points]))

    def wait_for_map(self):
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
            if (self.occupied is not None and self.low is not None
                    and self.high is not None and self.low_updates>=5):break
        self.assertIsNone(self.child.poll(),self.log_text())
        self.assertIsNotNone(self.occupied,self.log_text())
        self.assertIsNotNone(self.low,self.log_text())
        self.assertIsNotNone(self.high,self.log_text())

    def log_text(self):
        self.log.flush();self.log.seek(0);return self.log.read()[-5000:]

    def present(self,points,target,tolerance=.045):
        return bool(len(points) and (np.linalg.norm(points-target,axis=1)<tolerance).any())

    def test_low_channel_preserves_real_z_without_obstacle_endpoints(self):
        self.wait_for_map()
        self.assertFalse(self.present(self.occupied,self.floor))
        self.assertFalse(self.present(self.occupied,self.trench))
        self.assertTrue(self.present(self.low,self.floor,.001),self.log_text())
        self.assertTrue(self.present(self.low,self.trench,.001))
        self.assertFalse(self.present(self.low,self.no_return,.001))
        self.assertTrue(self.present(self.occupied,self.small_obstacle))
        self.assertTrue(self.present(self.occupied,self.wall))
        self.assertFalse(self.present(self.occupied,self.rear_leg))

    def test_typed_ground_keeps_low_rocks_and_drop_hazards(self):
        self.wait_for_map()
        deadline=time.monotonic()+3.
        while self.inflated is None and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.inflated)
        self.assertFalse(self.present(self.occupied,np.array([2.,-1.,.12])))
        self.assertTrue(self.present(self.low,np.array([2.,-1.,.12]),.001))
        # A measured drop and a 30 cm rise reach the trunk's clearance volume
        # (0.22..0.55 m above the observed ground) and block the body plane.
        self.assertTrue(self.present(self.inflated,np.array([2.,1.,.30])))
        self.assertTrue(self.present(self.inflated,np.array([1.,-2.,.30])))
        self.assertTrue(self.present(self.occupied,self.small_obstacle))
        # A 10 cm rise is terrain the legs handle: measured, reported, and
        # deliberately NOT turned into a body-plane obstruction.
        self.assertFalse(self.present(self.inflated,np.array([2.,2.,.30])))
        deadline=time.monotonic()+5.
        while self.explorer_map is None and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.explorer_map)
        grid=self.explorer_map;res=grid.info.resolution
        def value(x,y):
            cx=math.floor((x-grid.info.origin.position.x)/res)
            cy=math.floor((y-grid.info.origin.position.y)/res)
            return grid.data[cy*grid.info.width+cx]
        self.assertEqual(value(2.05,-.95),0)  # same observed support as Local
        # The Explorer's obstacle rule is the same geometric one as Local's:
        # measured geometry that reaches the trunk's clearance volume. A drop
        # and a 30 cm rise qualify; the 10 cm rise is terrain and only lands in
        # the sparse annotation, never in 2-D occupancy or inflation.
        self.assertEqual(value(2.05,1.05),100)     # measured drop
        self.assertEqual(value(1.05,-1.95),100)    # 30 cm rise
        self.assertNotEqual(value(2.05,2.05),100)  # 10 cm rise: legs' terrain
        # Positive control: measured geometry that reaches the body band stays
        # the Explorer's obstacle (wall and low obstacle share this cell).
        self.assertEqual(value(1.05,1.05),100)

    def test_local_profile_changes_constraint_but_preserves_measured_z(self):
        self.wait_for_map()
        deadline=time.monotonic()+4.
        while (self.terrain_facts is None or self.inflated is None) and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.terrain_facts,self.log_text())
        facts=self.terrain_facts
        drop=facts[(abs(facts[:,0]-2.)<.001)&(abs(facts[:,1]-1.)<.001)]
        self.assertEqual(len(drop),1,self.log_text())
        self.assertAlmostEqual(drop[0,2],-.30,places=5)
        self.assertAlmostEqual(drop[0,3],-.30,places=5)
        self.assertAlmostEqual(drop[0,4],.30,places=5)
        self.assertGreater(drop[0,5],0)
        self.assertFalse(self.present(self.inflated,np.array([2.,1.,.30])))
        self.assertFalse(self.present(self.inflated,np.array([1.,-2.,.30])))
        self.assertTrue(self.present(self.occupied,self.small_obstacle))
        self.assertFalse(self.present(self.occupied,np.array([2.,1.,.30])))

    def test_reobserved_support_retracts_only_terrain_projection(self):
        self.wait_for_map()
        deadline=time.monotonic()+3.
        while self.inflated is None and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.inflated)
        target=np.array([2.,-1.,.30])
        self.assertTrue(self.present(self.inflated,target),self.log_text())
        self.assertFalse(self.present(self.occupied,target),self.log_text())
        # New measured support at the SAME XY must revoke the old projection
        # at body height, even though its real XYZ endpoint is at floor height.
        self.terrain_kind=ground_height(0.0)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
            if self.occupied is not None and not self.present(self.inflated,target):break
        self.assertFalse(self.present(self.inflated,target),self.log_text())
        self.assertTrue(self.present(self.inflated,np.array([2.,1.,.30])))
        # Check the actual downstream 2-D messages, not just the RViz cloud.
        deadline=time.monotonic()+3.
        def state(msg):
            if msg is None:return 100
            if hasattr(msg,'info'):
                res=msg.info.resolution;origin=msg.info.origin.position;w=msg.info.width;data=msg.data
            else:res=msg.resolution;origin=msg.origin;w=msg.width;data=msg.occupancy
            x=math.floor((2.025-origin.x)/res);y=math.floor((-.975-origin.y)/res)
            return data[y*w+x]
        while time.monotonic()<deadline and (state(self.patch)==100 or state(self.explorer_map)!=0):
            self.executor.spin_once(timeout_sec=.02)
        self.assertNotEqual(state(self.patch),100)
        self.assertEqual(state(self.explorer_map),0)

    def test_flickering_hazard_never_projects_at_body_height(self):
        self.wait_for_map()
        deadline=time.monotonic()+3.
        while self.inflated is None and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.inflated)
        steady=np.array([-2.,-1.,.30])
        flicker=np.array([2.,-1.,.30])
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
            if self.present(self.inflated,steady):break
        # Positive control: a hazard observed every scan must project.
        self.assertTrue(self.present(self.inflated,steady),self.log_text())
        # Alternating hazard/support evidence never accumulates a streak, so
        # the plan's body plane must stay clear at that cell.
        self.assertFalse(self.present(self.inflated,flicker),self.log_text())
        end=self.counter+30
        while self.counter<end and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertFalse(self.present(self.inflated,flicker),self.log_text())

    def test_unobserved_projection_is_released_without_support(self):
        self.wait_for_map()
        deadline=time.monotonic()+3.
        while self.inflated is None and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertIsNotNone(self.inflated)
        target=np.array([2.,-1.,.30])
        deadline=time.monotonic()+4.
        while time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
            if self.present(self.inflated,target):break
        self.assertTrue(self.present(self.inflated,target),self.log_text())
        # The cell stops being observed at all: no fresh hazard and no support.
        # The projection must not outlive its evidence.
        self.hazard_unobserved=True
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
            if not self.present(self.inflated,target):break
        self.assertFalse(self.present(self.inflated,target),self.log_text())
        self.assertTrue(self.present(self.inflated,np.array([2.,1.,.30])))

    def test_support_cannot_erase_real_wall_sharing_projection(self):
        self.wall_shares_projection=True
        self.wait_for_map();self.terrain_kind=3
        end=self.counter+40
        deadline=time.monotonic()+3.
        while self.counter<end and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertTrue(self.present(self.occupied,np.array([2.,-1.,.30])),self.log_text())

    def test_same_floor_is_an_obstacle_without_low_channel(self):
        self.wait_for_map()
        self.assertTrue(self.present(self.occupied,self.floor))
        self.assertTrue(self.present(self.occupied,self.trench))
        self.assertFalse(self.present(self.occupied,self.rear_leg))

    def test_overhead_beam_keeps_its_own_channel(self):
        """A 3-D lidar always sees the tunnel's beams; they must not block it."""
        self.wait_for_map()
        self.assertFalse(self.present(self.occupied,self.beam_over))
        self.assertTrue(self.present(self.high,self.beam_over,.001),self.log_text())
        # The channel is a band boundary, not a licence to ignore real geometry.
        self.assertTrue(self.present(self.occupied,self.beam_inside))
        self.assertFalse(self.present(self.high,self.beam_inside,.001))
        self.assertFalse(self.present(self.high,self.floor,.001))
        self.assertFalse(self.present(self.high,self.wall,.001))

    def test_above_band_beam_is_an_obstacle_without_high_channel(self):
        self.wait_for_map()
        self.assertTrue(self.present(self.occupied,self.beam_over))

    def test_real_obstacle_remains_when_self_filter_disabled(self):
        self.wait_for_map()
        self.assertTrue(self.present(self.occupied,self.rear_leg))
        self.assertTrue(self.present(self.occupied,self.small_obstacle))
        self.assertFalse(self.present(self.occupied,self.floor))

    def tearDown(self):
        self.timer.cancel()
        if self.explorer_child is not None:
            self.explorer_child.terminate()
            try:self.explorer_child.wait(timeout=5.)
            except subprocess.TimeoutExpired:self.explorer_child.kill();self.explorer_child.wait()
            self.explorer_log.close()
        if self.child.poll() is None:
            self.child.terminate()
            try:self.child.wait(timeout=5.)
            except subprocess.TimeoutExpired:self.child.kill();self.child.wait()
        self.executor.remove_node(self.node);self.executor.shutdown()
        self.node.destroy_node();rclpy.shutdown(context=self.context);self.log.close()


if __name__=='__main__':unittest.main()
