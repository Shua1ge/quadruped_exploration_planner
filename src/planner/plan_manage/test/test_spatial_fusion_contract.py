"""Regression of the real Explorer callback: no ROS process or actuation."""
import importlib.util
import sys
import threading
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header

MANAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MANAGE / 'scripts'))
SPEC = importlib.util.spec_from_file_location('spatial_contract_explorer',
    MANAGE / 'scripts/frontier_explorer.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
from explorer_core.grid import ExplorationGrid, FREE, OCCUPIED, project_spatial_rays


def height(value):
    return 3 + int(round((value + .5) / .01))


def fusion_node():
    node = SimpleNamespace(position=(0., 0.), cloud_frame_id='sensor',
        map_fusion_started=True, last_map_update_ns=0,
        last_integrated_cloud_stamp_ns=0, pose_history=[(1000000000, 0., 0., .3, 0., 0., 0., 1.)],
        max_interpolation_gap_ns=50000000, max_nearest_pose_age_ns=20000000,
        use_ray_cloud=True, need_extrinsic=False, obstacle_z_relative_to_body=True,
        obstacle_min_z=-.25, obstacle_max_z=.25, body_slice_min_z=-.125,
        clearing_max_z=.85, mapping_range=7.5, no_return_range=3., ray_count=720,
        hit_dilation_bins=0, terrain_floor_tolerance=.035,
        obstacle_min_height=.22, obstacle_max_height=.55,
        footprint_radius=.25, footprint_offset=.18,
        _grid_lock=threading.Lock(), _live_grid=ExplorationGrid(16., 16., .2),
        changed_cells=set(), map_content_revision=0, map_update_count=0)
    stats=defaultdict(int)
    node.fusion_stats=lambda:stats
    node.map_fusion_allowed=lambda:True
    node.cloud_stamp_ns=lambda msg:1000000000
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000))
    node.get_logger=lambda:SimpleNamespace(info=lambda *a, **k:None,
        warning=lambda *a, **k:None, error=lambda *a, **k:None)
    node.record_terrain=lambda *args:None
    node.publish_maps=lambda:None
    return node


def test_real_callback_never_reintroduces_miss_invalid_or_low_surface_as_obstacle():
    node=fusion_node()
    # Every case has a different azimuth; positive controls retain both a real
    # body-height obstacle and a measured step/drop below the current body Z.
    points=[(3.,0.,0.,2), (2.,1.,-.2,1), (2.,-1.,-.2,height(.10)),
        (2.,2.,.1,1), (2.,-2.,-.3,height(.30)), (2.,-3.,-.6,height(-.30)),
        (1.,3.,0.,0), (1.,-3.,1.,1), (.1,0.,0.,1)]
    fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1)
            for i,n in enumerate(('x','y','z'))]
    fields.append(PointField(name='return_type',offset=12,datatype=PointField.UINT8,count=1))
    msg=create_cloud(Header(frame_id='sensor',stamp=Time(sec=1)),fields,points)
    MODULE.FrontierExplorer.integrate_cloud(node,msg)
    grid=node._live_grid
    for point in points[:3]+points[6:]:
        assert grid.value(grid.world_to_cell(*point[:2])) != OCCUPIED, point
    for point in points[3:6]:
        assert grid.value(grid.world_to_cell(*point[:2])) == OCCUPIED, point
    assert node.map_update_count == 1


@pytest.mark.parametrize('kind',[0,2])
def test_invalid_or_no_return_has_no_occupied_endpoint_in_either_channel(kind):
    result=project_spatial_rays([[3.,0.,0.]],[kind],[0.,0.,.3],np.eye(3),
        .05,.55,7.5,3.,720,0,.2)
    assert not result.obstacle.any()
    assert not np.isfinite(result.ranges).any()
    if kind==2:
        assert result.clearing.max()==pytest.approx(3.)


def test_self_filter_retains_real_close_side_wall():
    result=project_spatial_rays([[.1,0.,0.],[0.,.35,0.]],[1,1],
        [0.,0.,.3],np.eye(3),.05,.55,7.5,3.,720,0,.2,
        body_position=[0.,0.,.3],body_yaw=0.,self_radius=.25,self_offset=.18)
    assert result.obstacle.tolist()==[False,True]


def test_new_valid_free_evidence_retracts_old_false_wall_but_not_a_real_wall():
    grid=ExplorationGrid(12.,12.,.2)
    false=grid.world_to_cell(2.,0.);wall=grid.world_to_cell(2.,1.)
    grid.data[false[1],false[0]]=OCCUPIED;grid.evidence[false[1],false[0]]=1
    for _ in range(4):
        result=project_spatial_rays([[2.,0.,-.3],[2.,1.,0.]],[height(0.),1],
            [0.,0.,.3],np.eye(3),.05,.55,7.5,3.,720,0,.2)
        ground={grid.world_to_cell(*p[:2]) for p in result.world[result.support]}
        occupied={grid.world_to_cell(*p[:2]) for p in result.world[result.obstacle]}
        grid.integrate_ranges((0.,0.),result.ranges,7.5,clearing_ranges=result.clearing,
                              ground_cells=ground,obstacle_cells=occupied)
    assert grid.value(false)==FREE
    assert grid.value(wall)==OCCUPIED
