import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from explorer_core.grid import ExplorationGrid, FREE, OCCUPIED
from explorer_core.region_visits import RegionVisitWindow, BoundedVisitRoutes, choose_joint_visit
from explorer_core.path_planning import RouteSearchStatus


def region(rid, x, y=0):
    return NS(region_id=rid, centroid=(x,y))


def option(rid, cell, frontier=None, targets=None, path=None, length=None):
    return NS(region_id=rid, cell=cell, frontier_cell=frontier or cell,
              observation_cells=set(targets or {(100+i,100) for i in range(10)}),
              path=path or [(0,0),cell], path_length=length if length is not None else math.dist((0,0),cell),
              turn_cost=0.)


class EuclideanRoutes:
    def transition(self, a, b):
        return math.dist(a,b), [a,b]


def test_full_order_retained_but_window_is_always_one_plus_two():
    window=RegionVisitWindow();rs=[region(i,i*3) for i in range(60)]
    selected=window.reconcile(rs,(0,0),1)
    assert len(selected)==3 and len(window.order)==60
    current=selected[0].region_id;scope=window.commit(current);order=window.order[:]
    # Ordinary summaries can change without re-sorting all region visits.
    rs=[region(r.region_id,200-r.centroid[0]) for r in rs]
    assert window.reconcile(rs,(100,0),2)[0].region_id==current
    assert window.order==order and window.commit(current)==scope
    # A discovered Region cannot preempt an installed current task.
    assert window.reconcile(rs+[region(99,100)],(100,0),3)[0].region_id==current


def test_deferred_region_keeps_residual_and_reenters_on_new_revision():
    window=RegionVisitWindow();rs=[region(i,i*4) for i in range(6)]
    current=window.reconcile(rs,(0,0),1)[0].region_id
    window.defer(current,1)
    assert current in window.order and current not in [r.region_id for r in window.reconcile(rs,(0,0),1)]
    window.reconcile(rs,(0,0),2)
    assert current not in window.deferred


def test_current_and_future_station_choice_rejects_nearest_only_solution():
    # Same Frontier, alternative stations. Greedy current access favours (0,3),
    # but exiting through B/C favours (5,0). No motion-yaw rule creates this.
    a=option(1,(0,3),frontier=(4,4),length=3)
    b=option(1,(5,0),frontier=(4,4),length=5)
    B=option(2,(10,0));C=option(3,(20,0))
    greedy=choose_joint_visit([a,b],[],EuclideanRoutes(),depth=1)
    joint=choose_joint_visit([a,b],[[B],[C]],EuclideanRoutes(),depth=1)
    assert greedy.candidate is a and joint.candidate is b
    assert joint.prefix==((4,4),)


def test_expected_gain_is_marginal_and_same_frontier_is_not_visited_twice():
    shared={(i,100) for i in range(50)}
    unique={(i,200) for i in range(50)}
    a=option(1,(2,0),frontier=(2,1),targets=shared)
    b=option(1,(3,0),frontier=(3,1),targets=shared)
    alt=option(1,(2,1),frontier=a.frontier_cell,targets=shared)
    c=option(1,(4,0),frontier=(4,1),targets=unique)
    result=choose_joint_visit([a,b,alt,c],[],EuclideanRoutes(),depth=2)
    assert c.frontier_cell in result.prefix
    assert len(result.prefix)==len(set(result.prefix))==2


def test_retained_visit_prefix_tolerates_small_score_changes_but_is_not_a_veto():
    a=option(1,(2,0));b=option(1,(2,1),length=1.95)
    result=choose_joint_visit([a,b],[],EuclideanRoutes(),retained_prefix=(a.frontier_cell,),depth=1)
    assert result.candidate is a
    b.observation_cells={(i,100) for i in range(1000)}
    improved=choose_joint_visit([a,b],[],EuclideanRoutes(),retained_prefix=(a.frontier_cell,),depth=1)
    assert improved.candidate is b and improved.reordered
    # If evidence removes that option, the remaining valid candidate is used.
    assert choose_joint_visit([b],[],EuclideanRoutes(),retained_prefix=(a.frontier_cell,)).candidate is b


def test_unavailable_future_route_cannot_veto_a_valid_current_leg():
    class NoFuture:
        def transition(self,a,b):return float('inf'),None
    a=option(1,(1,0));b=option(2,(10,0))
    result=choose_joint_visit([a],[[b]],NoFuture())
    assert result.candidate is a and result.continuation_estimated


def test_search_capacity_preserves_a_resumable_search_and_never_means_no_route():
    grid=ExplorationGrid(30,20,1,0,0);grid.data.fill(FREE)
    grid.data[:17,15]=OCCUPIED
    routes=BoundedVisitRoutes()
    routes.begin(grid,set(),5)
    first=routes.query((1,1),(28,1),quota=5)
    assert first.status==RouteSearchStatus.INCOMPLETE and routes.budget.used<=5
    search=routes.searches[((1,1),(28,1))]
    for _ in range(100):
        routes.begin(grid,set(),16)
        result=routes.query((1,1),(28,1),quota=16)
        assert routes.budget.used<=16
        if result.status==RouteSearchStatus.FOUND:break
        assert result.status==RouteSearchStatus.INCOMPLETE
        assert routes.searches[((1,1),(28,1))] is search
    assert result.status==RouteSearchStatus.FOUND
    # Unrelated map changes do not invalidate this witnessed connection.
    routes.begin(grid,set(),1)
    assert routes.query((1,1),(28,1)).status==RouteSearchStatus.FOUND
    assert routes.cache_hits==1


def test_a_real_no_route_fact_is_invalidated_by_inspected_geometry_changes():
    grid=ExplorationGrid(12,8,1,0,0);grid.data.fill(FREE);grid.data[:,6]=OCCUPIED
    routes=BoundedVisitRoutes();routes.begin(grid,set(),1000)
    assert routes.query((1,3),(10,3),quota=1000).status==RouteSearchStatus.NO_ROUTE
    grid.data[3,6]=FREE
    routes.begin(grid,set(),1000)
    assert routes.query((1,3),(10,3)).status==RouteSearchStatus.FOUND


def test_changed_route_is_rechecked_and_cannot_publish_a_stale_witness():
    grid=ExplorationGrid(12,8,1,0,0);grid.data.fill(FREE)
    routes=BoundedVisitRoutes();routes.begin(grid,set(),100)
    assert routes.query((1,3),(10,3)).status==RouteSearchStatus.FOUND
    grid.data[:,6]=OCCUPIED
    routes.begin(grid,set(),1)
    assert routes.query((1,3),(10,3)).status!=RouteSearchStatus.FOUND


def test_preview_estimate_is_explicit_and_cannot_create_an_executable_path():
    grid=ExplorationGrid(30,20,1,0,0);grid.data.fill(FREE);grid.data[:17,15]=OCCUPIED
    routes=BoundedVisitRoutes();routes.begin(grid,set(),1)
    distance,path=routes.transition((1,1),(28,1))
    assert math.isfinite(distance) and path is None
    assert routes.estimated_transitions==1 and routes.budget.used==0


def test_same_frontier_retains_station_alternatives_in_production_builder(monkeypatch):
    from test_frontier_explorer import candidate_test_explorer, MODULE
    explorer=candidate_test_explorer();explorer._visit_pending_regions=set()
    explorer.observation_radius=4.
    grid=explorer.grid;grid.data[2,9]=MODULE.UNKNOWN
    monkeypatch.setattr(MODULE,'candidate_cells',lambda cluster:[(8,2)])
    monkeypatch.setattr(MODULE,'safe_viewpoint_cells',lambda *a,**kw:[(6,2),(6,3),(6,4)])
    monkeypatch.setattr(MODULE,'observation_target_cells',lambda *a:{(9,2)})
    routes=BoundedVisitRoutes();routes.begin(grid,set(),100)
    r=MODULE.FrontierRegion(3,[[(8,2)]],(8,2),{(8,2)})
    records=explorer.build_frontier_candidates((1,2),[r],set(),set(),
        frontier_limit=4,station_limit=2,route_provider=routes)
    assert {c.cell for c in records}=={(6,2),(6,4)}
    assert all(c.path for c in records)


def test_production_scheduler_only_refines_three_of_many_regions():
    from test_frontier_explorer import candidate_test_explorer, MODULE
    explorer=candidate_test_explorer();explorer.region_visit_window_enabled=True
    explorer.selection_strategy='hierarchical';explorer.region_visit_frontiers=4
    explorer.region_visit_stations=2;explorer.region_visit_route_expansions=100
    explorer.active_region_id=None;explorer.active_commitment_scope=None
    explorer.region_sequence=[];explorer.publish_region_sequence=lambda:None
    explorer.publish_region_overlay=lambda *a:None
    regions=[MODULE.FrontierRegion(i,[[(i+2,2)]],(i+2,2),{(i+2,2)}) for i in range(60)]
    seen=[]
    def build(start,rs,inf,blocked,excluded,**kw):
        seen.extend(r.region_id for r in rs)
        return [MODULE.FrontierCandidate(r.region_id,(6,2),(8,2),(6.5,2.5),
            [(1,2),(2,2),(3,2),(4,2),(5,2),(6,2)],5.,10,5,0.,{(9,2)}) for r in rs]
    explorer.build_frontier_candidates=build
    explorer.grid.data[2,9]=MODULE.UNKNOWN
    assert explorer.choose_window_visit((1,2),regions,set(),(),allow_region_release=True,
        allow_cross_region_preparation=False,required_scope=None) is not None
    assert len(seen)==3 and len(explorer.region_sequence)==60
    assert explorer.last_visit_stats['window']==seen


def test_window_commit_still_waits_for_exact_typed_installation_ack():
    from test_frontier_explorer import pending_handoff_explorer
    from scan_planner_msgs.msg import ObservationFeedback
    explorer=pending_handoff_explorer()
    explorer.region_visit_window_enabled=True
    window=explorer.visit_window();oldscope=window.commit(3)
    explorer.active_commitment_scope=oldscope
    explorer.last_selected_commitment_scope=oldscope
    pending=explorer.pending_observation_handoff
    pending.candidate.commitment_scope=window.scope(4)
    old=explorer.active_observation
    assert window.committed_region==3
    explorer.observation_feedback_callback(ObservationFeedback(
        task_id=pending.task.task_id,task_version=1,
        request_id=pending.request_id,trajectory_id=11,candidate_id=12,
        state=ObservationFeedback.EXECUTION_READY))
    assert explorer.active_observation is not old
    assert window.committed_region==4
    assert explorer.active_commitment_scope==window.committed_scope
    assert explorer.branch_commitment is None
    assert explorer.region_switches==1


def test_rejected_publication_does_not_advance_the_region_window():
    from test_frontier_explorer import candidate_test_explorer, MODULE
    explorer=candidate_test_explorer();explorer.region_visit_window_enabled=True
    explorer.selection_strategy='hierarchical'
    window=explorer.visit_window();scope=window.commit(3)
    explorer.publish_path=lambda *a,**kw:False
    c=MODULE.FrontierCandidate(4,(6,3),(8,3),(6.5,3.5),[(1,2),(6,3)],5,10,5,0,{(9,2)})
    assert explorer.activate_candidate(c.path,c) is False
    assert window.committed_region==3 and window.committed_scope==scope


def test_production_pending_budget_cannot_fall_through_to_a_different_region():
    from test_frontier_explorer import candidate_test_explorer, MODULE
    explorer=candidate_test_explorer();explorer.region_visit_window_enabled=True
    explorer.selection_strategy='hierarchical';explorer.region_visit_frontiers=4
    explorer.region_visit_stations=2;explorer.region_visit_route_expansions=1
    explorer.active_region_id=None;explorer.active_commitment_scope=None
    explorer.region_sequence=[];explorer.publish_region_sequence=lambda:None
    explorer.publish_region_overlay=lambda *a:None
    rs=[MODULE.FrontierRegion(i,[[(i+2,2)]],(i+2,2),{(i+2,2)}) for i in range(4)]
    def pending(start,regions,*a,**kw):
        explorer._visit_pending_regions.add(regions[0].region_id)
        return []
    explorer.build_frontier_candidates=pending
    assert explorer.choose_window_visit((1,2),rs,set(),(),allow_region_release=True,
        allow_cross_region_preparation=False,required_scope=None) is None
    assert explorer.region_visit_deferred
    assert not explorer.visit_window().deferred


def test_installed_same_region_goal_keeps_scope_after_old_frontier_disappears():
    from test_frontier_explorer import pending_handoff_explorer, MODULE
    from scan_planner_msgs.msg import ObservationFeedback
    explorer=pending_handoff_explorer();explorer.region_visit_window_enabled=True
    window=explorer.visit_window();oldscope=window.commit(3)
    explorer.active_commitment_scope=explorer.last_selected_commitment_scope=oldscope
    pending=explorer.pending_observation_handoff
    pending.candidate.commitment_scope=window.scope(4)
    explorer.observation_feedback_callback(ObservationFeedback(
        task_id=pending.task.task_id,task_version=1,request_id=pending.request_id,
        trajectory_id=11,candidate_id=12,state=ObservationFeedback.EXECUTION_READY))
    scope=explorer.active_commitment_scope;switches=explorer.region_switches
    candidate=MODULE.FrontierCandidate(4,(7,3),(10,5),(7.5,3.5),[(1,2),(7,3)],6,1,1,0,{(11,5)})
    explorer.publish_path=lambda *a,**kw:True
    assert explorer.activate_candidate(candidate.path,candidate)
    assert explorer.active_commitment_scope==scope
    assert explorer.region_switches==switches
    assert explorer.branch_commitment is None


def test_reward_estimation_does_not_mutate_the_frozen_observation_targets():
    a=option(1,(1,0),targets={(9,9),(9,10)})
    a.visit_remaining_cells={(9,10)}
    result=choose_joint_visit([a],[],EuclideanRoutes(),depth=1)
    assert result.score==pytest.approx(1/(1+a.path_length/.5))
    assert a.observation_cells=={(9,9),(9,10)}


def test_connectivity_index_prevents_a_large_unreachable_python_search():
    grid=ExplorationGrid(150,100,1,0,0);grid.data.fill(FREE);grid.data[:,75]=OCCUPIED
    routes=BoundedVisitRoutes();routes.begin(grid,set(),1,revision=1)
    result=routes.query((1,1),(145,1))
    assert result.status==RouteSearchStatus.NO_ROUTE
    assert result.reason=='RAW_COMPONENT_DISCONNECTED'
    assert routes.budget.used==0 and routes.reachable.nbytes==15000
    old=routes.reachable
    routes.begin(grid,set(),1,revision=1)
    routes.query((2,1),(145,1))
    assert routes.reachable is old and routes.reachability_ms==0


def test_cache_capacity_is_a_scheduling_yield_not_no_route():
    grid=ExplorationGrid(30,20,1,0,0);grid.data.fill(FREE);grid.data[:17,15]=OCCUPIED
    routes=BoundedVisitRoutes(max_retained_expansions=4)
    routes.begin(grid,set(),10)
    result=routes.query((1,1),(28,1),quota=10)
    assert result.status==RouteSearchStatus.INCOMPLETE
    assert result.reason=='SEARCH_CACHE_CAPACITY'
    assert (28,1) in routes.capacity_yields
    assert len(routes.searches)==0


def test_moving_start_reuses_inflight_search_and_certifies_a_new_connector():
    grid=ExplorationGrid(30,20,1,0,0);grid.data.fill(FREE);grid.data[:17,15]=OCCUPIED
    routes=BoundedVisitRoutes();routes.begin(grid,set(),5,revision=1)
    assert routes.query((1,1),(28,1),quota=5).status==RouteSearchStatus.INCOMPLETE
    first=routes.searches[((1,1),(28,1))]
    routes.begin(grid,set(),5,revision=1)
    assert routes.query((2,1),(28,1),quota=5).status==RouteSearchStatus.INCOMPLETE
    assert routes.searches[((1,1),(28,1))] is first
    assert ((2,1),(28,1)) not in routes.searches
    for _ in range(100):
        routes.begin(grid,set(),32,revision=1)
        result=routes.query((2,1),(28,1),quota=32)
        if result.status==RouteSearchStatus.FOUND:break
    assert result.status==RouteSearchStatus.FOUND
    assert result.path[0]==(2,1) and result.path[-1]==(28,1)
