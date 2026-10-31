"""Station alternatives, persistent information domains and next-visit costs."""
from types import SimpleNamespace as NS

from explorer_core.grid import ExplorationGrid, FREE, UNKNOWN, OCCUPIED
from explorer_core.observation_contract import bind_task_identity
from explorer_core.observation_lifecycle import ObservationTask, evaluate_observation
from explorer_core.region_visits import RegionVisitWindow, choose_joint_visit, same_visit_demand
from explorer_core.region_visits import BoundedVisitRoutes
from explorer_core.region_visits import station_alternatives
from test_region_visits import option, EuclideanRoutes
from test_frontier_explorer import MODULE, candidate_test_explorer, pending_handoff_explorer


def grid():
    g=ExplorationGrid(14,10,1,0,0)
    g.data.fill(FREE)
    return g


def boundary(rid, *cells):
    return NS(region_id=rid, clusters=[list(cells)], centroid=cells[0])


def test_station_choice_prices_the_next_demand_inside_the_same_region():
    near=option(1,(0,2),frontier=(3,3),length=2)
    aligned=option(1,(4,0),frontier=(3,3),length=4)
    later=option(1,(8,0),targets={(200+i,0) for i in range(10)})
    near.visit_demand_id=aligned.visit_demand_id=1
    later.visit_demand_id=2
    assert choose_joint_visit([near,aligned],[],EuclideanRoutes()).candidate is near
    choice=choose_joint_visit([near,aligned,later],[],EuclideanRoutes(),depth=2)
    assert choice.candidate is aligned
    assert choice.demand_prefix==(1,2)


def test_station_budget_retains_a_distinct_sibling_not_just_two_adjacent_poses():
    assert station_alternatives([(2,2),(2,3),(6,2),(6,3)],2)==[(2,2),(6,3)]


def test_different_frontier_views_of_one_demand_cannot_be_visited_twice():
    a=option(1,(1,0),frontier=(3,3))
    alternative=option(1,(2,0),frontier=(4,3))
    b=option(1,(3,0),targets={(200+i,0) for i in range(10)})
    a.visit_demand_id=alternative.visit_demand_id=7
    b.visit_demand_id=8
    choice=choose_joint_visit([a,alternative,b],[],EuclideanRoutes(),depth=3)
    assert len(choice.demand_prefix)==2
    assert set(choice.demand_prefix)=={7,8}


def test_redundant_prediction_does_not_add_a_second_visit():
    a=option(1,(1,0));b=option(1,(2,0))
    a.visit_demand_id=1;b.visit_demand_id=2
    choice=choose_joint_visit([a,b],[],EuclideanRoutes())
    assert len(choice.demand_prefix)==1
    assert a.observation_cells==b.observation_cells


def test_retained_demand_survives_a_frontier_cell_change_and_gain_competition():
    a=option(1,(2,0),frontier=(6,2))
    b=option(1,(4,0),frontier=(8,2),targets={(200+i,0) for i in range(10)})
    tempting=option(1,(1,0),targets={(300+i,0) for i in range(500)})
    a.visit_demand_id=17;b.visit_demand_id=18;tempting.visit_demand_id=19
    result=choose_joint_visit([tempting,a,b],[],EuclideanRoutes(),
                             retained_demands=(17,18),depth=2)
    assert result.candidate is a and result.demand_prefix==(17,18)


def test_disjoint_old_new_boundaries_can_view_the_same_fixed_demand():
    g=grid();g.data[2,10]=g.data[3,10]=UNKNOWN
    w=RegionVisitWindow()
    old=w.freeze_demand(1,(8,2),{(10,2),(10,3)})
    w.prefix_region=1;w.demand_prefix=(old.demand_id,)
    new=boundary(1,(8,3))
    assert w.sample_demands(new,[(8,3)],g,4,3)==[(8,3)]
    rebound=w.freeze_demand(1,(8,3),{(11,3)})
    assert rebound is old
    assert rebound.targets==frozenset({(10,2),(10,3)})
    assert not ({(8,2)} & {(8,3)})


def test_a_wall_prevents_reusing_an_unrelated_demand_across_it():
    g=grid();g.data[:,9]=OCCUPIED;g.data[2,10]=UNKNOWN
    w=RegionVisitWindow();old=w.freeze_demand(1,(10,3),{(10,2)})
    assert w.sample_demands(boundary(1,(7,3)),[(7,3)],g,4,4)==[(7,3)]
    assert w.sampled_demands.get((1,(7,3))) is None
    assert old.demand_id not in w.demands[1]
    assert g.value((10,2))==UNKNOWN  # scheduling release is not completion


def test_the_demand_batch_is_retained_while_the_inputs_are_unchanged():
    g=grid();g.data[4,10]=UNKNOWN
    w=RegionVisitWindow();d=w.freeze_demand(1,(8,4),{(10,4)})
    r=boundary(1,(8,4),(8,5),(8,6),(7,6),(6,6))
    w.prefix_region=1;w.demand_prefix=(d.demand_id,)
    for _ in range(5):
        chosen=w.sample_demands(r,list(r.clusters[0]),g,1,3)
        assert chosen==[(8,4)]
        assert w.freeze_demand(1,chosen[0],{(11,4)}) is d


def test_production_station_alternatives_share_targets_but_have_distinct_visibility(monkeypatch):
    e=candidate_test_explorer();e._visit_pending_regions=set()
    e.grid.data[2,9]=e.grid.data[3,9]=UNKNOWN
    monkeypatch.setattr(MODULE,'candidate_cells',lambda c:[(8,2)])
    monkeypatch.setattr(MODULE,'safe_viewpoint_cells',lambda *a,**k:[(6,2),(6,3)])
    visible={(6,2):{(9,2)},(6,3):{(9,3)}}
    monkeypatch.setattr(MODULE,'observation_target_cells',lambda g,f,r,v:visible[v])
    monkeypatch.setattr(MODULE,'observation_hypothesis_cells',lambda g,v,t,r:set(t)&visible[v])
    routes=BoundedVisitRoutes();routes.begin(e.grid,set(),100)
    r=MODULE.FrontierRegion(3,[[(8,2)]],(8,2),{(8,2)})
    records=e.build_frontier_candidates((1,2),[r],set(),set(),
        frontier_limit=4,station_limit=2,route_provider=routes)
    assert len(records)==2
    assert len({c.visit_demand_id for c in records})==1
    assert all(c.observation_cells=={(9,2),(9,3)} for c in records)
    assert [c.visit_visible_cells for c in records]==[{(9,2)},{(9,3)}]
    assert [c.unknown_gain for c in records]==[1,1]


def test_prepared_activation_keeps_the_fixed_target_domain(monkeypatch):
    e=candidate_test_explorer();e.grid.data[2,9]=UNKNOWN
    e.current_blocked_edges=lambda:set()
    c=MODULE.FrontierCandidate(4,(6,2),(8,2),(6.5,2.5),[(1,2),(6,2)],
        5.,1,1,0.,{(9,2)},visit_demand_id=42)
    e.prepared_candidate=c;e.directional_candidate_allowed=lambda *a:True
    activated=[];e.activate_candidate=lambda path,c:activated.append(c) or True
    def forbidden(*a):raise AssertionError('A prepared demand must not be refrozen')
    monkeypatch.setattr(MODULE,'observation_target_cells',forbidden)
    assert e.activate_prepared_observation(True)
    assert activated[0].visit_demand_id==42
    assert activated[0].observation_cells=={(9,2)}


def test_same_demand_same_task_keeps_measurements_when_the_boundary_moves():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    old.information_satisfied=False;old.visit_demand_id=c.visit_demand_id=7
    old.require_sensor_evidence=True;old.initial_sensor_revision=2;old.progress=.6
    c.region_id=old.region_id;c.frontier_cell=(8,3)
    assert e.publish_path(c.path,c)
    assert e.active_observation is old and old.progress==.6
    assert old.initial_sensor_revision==2
    assert same_visit_demand(old,c)


def test_supplemental_station_handoff_preserves_the_original_measurement_baseline():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    old.visit_demand_id=c.visit_demand_id=9
    old.information_satisfied=False;old.motion_released=True
    old.require_sensor_evidence=True;old.initial_sensor_revision=2
    c.region_id=old.region_id;c.observation_cells=set(old.target_cells)
    e.grid.sensor_revision=10
    assert e.stage_observation_handoff(c.path,c)
    staged=e.pending_observation_handoff.task
    assert staged.task_id!=old.task_id
    assert staged.target_cells==old.target_cells
    assert staged.initial_sensor_revision==2
    assert staged.visit_demand_id==9


def test_completed_task_cannot_be_reactivated_by_a_cached_demand():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    old.visit_demand_id=c.visit_demand_id=11
    c.region_id=old.region_id
    assert not e.publish_path(c.path,c)
    assert e.active_observation is old
    assert old.information_satisfied


def test_installation_ack_still_commits_if_predecessor_information_completed_late():
    from scan_planner_msgs.msg import ObservationFeedback
    e=pending_handoff_explorer();pending=e.pending_observation_handoff
    old=e.active_observation
    old.visit_demand_id=pending.candidate.visit_demand_id=pending.task.visit_demand_id=12
    pending.candidate.region_id=pending.task.region_id=old.region_id
    old.information_satisfied=True
    e.observation_feedback_callback(ObservationFeedback(task_id=pending.task.task_id,
        task_version=1,request_id=pending.request_id,trajectory_id=11,candidate_id=12,
        state=ObservationFeedback.EXECUTION_READY))
    assert e.active_observation is pending.task
    assert e.active_path_request_generation==pending.request_id


def test_prediction_alone_never_advances_the_task_measurement_ledger():
    g=grid();g.data[4,10]=UNKNOWN
    task=bind_task_identity(ObservationTask(1,(6,4),{(10,4)},visit_demand_id=2),10,g)
    c=option(1,(6,4),targets={(10,4)});c.visit_demand_id=2
    choose_joint_visit([c],[],EuclideanRoutes())
    evaluate_observation(task,g,None,1,.8,2,2,1.)
    assert task.observed_cells==0 and task.progress==0
    assert task.completion_streak==0 and not task.information_satisfied
