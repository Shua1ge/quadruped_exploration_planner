"""Station alternatives, persistent information domains and next-visit costs."""
from types import SimpleNamespace as NS

from explorer_core.grid import ExplorationGrid, FREE, UNKNOWN, OCCUPIED
from explorer_core.observation_contract import bind_task_identity
from explorer_core.observation_lifecycle import ObservationTask, evaluate_observation
from explorer_core.region_visits import RegionVisitWindow, choose_joint_visit, same_visit_demand
from explorer_core.region_visits import BoundedVisitRoutes
from explorer_core.region_visits import station_alternatives, VisitCostModel, _motion_shape
from explorer_core.grid import bresenham
import math
import pytest
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


def test_retained_demand_identity_does_not_force_an_uncompetitive_visit_order():
    a=option(1,(2,0),frontier=(6,2))
    b=option(1,(4,0),frontier=(8,2),targets={(200+i,0) for i in range(10)})
    tempting=option(1,(1,0),targets={(300+i,0) for i in range(500)})
    a.visit_demand_id=17;b.visit_demand_id=18;tempting.visit_demand_id=19
    result=choose_joint_visit([tempting,a,b],[],EuclideanRoutes(),
                             retained_demands=(17,18),depth=2)
    assert result.candidate is tempting and result.reordered
    assert a.visit_demand_id==17 and b.visit_demand_id==18
    assert a.frontier_cell==(6,2)  # reordering never edits the evidence domain


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
    plan=attach_visit_plan(e,c,next_demand=False)
    e.prepared_candidate=c;e.directional_candidate_allowed=lambda *a:True
    activated=[];e.activate_candidate=lambda path,c:activated.append(c) or True
    def forbidden(*a):raise AssertionError('A prepared demand must not be refrozen')
    monkeypatch.setattr(MODULE,'observation_target_cells',forbidden)
    assert e.activate_prepared_observation(True)
    assert activated[0].visit_demand_id==c.visit_demand_id
    assert activated[0].visit_plan is plan
    assert activated[0].visit_demand_ref is c.visit_demand_ref
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


def test_unavailable_future_preserves_the_current_joint_visit_not_a_greedy_station():
    class FutureDisconnected(EuclideanRoutes):
        def transition(self,a,b):
            return (float('inf'),None) if b[0]>=20 else super().transition(a,b)
    near=option(1,(0,2),frontier=(3,3),length=2)
    aligned=option(1,(4,0),frontier=(3,3),length=4)
    later=option(1,(8,0),targets={(200+i,0) for i in range(10)})
    future=option(2,(20,0))
    baseline=choose_joint_visit([near,aligned,later],[],EuclideanRoutes(),depth=2)
    result=choose_joint_visit([near,aligned,later],[[future]],FutureDisconnected(),depth=2)
    assert baseline.candidate is aligned and result.candidate is aligned
    assert len(result.prefix)==2 and result.score>0
    assert result.preview_unavailable and result.preview_regions_priced==0
    assert result.continuation_estimated
    assert result.estimated_time_s==pytest.approx(baseline.estimated_time_s)


def test_second_future_failure_retains_the_common_first_future_cost():
    class SecondDisconnected(EuclideanRoutes):
        def transition(self,a,b):
            return (float('inf'),None) if b[0]>=30 else super().transition(a,b)
    a=option(1,(2,0));b=option(1,(4,0),targets={(200+i,0) for i in range(10)})
    result=choose_joint_visit([a,b],[[option(2,(20,0))],[option(3,(30,0))]],SecondDisconnected())
    assert len(result.prefix)==2 and result.preview_unavailable
    assert result.preview_regions_priced==1 and result.continuation_time_s>0
    assert result.score>0


def test_unavailable_tail_does_not_make_its_prefix_artificially_cheaper():
    class PartialFuture(EuclideanRoutes):
        def transition(self,a,b):
            if a[1]==2 and b[0]>=20:return float('inf'),None
            return super().transition(a,b)
    a=option(1,(2,0));b=option(1,(3,2))
    baseline=choose_joint_visit([a,b],[],EuclideanRoutes(),depth=1)
    result=choose_joint_visit([a,b],[[option(2,(20,0))]],PartialFuture(),depth=1)
    assert result.candidate is baseline.candidate
    assert result.preview_unavailable and result.preview_regions_priced==0


def test_low_gain_retained_work_can_move_later_without_being_marked_complete():
    small=option(1,(-5,0),targets={(10+i,0) for i in range(8)})
    useful=option(1,(2,0),targets={(200+i,0) for i in range(300)})
    small.visit_demand_id=1;useful.visit_demand_id=2
    frozen=set(small.observation_cells)
    result=choose_joint_visit([small,useful],[],EuclideanRoutes(),body_yaw=0,
                             retained_demands=(1,2),depth=2)
    assert result.candidate is useful and result.reordered
    assert small.visit_demand_id==1 and small.observation_cells==frozen
    assert 1 in result.demand_prefix  # residual work survives reordering


def test_time_estimate_prices_a_short_reverse_above_a_slightly_longer_forward_visit():
    reverse=option(1,(-1,0));forward=option(1,(3,0))
    result=choose_joint_visit([reverse,forward],[],EuclideanRoutes(),body_yaw=0,depth=1)
    assert result.candidate is forward
    reverse.observation_cells={(i,0) for i in range(40)}
    result=choose_joint_visit([reverse,forward],[],EuclideanRoutes(),body_yaw=0,depth=1)
    assert result.candidate is reverse  # a preference, never a direction veto
    assert result.turn_time_s==pytest.approx(math.pi/.8)
    assert result.stop_turn_time_s==1.
    assert result.estimated_time_s==pytest.approx(
        result.travel_time_s+result.turn_time_s+result.stop_turn_time_s+result.continuation_time_s)


def test_grid_stair_steps_are_not_priced_as_repeated_body_turns():
    path=bresenham((0,0),(70,30));frozen=list(path)
    entry,exit,bends=_motion_shape(path,.2)
    assert abs(entry-math.atan2(30,70))<.15
    assert abs(exit-math.atan2(30,70))<.15
    assert sum(bends)<math.pi
    assert not any(b>.8 for b in bends)
    assert path==frozen
    _,_,corner=_motion_shape([(0,0),(10,0),(10,10)],1.)
    assert sum(corner)==pytest.approx(math.pi/2)


@pytest.mark.parametrize('kwargs',[
    {'linear_speed':0},{'yaw_rate':float('nan')},{'stop_turn_angle':4},
    {'stop_restart_time':-1},{'reorder_ratio':.9}])
def test_invalid_ranking_estimates_are_rejected_at_configuration(kwargs):
    with pytest.raises(ValueError):VisitCostModel(**kwargs)


# These use the real publication/typed-ACK bridge without spinning ROS or Local.
def attach_visit_plan(e, candidate, *, next_demand=True):
    w=e.visit_window()
    d=w.freeze_demand(candidate.region_id,candidate.frontier_cell,candidate.observation_cells)
    candidate.visit_demand_id=d.demand_id;candidate.visit_demand_ref=d
    ids=(d.demand_id,);frontiers=(candidate.frontier_cell,);stations=(candidate.cell,)
    if next_demand:
        other=w.freeze_demand(candidate.region_id,(9,3),{(10,3)})
        ids+=(other.demand_id,);frontiers+=(other.frontier,);stations+=((7,3),)
    candidate.visit_plan=w.draft(NS(demand_prefix=ids,prefix=frontiers,station_prefix=stations),candidate.region_id)
    return candidate.visit_plan


def ready(e, task=None, request=None, **overrides):
    from scan_planner_msgs.msg import ObservationFeedback
    pending=e.pending_observation_handoff
    task=task or (pending.task if pending else e.active_observation)
    request=request if request is not None else (pending.request_id if pending else e.active_path_request_generation)
    values=dict(task_id=task.task_id,task_version=task.task_version,request_id=request,
                trajectory_id=11,candidate_id=12,state=ObservationFeedback.EXECUTION_READY)
    values.update(overrides)
    e.observation_feedback_callback(ObservationFeedback(**values))


def planned_pending():
    e=pending_handoff_explorer()
    c=e.pending_observation_handoff.candidate
    # Re-stage so the plan really travels through production publication.
    e.pending_observation_handoff=None
    plan=attach_visit_plan(e,c)
    assert e.stage_observation_handoff(c.path,c)
    return e,plan


def test_planning_and_publishing_do_not_install_a_visit_prefix():
    e,plan=planned_pending();w=e.visit_window()
    assert w.plan_serial==1 and w.installed_plan is None and not w.demand_prefix
    assert e.pending_observation_handoff.task.visit_plan is plan
    ready(e)
    assert w.installed_plan is plan and w.demand_prefix==plan.demand_ids
    assert w.prefix==plan.frontiers


def test_exact_ack_commits_its_published_snapshot_despite_a_newer_draft():
    e,plan=planned_pending();w=e.visit_window();pending=e.pending_observation_handoff
    newer=w.draft(NS(demand_prefix=plan.demand_ids[::-1],prefix=plan.frontiers[::-1],
                     station_prefix=plan.stations[::-1]),plan.region_id)
    assert newer.plan_id>plan.plan_id and w.installed_plan is None
    ready(e)
    assert e.active_observation is pending.task and w.installed_plan is plan
    assert w.demand_prefix==plan.demand_ids


@pytest.mark.parametrize('override',[
    {'request_id':2},{'task_version':99},{'trajectory_id':0},{'candidate_id':0}])
def test_nonmatching_or_uncertified_ack_never_advances_the_prefix(override):
    e,plan=planned_pending()
    ready(e,**override)
    assert e.visit_window().installed_plan is None
    assert e.pending_observation_handoff is not None


def test_cancelled_pending_and_late_ack_keep_the_installed_prefix():
    e,plan=planned_pending();w=e.visit_window()
    w.install(plan,(90,1,90))
    pending=e.pending_observation_handoff
    e.cancel_pending_observation_handoff('TEST_REJECTED')
    ready(e)
    assert pending.state=='CANCELLING'
    assert w.installed_request==(90,1,90)
    assert not any(d.retired for d in plan.demands)


def test_failed_publication_leaves_existing_prefix_and_live_demands_intact():
    e,plan=planned_pending();w=e.visit_window()
    pending=e.pending_observation_handoff;e.pending_observation_handoff=None
    w.install(plan,(90,1,90))
    e.publish_reference_path=lambda *a,**kw:False
    assert not e.stage_observation_handoff(pending.path,pending.candidate)
    assert w.installed_request==(90,1,90)
    assert w.demand_prefix==plan.demand_ids and not any(d.retired for d in plan.demands)


def test_ordinary_initial_publication_waits_for_its_own_installation_ack():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None
    plan=attach_visit_plan(e,c)
    assert e.publish_path(c.path,c)
    assert e.active_observation.visit_plan is plan
    assert e.visit_window().installed_plan is None
    # Information can finish while the motion ACK is still in transit.
    e.active_observation.information_satisfied=True
    ready(e)
    assert e.visit_window().installed_plan is plan
    key=e.visit_window().installed_request
    ready(e)
    assert e.visit_window().installed_request==key


def test_completion_drops_only_its_scheduling_refs_before_prepared_handoff():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    c.region_id=old.region_id;c.frontier_cell=(8,2);c.observation_cells=set(old.target_cells)
    plan=attach_visit_plan(e,c);w=e.visit_window()
    old.visit_demand_id=c.visit_demand_id;old.visit_demand_ref=c.visit_demand_ref;old.visit_plan=plan
    old.information_satisfied=False;w.install(plan,(old.task_id,1,100))
    e.last_handoff_attempt_update=-1
    e.grid.data[2,9]=UNKNOWN
    def prepared(**kw):
        assert c.visit_demand_ref.retired
        assert old.visit_demand_id not in w.demands[old.region_id]
        assert w.demand_prefix==plan.demand_ids[1:]
        e.pending_observation_handoff=NS()  # an actual pending successor ends this callback
        return True
    e.activate_or_prepare_next_observation=prepared
    assert e._handoff_satisfied_observation('OBSERVATION_COMPLETE')
    assert old.information_satisfied and e.grid.value((9,2))==UNKNOWN


def test_completed_cached_demand_cannot_reopen_after_switching_to_another_task():
    e=pending_handoff_explorer();b=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    a=MODULE.FrontierCandidate(3,(6,2),(8,2),(6.5,2.5),[(1,2),(6,2)],5,1,1,0,set(old.target_cells))
    plan=attach_visit_plan(e,a)
    old.visit_demand_id=a.visit_demand_id;old.visit_demand_ref=a.visit_demand_ref;old.visit_plan=plan
    old.require_sensor_evidence=True;old.initial_sensor_revision=2
    e.grid.data[2,9]=UNKNOWN;e.grid.sensor_revision=10
    e.retire_visit_demand(old)
    assert e.publish_path(b.path,b)  # a different task has become active
    new=e.active_observation
    assert not e.publish_path(a.path,a)
    assert not e.stage_observation_handoff(a.path,a)
    e.prepared_candidate=a
    assert not e.activate_prepared_observation(True) and e.prepared_candidate is None
    assert e.active_observation is new and e.grid.value((9,2))==UNKNOWN


def test_incomplete_station_capture_keeps_the_cached_demand_and_sensor_baseline():
    e,plan=planned_pending();task=e.pending_observation_handoff.task
    task.motion_released=True;task.information_satisfied=False
    task.require_sensor_evidence=True;task.initial_sensor_revision=2
    e.retire_visit_demand(task)
    w=e.visit_window()
    assert task.visit_demand_id in w.demands[task.region_id]
    assert not task.visit_demand_ref.retired and task.initial_sensor_revision==2


def test_retired_information_does_not_veto_an_already_installed_pending_motion():
    e,plan=planned_pending();pending=e.pending_observation_handoff
    pending.task.information_satisfied=True
    e.retire_visit_demand(pending.task)
    ready(e)
    assert e.active_observation is pending.task
    assert e.visit_window().installed_plan is plan
    assert e.visit_window().demand_prefix==plan.demand_ids[1:]


def test_priced_prefix_does_not_append_an_unpriced_retained_demand():
    a=option(1,(1,0));b=option(1,(2,0))
    a.visit_demand_id=1;b.visit_demand_id=2  # identical reward: second leg has no marginal gain
    result=choose_joint_visit([a,b],[],EuclideanRoutes(),retained_demands=(1,2))
    assert result.demand_prefix==(1,)
    assert len(result.prefix)==len(result.station_prefix)==len(result.demand_prefix)


def test_committed_station_is_retained_for_small_changes_and_revalidated_if_absent():
    a=option(1,(2,0),frontier=(8,2));b=option(1,(2,1),frontier=(8,2),length=2.05)
    a.visit_demand_id=b.visit_demand_id=1
    result=choose_joint_visit([a,b],[],EuclideanRoutes(),depth=1,
        retained_demands=(1,),retained_stations=((1,b.cell),))
    assert result.candidate is b
    result=choose_joint_visit([a],[],EuclideanRoutes(),depth=1,
        retained_demands=(1,),retained_stations=((1,b.cell),))
    assert result.candidate is a  # retaining a plan never revives a missing station


def test_draft_and_capacity_deferral_cannot_erase_the_installed_prefix():
    w=RegionVisitWindow();d=w.freeze_demand(1,(8,2),{(9,2)})
    choice=NS(demand_prefix=(d.demand_id,),prefix=(d.frontier,),station_prefix=((6,2),))
    installed=w.draft(choice,1);w.install(installed,(1,1,1))
    for _ in range(20):w.draft(choice,1)
    w.defer(1,2)
    assert w.installed_plan is installed and w.demand_prefix==(d.demand_id,)
    assert len(w.demands[1])==1  # no draft history or permanent completed-ID ledger


def test_supplemental_station_ack_cannot_reopen_information_completed_in_flight():
    e=pending_handoff_explorer();c=e.pending_observation_handoff.candidate
    e.pending_observation_handoff=None;old=e.active_observation
    old.information_satisfied=False;old.motion_released=True
    c.region_id=old.region_id;c.observation_cells=set(old.target_cells)
    plan=attach_visit_plan(e,c)
    old.visit_demand_id=c.visit_demand_id;old.visit_demand_ref=c.visit_demand_ref;old.visit_plan=plan
    old.require_sensor_evidence=True;old.initial_sensor_revision=2
    assert e.stage_observation_handoff(c.path,c)
    pending=e.pending_observation_handoff
    old.information_satisfied=True;old.progress=.9;old.completion_reason='visible_information_resolved'
    old.satisfied_sensor_revision=12
    e.retire_visit_demand(old)
    ready(e)
    assert e.active_observation is pending.task
    assert e.active_observation.information_satisfied
    assert e.active_observation.progress==.9 and e.active_observation.satisfied_sensor_revision==12
    assert e.active_observation.initial_sensor_revision==2
    assert e.visit_window().demand_prefix==plan.demand_ids[1:]


def test_production_selection_keeps_installed_prefix_while_drafting_a_new_one():
    e=candidate_test_explorer();e.region_visit_window_enabled=True
    e.selection_strategy='hierarchical';e.region_visit_frontiers=4
    e.region_visit_stations=2;e.region_visit_route_expansions=100
    e.active_region_id=None;e.active_commitment_scope=None
    e.region_sequence=[];e.publish_region_sequence=lambda:None;e.publish_region_overlay=lambda *a:None
    w=e.visit_window();d=w.freeze_demand(3,(8,2),{(9,2)})
    installed=w.draft(NS(demand_prefix=(d.demand_id,),prefix=(d.frontier,),station_prefix=((6,2),)),3)
    w.install(installed,(10,1,10));e.grid.data[2,9]=UNKNOWN
    c=MODULE.FrontierCandidate(3,(6,2),(8,2),(6.5,2.5),[(1,2),(6,2)],5,1,1,0,{(9,2)},
        visit_demand_id=d.demand_id,visit_demand_ref=d,visit_visible_cells={(9,2)})
    e.build_frontier_candidates=lambda *a,**kw:[c]
    region=MODULE.FrontierRegion(3,[[(8,2)]],(8,2),{(8,2)})
    result=e.choose_window_visit((1,2),[region],set(),(),allow_region_release=True,
        allow_cross_region_preparation=True,required_scope=None)
    assert result is c and c.visit_plan is not installed
    assert w.installed_plan is installed and w.installed_request==(10,1,10)
    assert e.last_visit_stats['plan_state']=='DRAFT'
    assert e.last_visit_stats['installed_plan_id']==installed.plan_id
