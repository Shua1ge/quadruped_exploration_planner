import pathlib
import sys
from collections import deque
from types import SimpleNamespace

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/"scripts"))
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from scan_planner_msgs.msg import ObservationFeedback, ObservationTask as TaskMessage
from explorer_core.grid import ExplorationGrid
from explorer_core.observation_lifecycle import ObservationTask
from explorer_core.observation_task_bridge import ObservationTaskBridge


def bridge():
    value=ObservationTaskBridge()
    value.task_mode=True
    value.grid=ExplorationGrid(12,12,1,0,0)
    value.frame_id="world"
    value.last_observation_task_id=0
    value.tasks_created=value.task_requests_published=value.task_cancellations=0
    value.task_stations_reached=value.task_local_rejections=value.task_stale_feedback=0
    value.task_corridor_radius=.8
    value.observation_radius=3.0
    value.task_representative_limit=4
    value.task_max_station_adjustments=3
    value.task_feedback_seen=deque(maxlen=256)
    value.position=(2.,2.)
    value.observation_motion_released_count=0
    value.map_update_count=5
    value.messages=[]
    value.observation_task_pub=SimpleNamespace(publish=value.messages.append)
    now=SimpleNamespace(nanoseconds=1_000_000_000,to_msg=lambda:Time(sec=1))
    value.get_clock=lambda:SimpleNamespace(now=lambda:now)
    value.departure_is_settled=lambda:True
    value.recovery_handoffs=[]
    value.planning_status_callback=lambda msg:value.recovery_handoffs.append(msg.data)
    value.active_observation=ObservationTask(7,(3.,2.),{(x,5) for x in range(8)})
    value.evaluate_active_observation=lambda:value.active_observation
    value.observation_complete=lambda task:False
    return value


def path():
    message=Path()
    message.header.frame_id="world"
    message.poses=[PoseStamped(),PoseStamped()]
    message.poses[1].pose.position.x=3.0
    return message


def test_autonomy_arrival_collects_scans_then_hands_off_without_station_retry():
    value=bridge(); value.task_geometry_autonomy=True
    value.publish_observation_task(path(),100)
    assert value.messages[-1].local_geometry_autonomy
    assert not value.messages[-1].has_directional_gate
    task=value.active_observation
    value.active_path_request_generation=100
    value.observation_feedback_callback(ObservationFeedback(task_id=100,task_version=1,
        request_id=100,candidate_id=12,state=ObservationFeedback.STATION_REACHED,
        station=Point(x=2.,y=2.,z=.4)))
    value.grid.sensor_revision+=1
    value.active_raw_path=[(0,0),(3,0)]
    routes=[]
    value.reroute_active_goal=lambda: (_ for _ in ()).throw(AssertionError("Explorer must not redo Local geometry"))
    value.publish_reference_path=lambda cells: routes.append(cells)
    handoffs=[]
    value.handoff_satisfied_observation=handoffs.append
    assert value.continue_observation_station() and not routes and not handoffs
    value.grid.sensor_revision+=2
    assert value.continue_observation_station()
    assert routes==[] and value.active_observation is task
    assert handoffs==["OBSERVATION_STATION_MOTION_RELEASED"]
    assert task.motion_released and not task.information_satisfied
    assert value.observation_motion_released_count==1
    assert not value.continue_observation_station()
    assert task.completion_reason=="none"


def test_motion_exhaustion_is_nonspatial_terminal_and_deduplicated():
    value=bridge(); value.task_geometry_autonomy=True
    value.publish_observation_task(path(),100); value.active_path_request_generation=100
    value.active_goal=(3.,2.); value.active_region_id=7; value.active_commitment_scope=("region",7)
    value.local_execution_context=lambda:(1,2,3)
    decisions=[]; terminals=[]; statuses=[]; resets=[]
    value.replan_gate=SimpleNamespace(reset=lambda:resets.append(1))
    value.terminate_active_observation=lambda reason: terminals.append(reason)
    value.plan_from_current_position=lambda **kwargs:decisions.append(kwargs) or True
    value.publish_status=statuses.append
    value.reroute_active_goal=lambda: (_ for _ in ()).throw(AssertionError("second Local retry ledger"))
    value.apply_global_viewpoint_constraint=lambda *args: (_ for _ in ()).throw(AssertionError("no spatial proof"))
    message=ObservationFeedback(task_id=100,task_version=1,request_id=100,candidate_id=12,
        state=ObservationFeedback.LOCAL_MOTION_EXHAUSTED,station=Point(x=2.,y=2.,z=.4))
    value.observation_feedback_callback(message); value.observation_feedback_callback(message)
    assert terminals==["LOCAL_MOTION_EXHAUSTED"] and resets==[1]
    assert decisions==[dict(excluded_goals=((3.,2.),),required_scope=("region",7),allow_region_release=False)]
    assert value.local_motion_goal_cooldowns[(3.,2.)].position==(2.,2.)
    assert value.active_observation.completion_reason=="none"


def test_legacy_motion_failure_without_message_keeps_reason_and_arbitrates():
    value=bridge();value.active_goal=(3.,2.);value.active_region_id=7
    reasons=[];terminals=[];decisions=[]
    value.remember_local_motion_attempt=lambda goal,reason,**kwargs:reasons.append(reason)
    value.terminate_active_observation=terminals.append
    value.replan_gate=SimpleNamespace(reset=lambda:None)
    value.plan_from_current_position=lambda **kw:decisions.append(kw) or True
    value.publish_status=lambda status:None
    value.handle_local_motion_exhausted(reason="HEADING_ALIGNMENT_INEFFECTIVE")
    assert reasons==["HEADING_ALIGNMENT_INEFFECTIVE"]
    assert terminals==["LOCAL_MOTION_EXHAUSTED"] and len(decisions)==1
    value.handle_local_motion_exhausted(None)
    assert reasons[-1]=="LOCAL_MOTION_EXHAUSTED"


def test_failed_side_step_penalizes_same_departure_without_vetoing_local():
    value=bridge();value.task_geometry_autonomy=True;value.body_yaw=0.
    value.active_goal=(3.,2.);value.active_region_id=7
    decisions=[];statuses=[]
    value.publish_status=statuses.append
    value.terminate_active_observation=lambda reason:None
    value.replan_gate=SimpleNamespace(reset=lambda:None)
    value.plan_from_current_position=lambda **kwargs:decisions.append(kwargs) or True
    value.handle_local_motion_exhausted(ObservationFeedback(reason="LATERAL_RECOVERY_REJOIN_FAILED",
        trigger_reason="HANDOFF_ALIGNMENT_DEVIATED",recovery_reason="LATERAL_RECOVERY_OVERSHOOT",
        planning_reason="LOCAL_PATH_FAMILY_EXHAUSTED"))
    assert len(decisions)==1 and value.local_departure_replan_allowed()
    east=SimpleNamespace(goal=(8.,2.),path=[])
    north=SimpleNamespace(goal=(2.,8.),path=[])
    assert value.local_departure_history_penalty(east)>value.local_departure_history_penalty(north)
    assert any("trigger=HANDOFF_ALIGNMENT_DEVIATED" in s and "recovery=LATERAL_RECOVERY_OVERSHOOT" in s for s in statuses)
    value.position=(2.02,2.);value.body_yaw=.02
    value.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
    assert value.local_departure_history_penalty(east)>0 # Clock does not erase evidence.
    value.position=(2.31,2.)
    assert value.local_departure_history_penalty(east)==0.


def test_budget_rejections_cannot_fabricate_eight_blocked_directions():
    import math
    value=bridge();value.body_yaw=0.
    for i in range(8):
        angle=i*math.pi/4
        goal=(2.+5*math.cos(angle),2.+5*math.sin(angle))
        value.remember_local_motion_attempt(goal,"LATERAL_RECOVERY_ALREADY_USED")
    assert not getattr(value,"departure_action_failures",[])
    for i in range(64):
        angle=i*2*math.pi/64
        assert value.local_departure_history_penalty(SimpleNamespace(
            goal=(2.+5*math.cos(angle),2.+5*math.sin(angle)),path=[]))==0.


def test_same_entry_history_is_a_cost_and_another_branch_still_reaches_local():
    value=bridge();value.body_yaw=0.
    value.remember_local_motion_attempt((9.,2.),"LATERAL_RECOVERY_REJOIN_FAILED",cells=[(2,2),(5,2),(9,2)])
    # Route first goes east then turns north; its distant goal must not bypass memory.
    east=SimpleNamespace(goal=(5.,9.),path=[(2,2),(5,2),(5,9)])
    north=SimpleNamespace(goal=(5.,9.),path=[(2,2),(2,5),(5,9)])
    assert value.local_departure_history_penalty(east)>value.local_departure_history_penalty(north)


def test_departure_key_uses_remaining_route_not_old_task_start():
    import math
    value=bridge();value.position=(5.5,2.5)
    yaw=value.departure_route_yaw((9.5,2.5),[(1,2),(9,2)])
    assert abs(yaw)<1e-9


def test_typed_current_pose_failure_is_not_reduced_to_an_action():
    value=bridge();value.active_goal=(3.,2.);value.active_region_id=7
    value.publish_status=lambda status:None
    value.terminate_active_observation=lambda reason:None
    value.plan_from_current_position=lambda **kwargs: (_ for _ in ()).throw(AssertionError("pose blocked"))
    value.handle_local_motion_exhausted(ObservationFeedback(reason="LATERAL_RECOVERY_REJOIN_FAILED",
        failure_scope=ObservationFeedback.FAILURE_CURRENT_POSE))
    assert not value.local_departure_replan_allowed()


def test_transition_phase_preserves_objective_and_duplicate_cannot_extend_deadline():
    from explorer_core.observation_task_bridge import PendingObservationHandoff
    value=bridge(); value.publish_status=lambda status:None
    task=SimpleNamespace(task_id=101,task_version=1,goal=(3.,2.))
    pending=PendingObservationHandoff(value.active_observation,task,None,None,request_id=99)
    value.pending_observation_handoff=pending
    now=SimpleNamespace(nanoseconds=2_000_000_000)
    value.get_clock=lambda:SimpleNamespace(now=lambda:now)
    message=ObservationFeedback(task_id=101,task_version=1,request_id=99,
        state=ObservationFeedback.TRANSITION_WAIT_STOP)
    assert value.pending_observation_feedback(message)
    assert pending.published_ns==now.nanoseconds
    now.nanoseconds+=1_000_000_000
    assert value.pending_observation_feedback(message)
    assert pending.published_ns==2_000_000_000 and pending.state=='PREPARING'
    assert value.active_observation is pending.predecessor
    message.state=ObservationFeedback.TRANSITION_RESTARTING
    assert value.pending_observation_feedback(message)
    assert pending.published_ns==now.nanoseconds
    message.state=ObservationFeedback.TRANSITION_WAIT_STOP
    now.nanoseconds+=1_000_000_000
    value.pending_observation_feedback(message)
    assert pending.published_ns==3_000_000_000


def test_nonspatial_failed_goal_retry_and_handoff_exclusion_have_bounded_expiry():
    value=bridge()
    value.remember_local_motion_attempt((3.,2.),'HANDOFF_ROLLING_VELOCITY_ERROR')
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    assert not value.local_motion_retry_allowed(attempt)
    value.handoff_excluded_goals=[(3.,2.)]
    value.handoff_excluded_until={(3.,2.):6_000_000_000}
    assert value.current_handoff_exclusions()==((3.,2.),)
    value.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=6_000_000_000))
    assert value.local_motion_retry_allowed(attempt)
    assert value.current_handoff_exclusions()==()


def test_route_checkpoints_use_directed_edges_and_freeze_revision():
    from explorer_core.sparse_routing import SparseNode,SparseEdge,SparseRouteGraph,SparseRouteEstimate
    value=bridge(); value.task_geometry_autonomy=True
    graph=SparseRouteGraph()
    graph.upsert_node(SparseNode(1,(0.,0.)));graph.upsert_node(SparseNode(2,(3.,0.)))
    graph.upsert_edge(SparseEdge(11,1,2,3.,1.,((0.,0.),(3.,0.))))
    graph.graph_revision=5; value.sparse_router=graph
    value.active_observation.route_estimate=SparseRouteEstimate(3.,0.,1,2,
        ((0.,0.),(3.,0.)),5,((11,1,2),))
    value.publish_observation_task(path(),100)
    message=value.messages[-1]
    assert message.route_graph_revision==5 and len(message.route_portals)==1
    assert message.route_portals[0].connection_id==11
    assert message.route_portals[0].direction.x==1
    graph.graph_revision=6; graph.edges.clear()
    value.publish_observation_task(path(),101)
    assert value.messages[-1].route_portals==message.route_portals
    assert value.messages[-1].route_graph_revision==5


def test_stale_sparse_route_explicitly_falls_back_without_inventing_connections():
    from explorer_core.sparse_routing import SparseRouteEstimate
    value=bridge(); value.task_geometry_autonomy=True
    value.sparse_router=SimpleNamespace(graph_revision=6)
    value.active_observation.route_estimate=SparseRouteEstimate(3.,0.,1,2,
        ((0.,0.),(3.,0.)),5,((11,1,2),))
    value.publish_observation_task(path(),100)
    assert not value.messages[-1].route_portals and value.messages[-1].route_graph_revision==0


def test_reroute_keeps_task_and_denominator_and_station_budget():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_observation.station_adjustments=2
    value.active_observation.goal=(3.2,2.)
    value.publish_observation_task(path(),101)
    first,second=value.messages
    assert first.task_id==second.task_id==100
    assert second.task_version==first.task_version==1
    assert second.request_id!=first.request_id
    assert second.target_cell_count==8 and len(second.target_cells)==4
    assert value.active_observation.station_adjustments==2
    assert value.tasks_created==1


def test_rest_requires_fresh_continuous_odometry_and_no_rotation():
    value=bridge()
    del value.departure_is_settled
    now=value.get_clock().now()
    assert not value.departure_is_settled()
    for _ in range(6):
        value.record_departure_motion(0.02,0.03)
        now.nanoseconds += 100_000_000
    assert value.departure_is_settled()
    value.record_departure_motion(0.02,0.3)
    assert not value.departure_is_settled()
    value.record_departure_motion(0.09,0.01)
    assert not value.departure_is_settled()
    value.record_departure_motion(float("nan"),0)
    assert not value.departure_is_settled()
    now.nanoseconds += 1_000_000_000
    value.record_departure_motion(0,0)
    assert not value.departure_is_settled()


def test_transient_motion_and_context_changes_do_not_spend_recovery_budget():
    value=bridge()
    value.position=(2.,2.)
    value.active_raw_path=[(2,2),(3,2)]
    value.publish_observation_task(path(),100)
    value.publish_status=lambda status:None
    value.departure_is_settled=lambda:False
    value.reroute_active_goal=lambda **kw: (_ for _ in ()).throw(AssertionError("moving retry"))
    value.latch_blocked_execution("DEPARTURE_UNSAFE")
    for i in range(40):
        value.get_clock().now().nanoseconds += 100_000_000
        value.body_yaw=i*.2
        assert value.continue_blocked_execution()
    assert value.departure_recovery_attempts==0
    assert not value.departure_recovery_terminal


def test_new_objective_cancels_old_before_publication():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_observation=ObservationTask(8,(4.,2.),{(5,5)})
    value.publish_observation_task(path(),101)
    assert [message.lifecycle for message in value.messages]==[
        TaskMessage.ACTIVE,TaskMessage.SUPERSEDED,TaskMessage.ACTIVE]
    assert value.messages[1].task_id==100 and value.messages[2].task_id==101


def test_sensor_completed_handoff_keeps_old_execution_until_successor_ack():
    value=bridge()
    value.publish_observation_task(path(),100)
    old=value.active_observation
    value.completion_handoff_in_progress=True
    value.active_observation=ObservationTask(8,(4.,2.),{(5,5)})
    value.completion_handoff_in_progress=False
    value.publish_observation_task(path(),101)
    assert [m.lifecycle for m in value.messages]==[TaskMessage.ACTIVE,TaskMessage.ACTIVE]
    assert old.lifecycle=="COMPLETED"
    assert value.messages[-1].predecessor_task_id==100
    assert value.messages[-1].predecessor_task_version==1
    # Cancellation retains its immediate, explicit stop-first semantics.
    value.cancel_observation_task(value.active_observation)
    assert value.messages[-1].lifecycle==TaskMessage.CANCELLED
    assert value.messages[-1].task_id==101


def test_station_retry_sends_only_unresolved_boundaries_without_changing_denominator():
    value=bridge()
    value.publish_observation_task(path(),100)
    for x in range(7):
        value.grid.sensor_observed[5,x]=value.active_observation.initial_sensor_revision+1
    value.publish_observation_task(path(),101)
    retry=value.messages[-1]
    assert retry.task_id==100 and retry.task_version==1
    assert retry.target_cell_count==8
    assert len(retry.target_cells)==1
    assert retry.target_cells[0].x==value.grid.cell_to_world((7,5))[0]


def test_resolved_targets_are_not_reintroduced_while_completion_confirms():
    value=bridge()
    value.publish_observation_task(path(),100)
    for x in range(8):
        value.grid.sensor_observed[5,x]=value.active_observation.initial_sensor_revision+1
    value.publish_observation_task(path(),101)
    assert value.messages[-1].target_cell_count==8
    assert not value.messages[-1].target_cells


def test_station_capture_timeout_releases_motion_without_false_completion():
    value=bridge();value.task_geometry_autonomy=True
    value.publish_observation_task(path(),100)
    value.pending_observation_station=(value.grid.sensor_revision,1_000_000_000,None)
    calls=[];value.handoff_satisfied_observation=calls.append
    value.get_clock().now().nanoseconds+=2_000_000_000
    assert value.continue_observation_station()
    assert value.active_observation.motion_released
    assert not value.active_observation.information_satisfied
    assert value.observation_motion_released_count==1
    assert len(calls)==1


def test_station_capture_scheduling_matrix_never_authorizes_another_old_route():
    for elapsed_ns in (0,500_000_000,1_999_999_999,2_000_000_000):
        for scans in (0,1,2,3):
            value=bridge();value.task_geometry_autonomy=True
            value.publish_observation_task(path(),100)
            revision=value.grid.sensor_revision
            value.pending_observation_station=(revision,1_000_000_000,None)
            value.grid.sensor_revision+=scans
            value.get_clock().now().nanoseconds+=elapsed_ns
            calls=[];value.handoff_satisfied_observation=calls.append
            value.publish_reference_path=lambda *args: (_ for _ in ()).throw(
                AssertionError("station capture must not revive old motion"))
            assert value.continue_observation_station()
            released=scans>=3 or elapsed_ns>=2_000_000_000
            assert value.active_observation.motion_released==released
            assert len(calls)==int(released)
            assert not value.active_observation.information_satisfied


def test_motion_attempt_ignores_heading_jitter_and_unrelated_map_changes():
    value=bridge();value.grid.data.fill(0)
    value.remember_local_motion_attempt((3.,2.),"LOCAL_PATH_FAMILY_EXHAUSTED")
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.body_yaw=2.;value.map_update_count+=100
    value.grid.data[10,10]=100
    value.position=(2.2,2.)
    assert not value.local_motion_retry_allowed(attempt)
    value.position=(3.7,2.)
    assert value.local_motion_retry_allowed(attempt)


def test_only_departure_obstacle_failure_reopens_on_real_obstacle_removal():
    value=bridge();value.grid.data.fill(0);value.grid.data[2,2]=100
    value.remember_local_motion_attempt((3.,2.),"DEPARTURE_UNSAFE")
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.grid.data[2,2]=-1
    assert not value.local_motion_retry_allowed(attempt)
    value.grid.data[2,2]=0
    assert value.local_motion_retry_allowed(attempt)
    value.grid.data[2,2]=100
    value.remember_local_motion_attempt((3.,2.),"HANDOFF_MOVING_HEADING_ERROR")
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.grid.data[2,2]=0
    assert not value.local_motion_retry_allowed(attempt)


def test_departure_retry_requires_real_heading_change_not_jitter():
    value=bridge();value.grid.data.fill(0);value.body_yaw=3.10
    value.remember_local_motion_attempt((3.,2.),"DEPARTURE_UNSAFE")
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    for yaw in (3.11,-3.10,3.10):
        value.body_yaw=yaw
        assert not value.local_motion_retry_allowed(attempt)
    value.body_yaw=2.10
    assert value.local_motion_retry_allowed(attempt)
    value.body_yaw=float("nan")
    assert not value.local_motion_retry_allowed(attempt)


def test_stale_feedback_cannot_end_new_task_and_arrival_is_not_completion():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_path_request_generation=100
    message=ObservationFeedback(task_id=99,task_version=1,request_id=100,
        candidate_id=12,state=ObservationFeedback.STATION_REACHED,
        station=Point(x=2.,y=2.,z=.4),station_adjustments=1)
    value.observation_feedback_callback(message)
    assert value.task_stale_feedback==1 and value.task_stations_reached==0
    message.task_id=100
    value.observation_feedback_callback(message)
    value.observation_feedback_callback(message)
    assert value.task_stations_reached==1
    assert value.active_observation.completion_reason=="none"
    assert value.pending_observation_station[2].candidate_id==12


def test_rejection_uses_measured_failed_object_and_scope_arbitration():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_path_request_generation=100
    constraints=[]
    statuses=[]
    value.blacklist_radius=.4
    value.apply_global_viewpoint_constraint=lambda *args:constraints.append(args)
    value.planning_status_callback=lambda message:statuses.append(message.data)
    message=ObservationFeedback(task_id=100,task_version=1,request_id=100,
        candidate_id=12,state=ObservationFeedback.VIEWPOINT_REJECTED,
        station=Point(x=2.1,y=2.2,z=.4),reason="NO_SAFE_APPROACH")
    value.observation_feedback_callback(message)
    assert constraints[0][1]==(2.1,2.2)
    assert statuses==["VIEWPOINT_LOCAL_REJECTED request_id=100 task_id=100 task_version=1"]


def test_arrival_waits_for_observation_then_requests_same_task_station():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_path_request_generation=100
    value.observation_done_updates=3
    value.evaluate_active_observation=lambda:value.active_observation
    value.observation_complete=lambda task:False
    reroutes=[]
    value.reroute_active_goal=lambda:reroutes.append(True) or True
    value.observation_feedback_callback(ObservationFeedback(task_id=100,task_version=1,
        request_id=100,candidate_id=12,state=ObservationFeedback.STATION_REACHED,
        station=Point(x=2.,y=2.,z=.4),station_adjustments=1))
    assert value.continue_observation_station() and not reroutes
    # One accepted post-arrival scan permits continued station selection;
    # task completion still has its independent multi-update streak.
    value.map_update_count+=3
    assert value.continue_observation_station() and not reroutes
    value.grid.sensor_revision+=1
    assert value.continue_observation_station() and reroutes==[True]
    assert value.active_observation.task_id==100
    assert value.active_observation.station_adjustments==1


def test_sensor_completion_closes_task_without_another_station():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.pending_observation_station=(5,1_000_000_000,None)
    value.evaluate_active_observation=lambda:value.active_observation
    value.observation_complete=lambda task:True
    finished=[]
    value.handoff_satisfied_observation=lambda reason:finished.append(reason)
    assert value.continue_observation_station()
    assert finished==["OBSERVATION_TASK_COMPLETE_AT_STATION"]
    assert value.pending_observation_station is None


def test_completed_arrival_hands_off_in_feedback_without_waiting_timer():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_path_request_generation=100
    value.observation_complete=lambda task:True
    reasons=[]
    value.handoff_satisfied_observation=lambda reason:reasons.append(reason)
    value.observation_feedback_callback(ObservationFeedback(task_id=100,task_version=1,
        request_id=100,candidate_id=12,state=ObservationFeedback.STATION_REACHED,
        station=Point(x=2.,y=2.,z=.4),station_adjustments=1))
    assert reasons==["OBSERVATION_TASK_COMPLETE_AT_STATION"]
    assert value.pending_observation_station is None


def test_execution_block_is_not_spatial_failure_or_map_revision_retry():
    value=bridge()
    value.position=(2.,2.)
    value.body_yaw=0.
    value.grid.data.fill(0)
    value.publish_observation_task(path(),100)
    value.active_path_request_generation=100
    value.pending_path_request_generation=100
    value.active_raw_path=[(2,2)]
    statuses=[]; retries=[]
    value.publish_status=statuses.append
    value.publish_reference_path=lambda route:retries.append(route)
    value.apply_global_viewpoint_constraint=lambda *args: (_ for _ in ()).throw(
        AssertionError("execution failure must not constrain a viewpoint"))
    value.observation_feedback_callback(ObservationFeedback(task_id=100,task_version=1,
        request_id=100,state=ObservationFeedback.EXECUTION_BLOCKED,
        station=Point(x=2.,y=2.,z=.4),reason="DEPARTURE_UNSAFE"))
    task=value.active_observation
    assert value.pending_path_request_generation is None
    value.map_update_count+=10000
    value.grid.data[10,10]=100
    assert value.continue_blocked_execution() and not retries
    value.body_yaw=.6
    value.healthy_task_elapsed=999.
    assert value.continue_blocked_execution()
    assert not retries  # A yaw revision cannot bypass the settling wait.
    value.get_clock().now().nanoseconds += 2_000_000_000
    assert value.continue_blocked_execution()
    assert retries==[[(2,2)]]
    assert value.healthy_task_elapsed==0.
    assert value.active_observation is task


def test_local_occupancy_change_authorizes_execution_retry():
    value=bridge()
    value.position=(2.,2.)
    value.grid.data.fill(0)
    value.blocked_execution_context=value.local_execution_context()
    value.departure_recovery_started_ns=value.get_clock().now().nanoseconds
    value.active_raw_path=[(2,2)]
    retries=[]
    value.publish_reference_path=lambda route:retries.append(route)
    value.publish_status=lambda status:None
    x,y=value.grid.world_to_cell(*value.position)
    value.grid.data[y,x]=100
    assert value.continue_blocked_execution() and not retries
    value.get_clock().now().nanoseconds += 2_000_000_000
    assert value.continue_blocked_execution() and retries==[[(2,2)]]


def test_legacy_mode_ignores_task_feedback():
    value=bridge()
    value.task_mode=False
    value.observation_feedback_callback(ObservationFeedback())
    assert value.task_stale_feedback==0


def test_path_failure_reroutes_same_task_once_without_execution_wait():
    value = bridge()
    value.publish_observation_task(path(), 100)
    value.active_path_request_generation = 100
    value.active_raw_path = [(1, 1), (2, 1)]
    value.blocked_execution_context = ("old",)
    routes, statuses = [], []
    value.reroute_active_goal = lambda **kw: routes.append(kw) or True
    value.publish_status = statuses.append
    message = ObservationFeedback(task_id=100, task_version=1, request_id=100,
        candidate_id=12, state=ObservationFeedback.PATH_REJECTED,
        station=Point(x=2., y=2.), reason="LOCAL_PATH_FAMILY_EXHAUSTED")
    task = value.active_observation
    value.observation_feedback_callback(message)
    value.observation_feedback_callback(message)
    assert routes == [{"excluded_route": ((1, 1), (2, 1))}]
    assert statuses == ["LOCAL_PATH_REPAIRED"]
    assert value.active_observation is task
    assert value.blocked_execution_context is None


def test_exhausted_path_without_alternative_enters_sibling_arbitration():
    value = bridge()
    value.publish_observation_task(path(), 100)
    value.active_path_request_generation = 100
    value.active_raw_path = [(1, 1), (2, 1)]
    value.position = (1., 1.)
    value.task_path_constraints = {(100, 1): {((3, 1), (4, 1))}}
    value.reroute_active_goal = lambda **kw: False
    statuses = []
    value.planning_status_callback = lambda msg: statuses.append(msg.data)
    value.observation_feedback_callback(ObservationFeedback(task_id=100,
        task_version=1, request_id=100, candidate_id=12,
        state=ObservationFeedback.PATH_REJECTED, station=Point(x=2., y=2.)))
    assert statuses == ["VIEWPOINT_LOCAL_REJECTED request_id=100 task_id=100 task_version=1"]
    assert value.blocked_execution_context is None


def test_path_failure_without_spatial_evidence_keeps_viewpoint_and_task():
    value = bridge()
    value.publish_observation_task(path(), 100)
    value.active_path_request_generation = 100
    value.active_raw_path = [(1, 1), (2, 1)]
    value.position = (1., 1.)
    value.reroute_active_goal = lambda **kw: False
    value.planning_status_callback = lambda msg: (_ for _ in ()).throw(
        AssertionError("unlocated failure must not reject a viewpoint"))
    statuses = []
    value.publish_status = statuses.append
    value.task_path_constraints = {(100, 1): {((1, 1), (2, 1))}}
    task = value.active_observation
    value.observation_feedback_callback(ObservationFeedback(task_id=100,
        task_version=1, request_id=100, state=ObservationFeedback.PATH_REJECTED,
        station=Point(x=1., y=1.)))
    assert value.active_observation is task
    assert value.blocked_execution_context == value.local_execution_context()
    assert statuses == ["LOCAL_PATH_UNRESOLVED_NO_SPATIAL_EVIDENCE"]


def test_unchanged_departure_retries_are_bounded_across_request_refreshes():
    value = bridge()
    value.position = (2., 2.)
    value.grid.data.fill(0)
    value.active_raw_path = [(2, 2), (3, 2)]
    value.publish_observation_task(path(), 100)
    now = value.get_clock().now()
    statuses, retries, reroutes = [], [], []
    value.publish_status = statuses.append
    value.reroute_active_goal = lambda **kw: reroutes.append(kw) or False
    value.publish_reference_path = lambda route: retries.append(route)
    task = value.active_observation
    def block(request):
        value.active_path_request_generation = request
        value.observation_feedback_callback(ObservationFeedback(task_id=100,
            task_version=1, request_id=request, state=ObservationFeedback.EXECUTION_BLOCKED,
            station=Point(x=2., y=2.), reason="DEPARTURE_UNSAFE"))
    block(100)
    value.observation_feedback_callback(ObservationFeedback(task_id=100,
        task_version=1, request_id=100, state=ObservationFeedback.EXECUTION_BLOCKED,
        station=Point(x=2., y=2.), reason="DEPARTURE_UNSAFE"))
    assert value.continue_blocked_execution() and not retries
    for request in (101, 102):
        now.nanoseconds += 2_000_000_000
        assert value.continue_blocked_execution()
        value.publish_observation_task(path(), request)  # New request/ACK does not refill budget.
        block(request)
    assert len(retries) == len(reroutes) == value.departure_recovery_attempts == 2
    now.nanoseconds += 2_000_000_000
    assert value.continue_blocked_execution()
    assert value.departure_recovery_terminal
    assert value.messages[-1].lifecycle == TaskMessage.CANCELLED
    assert statuses[-1] == "DEPARTURE_RECOVERY_EXHAUSTED"
    assert value.recovery_handoffs == [
        "LOCAL_RECOVERY_EXHAUSTED request_id=102 task_id=100 task_version=1"]
    for _ in range(100):
        now.nanoseconds += 10_000_000_000
        value.body_yaw = now.nanoseconds
        assert value.continue_blocked_execution()
    assert len(retries) == 2 and value.task_cancellations == 1
    assert len(value.recovery_handoffs) == 1
    assert value.active_observation is task
    value.active_observation = ObservationTask(8, (4., 2.), {(5, 5)})
    assert value.departure_recovery_attempts == 0 and not value.departure_recovery_terminal


def test_invalid_contract_has_terminal_result_without_republishing():
    value = bridge()
    value.position = (2., 2.)
    value.publish_observation_task(path(), 100)
    statuses = []
    value.publish_status = statuses.append
    value.publish_reference_path = lambda route: (_ for _ in ()).throw(AssertionError("invalid contract retry"))
    value.latch_blocked_execution("INVALID_TASK_CONTRACT")
    value.get_clock().now().nanoseconds += 2_000_000_000
    assert value.continue_blocked_execution()
    assert value.departure_recovery_terminal and statuses == ["DEPARTURE_RECOVERY_EXHAUSTED"]


def test_alternating_path_rejections_cannot_bypass_task_retry_budget():
    value = bridge()
    value.position = (1., 1.)
    value.active_raw_path = [(1, 1), (2, 1)]
    value.publish_observation_task(path(), 100)
    reroutes = []
    value.reroute_active_goal = lambda **kw: reroutes.append(kw) or True
    value.publish_status = lambda status: None
    value.task_path_constraints = {}
    for request in (100, 101, 102):
        value.get_clock().now().nanoseconds += 2_000_000_000
        value.active_path_request_generation = request
        value.observation_feedback_callback(ObservationFeedback(task_id=100,
            task_version=1, request_id=request, state=ObservationFeedback.PATH_REJECTED,
            station=Point(x=1., y=1.)))
    assert len(reroutes) == value.departure_recovery_attempts == 2
    assert value.blocked_execution_context is not None
    value.get_clock().now().nanoseconds += 2_000_000_000
    value.continue_blocked_execution()
    assert value.departure_recovery_terminal
    value.observation_feedback_callback(ObservationFeedback(task_id=100,
        task_version=1, request_id=102, candidate_id=999,
        state=ObservationFeedback.STATION_REACHED, station=Point(x=1., y=1.)))
    assert value.pending_observation_station is None and value.task_stations_reached == 0


def test_time_and_task_change_do_not_reopen_unchanged_departure_failure():
    value=bridge();value.grid.data.fill(0);value.body_yaw=0.
    value.remember_local_motion_attempt((3.,2.),'LATERAL_RECOVERY_INEFFECTIVE')
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.get_clock().now().nanoseconds+=60_000_000_000
    value.active_observation=ObservationTask(8,(3.2,2.),{(5,5)})
    assert not value.local_motion_retry_allowed(attempt)
    value.body_yaw=1.
    assert value.local_motion_retry_allowed(attempt)


def test_turn_stage_successor_wait_is_finite_and_duplicate_does_not_extend_it():
    from explorer_core.observation_task_bridge import PendingObservationHandoff
    value=bridge();value.path_request_timeout=5.;value.publish_status=lambda _:None
    task=SimpleNamespace(task_id=101,task_version=1,goal=(3.,2.))
    pending=PendingObservationHandoff(value.active_observation,task,None,None,request_id=99)
    value.pending_observation_handoff=pending
    now=SimpleNamespace(nanoseconds=2_000_000_000)
    value.get_clock=lambda:SimpleNamespace(now=lambda:now)
    cancelled=[]
    def cancel(reason):
        cancelled.append(reason);pending.state='CANCELLING';pending.cancel_started_ns=now.nanoseconds
    value.cancel_pending_observation_handoff=cancel
    message=ObservationFeedback(task_id=101,task_version=1,request_id=99,
        state=ObservationFeedback.TRANSITION_WAIT_STOP,reason='WAITING_FOR_TURN_STAGE')
    assert value.pending_observation_feedback(message)
    now.nanoseconds+=8_000_000_000
    assert value.continue_pending_observation_handoff()
    assert pending.state=='PREPARING' and cancelled==[]
    assert value.pending_observation_feedback(message)
    assert pending.published_ns==2_000_000_000
    now.nanoseconds=17_000_000_000
    assert value.continue_pending_observation_handoff()
    assert cancelled==['EXECUTION_READY_TIMEOUT']


def test_occupied_recovery_start_waits_for_physical_change_without_goal_churn():
    value=bridge();value.task_geometry_autonomy=True;value.body_yaw=0.
    value.active_goal=(3.,2.);value.active_region_id=7
    value.publish_status=lambda status:None
    value.terminate_active_observation=lambda reason:None
    value.plan_from_current_position=lambda **kwargs: (_ for _ in ()).throw(AssertionError("occupied recovery cannot be repaired by changing remote Goal"))
    value.handle_local_motion_exhausted(ObservationFeedback(reason="RECOVERY_START_FOOTPRINT_OCCUPIED"))
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
    assert not value.local_departure_replan_allowed()
    assert not value.local_motion_retry_allowed(attempt)
    value.position=(2.31,2.)
    assert value.local_departure_replan_allowed()


def test_prepared_successor_action_failure_survives_cancel_without_freezing_all_goals():
    from explorer_core.observation_task_bridge import PendingObservationHandoff
    value=bridge();value.task_geometry_autonomy=True;value.body_yaw=0.
    value.publish_status=lambda status:None
    task=SimpleNamespace(task_id=101,task_version=1,goal=(3.5,2.))
    pending=PendingObservationHandoff(value.active_observation,task,[(2,2),(4,2)],None,request_id=99)
    value.pending_observation_handoff=pending
    message=ObservationFeedback(task_id=101,task_version=1,request_id=99,
        state=ObservationFeedback.LOCAL_MOTION_EXHAUSTED,reason="LATERAL_RECOVERY_REJOIN_FAILED")
    assert value.pending_observation_feedback(message) and pending.state=='CANCELLING'
    message.state=ObservationFeedback.CANCEL_ACK
    assert value.pending_observation_feedback(message)
    assert value.pending_observation_handoff is None and value.local_departure_replan_allowed()
    same=SimpleNamespace(goal=(9.,2.),path=[])
    assert value.local_departure_history_penalty(same)>0


def test_prepared_successor_occupied_departure_survives_cancel_ack():
    from explorer_core.observation_task_bridge import PendingObservationHandoff
    value=bridge();value.task_geometry_autonomy=True;value.body_yaw=0.
    value.publish_status=lambda status:None
    task=SimpleNamespace(task_id=101,task_version=1,goal=(3.5,2.))
    pending=PendingObservationHandoff(value.active_observation,task,[],None,request_id=99)
    value.pending_observation_handoff=pending
    message=ObservationFeedback(task_id=101,task_version=1,request_id=99,
        state=ObservationFeedback.LOCAL_MOTION_EXHAUSTED,reason="RECOVERY_START_FOOTPRINT_OCCUPIED")
    assert value.pending_observation_feedback(message)
    assert pending.state=='CANCELLING'
    assert not value.local_departure_replan_allowed()
    message.state=ObservationFeedback.CANCEL_ACK
    assert value.pending_observation_feedback(message)
    assert value.pending_observation_handoff is None
    assert not value.local_departure_replan_allowed()


def test_exhausted_goal_does_not_reopen_with_task_id_or_time_but_other_goal_is_allowed():
    value=bridge();value.body_yaw=0.;value.grid.data.fill(0)
    value.remember_local_motion_attempt((3.,2.),"DEPARTURE_EPISODE_EXHAUSTED")
    attempt=value.local_motion_goal_cooldowns[(3.,2.)]
    value.get_clock().now().nanoseconds+=100_000_000_000
    value.active_observation=ObservationTask(99,(4.,2.),{(5,5)})
    assert not value.local_motion_retry_allowed(attempt)
    assert value.local_departure_replan_allowed()
    value.position=(2.4,2.)
    assert value.local_motion_retry_allowed(attempt)


def test_installation_busy_and_budget_failures_are_temporary_and_not_directional():
    value=bridge();value.grid.data.fill(0);value.body_yaw=0.
    for reason in ("INSTALLATION_EVIDENCE_CHANGED", "LATERAL_RECOVERY_ALREADY_USED"):
        value.remember_local_motion_attempt((3.,2.),reason)
        attempt=value.local_motion_goal_cooldowns[(3.,2.)]
        assert not value.local_motion_retry_allowed(attempt)
        value.get_clock().now().nanoseconds+=2_000_000_000
        assert value.local_motion_retry_allowed(attempt)
        assert not getattr(value,"departure_action_failures",[])


def test_recovery_phase_wait_has_a_fixed_deadline_and_duplicate_does_not_extend_it():
    from explorer_core.observation_task_bridge import PendingObservationHandoff
    value=bridge();value.publish_status=lambda status:None
    value.path_request_timeout=5.
    task=SimpleNamespace(task_id=101,task_version=1,goal=(3.5,2.))
    pending=PendingObservationHandoff(value.active_observation,task,[],None,request_id=99)
    value.pending_observation_handoff=pending
    message=ObservationFeedback(task_id=101,task_version=1,request_id=99,
        state=ObservationFeedback.TRANSITION_RESTARTING,reason="DEPARTURE_RETRY_PENDING")
    assert value.pending_observation_feedback(message)
    started=pending.published_ns
    value.get_clock().now().nanoseconds+=6_000_000_000
    assert value.pending_observation_feedback(message)
    assert pending.published_ns==started
    assert value.continue_pending_observation_handoff() and pending.state=="PREPARING"
    cancelled=[];value.cancel_pending_observation_handoff=cancelled.append
    value.get_clock().now().nanoseconds=started+15_000_000_000
    value.continue_pending_observation_handoff()
    assert cancelled==["EXECUTION_READY_TIMEOUT"]


def test_initial_task_recovery_wait_extends_only_its_own_original_request_deadline():
    value=bridge();value.publish_observation_task(path(),100)
    value.active_path_request_generation=value.pending_path_request_generation=100
    value.pending_path_publish_ns=1_000_000_000;value.path_request_timeout=5.
    feedback=ObservationFeedback(task_id=100,task_version=1,request_id=100,
        state=ObservationFeedback.TRANSITION_RESTARTING,reason="DEPARTURE_RETRY_PENDING")
    value.observation_feedback_callback(feedback)
    assert value.motion_request_timeout()==15.
    value.get_clock().now().nanoseconds+=6_000_000_000
    value.observation_feedback_callback(feedback)
    assert value.pending_path_publish_ns==1_000_000_000
    assert value.motion_request_timeout()==15. # Duplicate phase cannot extend it.
    value.pending_path_request_generation=101
    assert value.motion_request_timeout()==5.
