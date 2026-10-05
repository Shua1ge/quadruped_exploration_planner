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
    value.map_update_count=5
    value.messages=[]
    value.observation_task_pub=SimpleNamespace(publish=value.messages.append)
    now=SimpleNamespace(nanoseconds=1_000_000_000,to_msg=lambda:Time(sec=1))
    value.get_clock=lambda:SimpleNamespace(now=lambda:now)
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


def test_new_objective_cancels_old_before_publication():
    value=bridge()
    value.publish_observation_task(path(),100)
    value.active_observation=ObservationTask(8,(4.,2.),{(5,5)})
    value.publish_observation_task(path(),101)
    assert [message.lifecycle for message in value.messages]==[
        TaskMessage.ACTIVE,TaskMessage.SUPERSEDED,TaskMessage.ACTIVE]
    assert value.messages[1].task_id==100 and value.messages[2].task_id==101


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
    value.map_update_count+=3
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
    assert retries==[[(2,2)]]
    assert value.healthy_task_elapsed==0.
    assert value.active_observation is task


def test_local_occupancy_change_authorizes_execution_retry():
    value=bridge()
    value.position=(2.,2.)
    value.grid.data.fill(0)
    value.blocked_execution_context=value.local_execution_context()
    value.active_raw_path=[(2,2)]
    retries=[]
    value.publish_reference_path=lambda route:retries.append(route)
    value.publish_status=lambda status:None
    x,y=value.grid.world_to_cell(*value.position)
    value.grid.data[y,x]=100
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
    value.reroute_active_goal = lambda **kw: False
    statuses = []
    value.planning_status_callback = lambda msg: statuses.append(msg.data)
    value.observation_feedback_callback(ObservationFeedback(task_id=100,
        task_version=1, request_id=100, candidate_id=12,
        state=ObservationFeedback.PATH_REJECTED, station=Point(x=2., y=2.)))
    assert statuses == ["VIEWPOINT_LOCAL_REJECTED request_id=100 task_id=100 task_version=1"]
    assert value.blocked_execution_context is None
