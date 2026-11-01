"""Connection continuation, ownership and installed-work identity."""
from types import SimpleNamespace as NS
from explorer_core.grid import ExplorationGrid,FREE,UNKNOWN,OCCUPIED
from explorer_core.region_visits import RegionVisitWindow,BoundedVisitRoutes
from explorer_core.path_planning import RouteSearchStatus
from test_frontier_explorer import candidate_test_explorer,MODULE


def test_installed_demand_reattaches_after_cache_release_without_refreezing():
    w=RegionVisitWindow();g=ExplorationGrid(14,10,1,0,0);g.data.fill(FREE);g.data[2,10]=UNKNOWN
    d=w.freeze_demand(1,(8,2),{(10,2)})
    p=w.draft(NS(demand_prefix=(d.demand_id,),prefix=(d.frontier,),station_prefix=((6,2),)),1)
    w.install(p,(1,1,1));w.retain_regions([], 'OUTSIDE_VISIT_WINDOW')
    r=NS(region_id=1,clusters=[[(8,2)]],centroid=(8,2))
    w.reconcile([r],(0,0),2);w.sample_demands(r,[(8,2)],g,4,3)
    assert w.freeze_demand(1,(8,2),{(11,2)}) is d
    assert d.targets==frozenset({(10,2)})


def job_fixture():
    e=candidate_test_explorer();e.region_visit_window_enabled=True;e.selection_strategy='hierarchical'
    e.region_visit_route_expansions=3;e.region_sequence=[];e.publish_region_sequence=lambda:None
    e.publish_region_overlay=lambda *a:None;e.region_visit_frontiers=4;e.region_visit_stations=2
    e.grid.data[2:6,4]=OCCUPIED;e.grid.data[3,10]=UNKNOWN
    w=e.visit_window();d=w.freeze_demand(3,(8,2),{(10,3)})
    c=MODULE.FrontierCandidate(3,(8,2),(8,2),(8.5,2.5),[],7,1,1,0,{(10,3)},
        visit_demand_id=d.demand_id,visit_demand_ref=d,visit_visible_cells={(10,3)})
    r=MODULE.FrontierRegion(3,[[(8,2)]],(8,2),{(8,2)})
    key=e._visit_context((),None,True,True)
    e._pending_visit=(key,(1,2),[r],[c]);e.last_visit_window=[3]
    e.build_frontier_candidates=lambda *a,**kw:(_ for _ in ()).throw(AssertionError('must not enumerate'))
    return e,c


def resume(e):
    return e.resume_visit_connection((1,2),set(),(),required_scope=None,
        allow_region_release=True,allow_cross_region_preparation=True)


def test_pending_connection_advances_on_unchanged_input_without_enumeration():
    e,c=job_fixture();counts=[]
    for _ in range(100):
        handled,result=resume(e);assert handled
        if result is not None:break
        counts.append(sum(len(s.closed) for s in e._visit_routes.searches.values()))
    else:raise AssertionError('connection did not finish')
    assert result is c and result.path and e._pending_visit is None
    assert counts==sorted(counts) and len(counts)>1


def test_unrelated_map_update_keeps_the_same_connection_owner():
    e,c=job_fixture();assert resume(e)==(True,None)
    owner=next(iter(e._visit_routes.searches.values()))
    e.map_content_revision+=1;e.grid.data[7,11]=OCCUPIED
    assert resume(e)==(True,None)
    assert owner in e._visit_routes.searches.values()


def test_changed_contract_or_invalid_station_wakes_new_selection():
    e,c=job_fixture();resume(e);e.route_constraint_revision=7
    assert resume(e)==(False,None) and e._pending_visit is None
    e,c=job_fixture();e.grid.data[2,8]=OCCUPIED
    assert resume(e)==(False,None)


def test_station_observation_satisfied_during_wait_does_not_get_published():
    e,c=job_fixture();e.grid.data[3,10]=FREE
    assert resume(e)==(False,None) and not c.path


def test_preview_does_not_hold_the_current_connection_working_set():
    g=ExplorationGrid(20,20,1,0,0);g.data.fill(FREE);g.data[1:18,8]=OCCUPIED
    r=BoundedVisitRoutes();r.begin(g,set(),4096,revision=1)
    _,path=r.transition((2,2),(15,2))
    assert path is None and not r.searches


def test_other_candidates_cannot_evict_the_owner():
    g=ExplorationGrid(80,80,1,0,0);g.data.fill(FREE);g.data[1:75,40]=OCCUPIED
    r=BoundedVisitRoutes();r.begin(g,set(),10,revision=1)
    assert r.query((2,2),(70,2)).status==RouteSearchStatus.INCOMPLETE
    owner=next(iter(r.searches.values()))
    for target in [(70,3),(70,4),(70,5)]:
        assert r.query((2,2),target).status==RouteSearchStatus.INCOMPLETE
    assert list(r.searches.values())==[owner]
    r.retain_goals({(70,3)})
    assert not r.searches  # explicit problem change, not a silent eviction


def test_handoff_wait_has_no_second_full_planning_fallback():
    e=candidate_test_explorer();e.task_mode=True
    e._active_observation=MODULE.ObservationTask(3,(6.,2.),{(9,2)},information_satisfied=True)
    e.active_goal=(6.,2.);e.last_handoff_attempt_update=-1;e.map_update_count=1
    e.publish_status=lambda s:None
    def prepare(**kw):e._pending_visit=('connection',);return False
    e.activate_or_prepare_next_observation=prepare
    e.plan_from_current_position=lambda **kw:(_ for _ in ()).throw(AssertionError('fallback repeated'))
    assert not e._handoff_satisfied_observation('DONE')
    # An unchanged map must not suppress the next compute slice.
    e._pending_visit=('connection',)
    assert not e._handoff_satisfied_observation('DONE')


def test_start_motion_beyond_the_witness_anchor_wakes_replanning():
    e,c=job_fixture()
    handled,_=e.resume_visit_connection((3,2),set(),(),required_scope=None,
        allow_region_release=True,allow_cross_region_preparation=True)
    assert not handled and e._pending_visit is None


def test_preparation_entry_resumes_without_frontier_or_region_construction(monkeypatch):
    e,c=job_fixture();e.active_goal=(1.5,2.5)
    e._active_observation=MODULE.ObservationTask(3,e.active_goal,{(9,5)},information_satisfied=True)
    e.map_update_count=1;e.last_preparation_attempt_update=-1
    e.current_handoff_exclusions=lambda:();e.current_blocked_edges=lambda:set()
    e.publish_status=lambda status:None
    e._pending_visit=(e._visit_context((e.active_goal,),None,False,True),(1,2),
                      e._pending_visit[2],e._pending_visit[3])
    monkeypatch.setattr(MODULE,'cluster_frontiers',lambda *a:(_ for _ in ()).throw(AssertionError('full decision repeated')))
    for _ in range(100):
        if e.prepare_next_observation(force=False):break
    else:raise AssertionError('unchanged-map preparation did not finish')
    assert e.prepared_candidate is c


def test_promoted_region_connection_resumes_as_current_instead_of_starving():
    import time
    e,c=job_fixture();regions=e._pending_visit[2]
    previous=MODULE.FrontierRegion(1,[],(0,0),set())
    e._visit_pending_regions=set();e._visit_capacity_regions=set();e.region_visit_deferred=False
    e._visit_routes.begin(e.grid,set(),3,revision=1)
    assert e._select_visit_records(time.perf_counter(),(1,2),[previous]+regions,[c],(),
        required_scope=None,allow_region_release=True,allow_cross_region_preparation=True) is None
    assert e._pending_visit[2][0].region_id==3
    for _ in range(100):
        handled,result=resume(e);assert handled
        if result is not None:break
    else:raise AssertionError('promoted connection starved')
    assert result is c
