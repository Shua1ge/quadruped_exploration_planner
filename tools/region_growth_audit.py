#!/usr/bin/env python3
"""Offline frontier-growth evidence audit. No ROS, publication, or runtime writes.

The baseline calls the real dyn partitioner/tracker. Suggestions are one-step
evidence comparisons against its PREVIOUS regions, never a second live registry.
"""
import argparse
from collections import deque
from pathlib import Path
import json
import math
import sys
import time

sys.dont_write_bytecode = True
import numpy as np

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
MOVES = ((-1, 0), (0, -1), (0, 1), (1, 0))
DEFAULTS = dict(region_size_m=8., region_match_distance_m=6.,
                min_frontier_size=6, max_path_ratio=1.5, max_detour_ratio=1.75,
                growth_budget_cells=20000, continuity_budget_cells=2000,
                continuity_max_m=12., work_span_m=12.)


def load_production(source_root):
    scripts = Path(source_root) / 'src/planner/plan_manage/scripts'
    if not (scripts/'explorer_core/frontier_regions.py').is_file():
        raise ValueError(f'Not a SCAN-Planner source root: {source_root}')
    sys.path.insert(0, str(scripts))
    from explorer_core.grid import ExplorationGrid
    from explorer_core import frontier_regions as fr
    from explorer_core import path_planning as pp
    return ExplorationGrid, fr, pp


def cell_set(values, width, height, name):
    result = set()
    for value in values:
        if (not isinstance(value, (list, tuple)) or len(value) != 2
                or any(isinstance(v, bool) or not isinstance(v, int) for v in value)):
            raise ValueError(f'{name}: expected integer [x,y] cells')
        x, y = value
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError(f'{name}: out-of-bounds cell {value}')
        result.add((x, y))
    return result


def around(cell):
    return [(cell[0]+dx, cell[1]+dy) for dx, dy in MOVES]


def tree_path(parents, source, target):
    first, node = [], source
    while node is not None:
        first.append(node)
        node = parents[node]
    positions = {p: i for i, p in enumerate(first)}
    second, node = [], target
    while node not in positions:
        second.append(node)
        node = parents[node]
    return first[:positions[node]+1] + list(reversed(second))


def growth_components(new_free, old_owners, current, budget):
    """Four-neighbour propagation ONLY inside explicitly measured U->F cells."""
    remaining = set(new_free)
    component_of, components = {}, []
    used = 0
    for seed in sorted(new_free):
        if seed not in remaining:
            continue
        parents, queue = {seed: None}, deque([seed])
        remaining.remove(seed)
        cid = len(components)
        sources = {}
        while queue:
            if used >= budget:
                return component_of, components, used, False
            cell = queue.popleft()
            used += 1
            component_of[cell] = cid
            for nb in around(cell):
                if current.value(nb) == FREE:
                    for owner in old_owners.get(nb, ()):
                        pair = (nb, cell)
                        sources[owner] = min(sources.get(owner, pair), pair)
                if nb in remaining:
                    remaining.remove(nb)
                    parents[nb] = cell
                    queue.append(nb)
        components.append(dict(parents=parents, sources=sources))
    return component_of, components, used, True


def short_continuity(grid, first, second, params, budget, fr, pp):
    # Cheap sampled endpoints support a positive connection only. A failed or
    # overlong sample does NOT prove no other short connection exists.
    a, b = min(((a, b) for a in fr.candidate_cells(first)
                for b in fr.candidate_cells(second)),
               key=lambda pair: (math.dist(*pair), pair))
    lower_bound = math.dist(a, b)*grid.resolution
    if lower_bound > params['continuity_max_m']:
        return dict(status='NOT_CHECKED_SPAN', sampled_distance_lower_bound_m=lower_bound)
    search = pp.KnownFreeSearch(a, b)
    result = search.advance(grid, set(), budget)
    if result.status == pp.RouteSearchStatus.INCOMPLETE:
        return dict(status='SEARCH_INCOMPLETE', expanded=len(search.closed))
    if result.status != pp.RouteSearchStatus.FOUND:
        return dict(status='NO_SAMPLED_ROUTE', expanded=len(search.closed))
    length = pp.path_length_cells(result.path, grid.resolution)
    return dict(status=('SHORT_CONNECTION_SUPPORTED' if length <= params['continuity_max_m']
                        else 'SAMPLED_ROUTE_TOO_LONG'), length_m=length,
                path=result.path, expanded=len(search.closed))


def compare_transition(previous, current, old_regions, clusters, baseline_regions,
                       measured, evidence_supplied, active_demand, params, fr, pp):
    old_frontiers = previous.frontier_cells(set()) if previous is not None else set()
    frontiers = current.frontier_cells(set())
    old_owners = {}
    for region in old_regions:
        for cell in region.cells:
            old_owners.setdefault(cell, set()).add(region.region_id)
    baseline_owner = {cell: r.region_id for r in baseline_regions for cell in r.cells}
    new_free, all_unknown_to_free = set(), set()
    changed = set()
    if previous is not None:
        ys, xs = np.nonzero(previous.data != current.data)
        changed = set(zip(xs.tolist(), ys.tolist()))
        all_unknown_to_free = {c for c in changed
            if previous.value(c) == UNKNOWN and current.value(c) == FREE}
        new_free = {c for c in measured
                    if previous.value(c) == UNKNOWN and current.value(c) == FREE}
    component_of, components, used, complete = growth_components(
        new_free, old_owners, current, params['growth_budget_cells'])
    rows = []
    for index, cluster in enumerate(clusters):
        cells = set(cluster)
        overlaps = {}
        for cell in sorted(cells):
            for owner in old_owners.get(cell, ()):
                overlaps.setdefault(owner, cell)
        entries = {}
        for cell in sorted(cells):
            for entry in [cell]+around(cell):
                if entry in component_of:
                    cid = component_of[entry]
                    if cid < len(components):
                        entries.setdefault(cid, (cell, entry))
        witnesses = {owner: dict(kind='OVERLAP', path=[cell])
                     for owner, cell in overlaps.items()}
        for cid, (frontier, entry) in sorted(entries.items()):
            component = components[cid]
            for owner, (old_cell, source) in component['sources'].items():
                if owner in witnesses:
                    continue
                path = [old_cell]+tree_path(component['parents'], source, entry)
                if path[-1] != frontier:
                    path.append(frontier)
                witnesses[owner] = dict(kind='MEASURED_FREE_GROWTH', path=path)
        owners = sorted(witnesses)
        if previous is None or not old_regions:
            status = 'NEW_WORK_CANDIDATE'
        elif not complete:
            status = 'DEFERRED_GROWTH_BUDGET'
        elif len(owners) > 1:
            status = 'AMBIGUOUS_MULTIPLE_PARENTS'
        elif len(owners) == 1:
            status = 'CONTINUITY_SUPPORTED'
        else:
            status = 'NO_PARENT_EVIDENCE'
        rows.append(dict(cluster=index, frontier_cells=sorted(cells),
            baseline_region_ids=sorted({baseline_owner[c] for c in cells}),
            candidate_parent_ids=owners, status=status, witnesses=witnesses,
            active_demand_adjacent=bool(cells.intersection(active_demand)
                or any(nb in active_demand for c in cells for nb in around(c)))))
    # Distinct frontiers with the same parent need a connection witness before
    # claiming that they are one continuously visitable work set.
    budget = pp.ExpansionBudget(params['continuity_budget_cells'])
    checks, checked_pairs = [], {}
    for owner in sorted(old_owners_id(old_regions)):
        children = [r for r in rows if r['status'] == 'CONTINUITY_SUPPORTED'
                    and r['candidate_parent_ids'] == [owner]]
        if len(children) < 2:
            continue
        linked = {children[0]['cluster']}
        pending = children[1:]
        while pending:
            progressed = False
            for row in list(pending):
                for anchor in sorted(linked):
                    key = tuple(sorted((anchor, row['cluster'])))
                    if key not in checked_pairs:
                        checked_pairs[key] = short_continuity(current, clusters[anchor],
                            clusters[row['cluster']], params, budget, fr, pp)
                        checks.append(dict(parent_region_id=owner, first=anchor,
                                           second=row['cluster'], **checked_pairs[key]))
                    result = checked_pairs[key]
                    if result['status'] == 'SHORT_CONNECTION_SUPPORTED':
                        linked.add(row['cluster'])
                        pending.remove(row)
                        progressed = True
                        break
            if not progressed:
                break
        if pending:
            reason = ('DEFERRED_CONTINUITY_BUDGET' if any(
                c['parent_region_id'] == owner and c['status'] == 'SEARCH_INCOMPLETE'
                for c in checks) else 'REVIEW_MULTIPLE_CONTINUATIONS')
            for row in children:
                row['status'] = reason
    for row in rows:
        cells = row['frontier_cells']
        representatives = fr.candidate_cells(cells)
        span = max(math.dist(a, b) for a in representatives
                   for b in representatives)*current.resolution
        row['euclidean_span_lower_bound_m'] = span
        row['workload_review'] = span > params['work_span_m']
        row['suggested_parent_id'] = (row['candidate_parent_ids'][0]
            if row['status'] == 'CONTINUITY_SUPPORTED' else None)
        row['baseline_identity_disagreement'] = (row['suggested_parent_id'] is not None
            and row['baseline_region_ids'] != [row['suggested_parent_id']])
    return dict(delta=dict(changed_cells=len(changed),
        cell_transitions=[dict(cell=c, before=previous.value(c), after=current.value(c),
                               declared_measured_free=c in measured) for c in sorted(changed)],
        appeared_frontier_cells=sorted(frontiers-old_frontiers),
        disappeared_frontier_cells=sorted(old_frontiers-frontiers),
        measured_unknown_to_free_cells=sorted(new_free),
        unmeasured_unknown_to_free_cells=sorted(all_unknown_to_free-new_free),
        evidence_supplied=evidence_supplied), growth_budget_used=used,
        growth_complete=complete, continuity_budget_used=budget.used,
        continuity_checks=checks, clusters=rows,
        active_child_candidates=[r['cluster'] for r in rows if r['active_demand_adjacent']],
        note='No split/merge, task mutation, completion decision, or motion certification')


def old_owners_id(regions):
    return {r.region_id for r in regions}


def run_case(case, source_root):
    Grid, fr, pp = load_production(source_root)
    if case.get('schema_version', 1) != 1:
        raise ValueError('Only schema_version=1 is supported')
    if not isinstance(case.get('frames'), list) or not case['frames']:
        raise ValueError('frames must be a nonempty snapshot list')
    config = case['grid']
    width, height = int(config['width']), int(config['height'])
    resolution = float(config['resolution'])
    origin = config.get('origin', [0., 0.])
    if (width <= 0 or height <= 0 or resolution <= 0 or not math.isfinite(resolution)
            or len(origin) != 2 or not all(math.isfinite(float(v)) for v in origin)):
        raise ValueError('Invalid grid geometry')
    params = {**DEFAULTS, **case.get('parameters', {})}
    if params.keys() != DEFAULTS.keys():
        raise ValueError('Unknown parameter name')
    for key in ('min_frontier_size', 'growth_budget_cells', 'continuity_budget_cells'):
        if isinstance(params[key], bool) or not isinstance(params[key], int) or params[key] < 0:
            raise ValueError(f'{key} must be a nonnegative integer')
    for key in set(params)-{'min_frontier_size', 'growth_budget_cells', 'continuity_budget_cells'}:
        if not math.isfinite(params[key]) or params[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if params['min_frontier_size'] < 1:
        raise ValueError('min_frontier_size must be >=1')
    tracker = fr.PersistentRegionTracker(params['region_match_distance_m']/resolution)
    previous, old_regions, frames = None, [], []
    last_revision, last_epoch = None, None
    for record in case['frames']:
        revision = record['revision']
        epoch = record.get('epoch', 0)
        if not isinstance(revision, int) or isinstance(revision, bool):
            raise ValueError('revision must be an integer')
        reset = last_epoch is not None and epoch != last_epoch
        if reset:
            tracker = fr.PersistentRegionTracker(params['region_match_distance_m']/resolution)
            previous, old_regions, last_revision = None, [], None
        if last_revision is not None and revision <= last_revision:
            raise ValueError('Snapshot revision must increase within one epoch')
        raw = np.asarray(record['occupancy'])
        if raw.shape not in ((height, width), (height*width,)):
            raise ValueError('occupancy shape does not match grid')
        if not np.isin(raw, [UNKNOWN, FREE, OCCUPIED]).all():
            raise ValueError('occupancy values must be -1, 0, or 100')
        # ExplorationGrid rounds metres with ceil; avoid 24*.2/.2 becoming
        # 24.000000000000004 when the snapshot already specifies exact cells.
        grid = Grid(math.nextafter(width*resolution, 0.),
                    math.nextafter(height*resolution, 0.), resolution, *origin)
        if (grid.width, grid.height) != (width, height):
            raise ValueError('Grid metre/cell conversion disagrees with snapshot shape')
        grid.data[:] = raw.reshape(height, width).astype(np.int8)
        measured = cell_set(record.get('measured_free_cells', []), width, height, 'measured_free_cells')
        if any(grid.value(c) != FREE for c in measured):
            raise ValueError('measured_free_cells must currently be FREE')
        active = cell_set(record.get('active_demand_cells', []), width, height, 'active_demand_cells')
        t = time.perf_counter()
        clusters = fr.cluster_frontiers(grid.frontier_cells(set()), params['min_frontier_size'])
        observations = fr.partition_frontier_clusters_by_topology(clusters, [None]*len(clusters),
            max(1, round(params['region_size_m']/resolution)), grid, set(),
            params['max_path_ratio'], params['max_detour_ratio'])
        baseline = tracker.update(observations, record.get('active_region_id'))
        baseline_ms = (time.perf_counter()-t)*1000
        t = time.perf_counter()
        comparison = compare_transition(previous, grid, old_regions, clusters, baseline,
            measured, 'measured_free_cells' in record, active, params, fr, pp)
        suggestion_ms = (time.perf_counter()-t)*1000
        frames.append(dict(revision=revision, epoch=epoch, epoch_reset=reset,
            capture_stamp_ns=record.get('capture_stamp_ns'),
            evidence_source=record.get('evidence_source', 'input-declared mask; not independently verified'),
            baseline_regions=[dict(id=r.region_id, lineage_id=r.lineage_id,
                cells=sorted(r.cells)) for r in baseline], **comparison,
            timing=dict(baseline_partition_tracker_ms=baseline_ms,
                growth_audit_ms=suggestion_ms,
                comparable_timings=False)))
        previous, old_regions = grid, baseline
        last_revision, last_epoch = revision, epoch
    return dict(name=case.get('name', 'input'), grid=dict(width=width,height=height,
                resolution=resolution,origin=origin), parameters=params, frames=frames,
                scope='One-step evidence audit against actual tracker history; suggestions never feed baseline')


def demo_cases():
    def case(name, arrays, **params):
        frames = []
        for i, a in enumerate(arrays):
            measured = np.argwhere(a == FREE) if i == 0 else np.argwhere(
                (arrays[i-1] == UNKNOWN) & (a == FREE))
            frames.append(dict(revision=i+1, occupancy=a.tolist(),
                               measured_free_cells=[[int(x), int(y)] for y, x in measured]))
        return dict(name=name, grid=dict(width=arrays[0].shape[1],height=arrays[0].shape[0],
            resolution=.2), parameters=dict(min_frontier_size=2, **params), frames=frames)
    before = np.full((24, 75), UNKNOWN, dtype=np.int8)
    before[[0, -1], :] = OCCUPIED
    before[:, 0] = OCCUPIED
    before[1:-1, 1:15] = FREE
    after = before.copy(); after[1:-1, 15:55] = FREE
    result = [case('open_advance_8m', [before, after]),
              case('cold_start', [before])]
    pillar = before.copy(); pillar[10:14, 12:18] = OCCUPIED
    result.append(case('pillar_splits_frontier', [before, pillar]))
    t0 = np.full((150, 45), OCCUPIED, dtype=np.int8)
    t0[73:77, 2:35] = UNKNOWN; t0[4:146, 31:35] = UNKNOWN
    t0[73:77, 2:22] = FREE
    t1 = t0.copy();t1[73:77, 2:35] = FREE;t1[20:130, 31:35] = FREE
    t = case('t_junction_multiple_continuations', [t0, t1])
    t['frames'][-1]['active_demand_cells'] = [[32,19]]
    result.append(t)
    m0 = np.full((30, 50), OCCUPIED, dtype=np.int8)
    m0[2:26, 2:42] = UNKNOWN
    m0[2:10, 2:12] = FREE;m0[18:26, 2:12] = FREE
    m1 = m0.copy();m1[2:26, 2:30] = FREE
    result.append(case('two_parents_meet', [m0, m1]))
    seed = case('unmeasured_free_is_not_growth', [before, after])
    seed['frames'][-1]['measured_free_cells'] = []
    result.append(seed)
    result.append(case('growth_budget_exhausted', [before, after],growth_budget_cells=20))
    result.append(case('continuity_budget_exhausted', [before, pillar],continuity_budget_cells=0))
    return result


def markdown(report):
    lines = ['# Frontier 延续只读对照', '',
        '当前分组与 tracker 直接调用 dyn 源码；新规则只输出证据建议，不生成或取消任务。', '',
        '| 案例 | 最后帧当前 Region ID | 新规则状态 | 建议父级 | 身份分歧簇数 |',
        '|---|---|---|---|---:|']
    for case in report['cases']:
        frame = case['frames'][-1]
        rows = frame['clusters']
        lines.append(f"| {case['name']} | {[r['id'] for r in frame['baseline_regions']]} | "
            f"{sorted({r['status'] for r in rows})} | "
            f"{[r['suggested_parent_id'] for r in rows]} | "
            f"{sum(r['baseline_identity_disagreement'] for r in rows)} |")
    lines += ['', '边界：CONTINUITY_SUPPORTED 是工作延续候选，不是合并授权、真实分支证明或四足通行认证。',
        'REVIEW_MULTIPLE_CONTINUATIONS 不自动分裂；NEW_WORK_CANDIDATE 只建议创建工作包，不分配正式ID。',
        'JSON保留逐簇来源、实测新增FREE、路径见证、预算和当前tracker归属。',
        '输入必须显式提供实测FREE来源；不能从OccupancyGrid变化或单帧LocalMapPatch推断它。',
        'baseline与audit计时覆盖不同工作，不能用二者相除宣称新规划器提速。',
        '多帧回放的建议不反灌当前tracker，不构成完整新Registry生命周期仿真。', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    select = parser.add_mutually_exclusive_group(required=True)
    select.add_argument('--demo', action='store_true')
    select.add_argument('--input', type=Path, help='JSON snapshots, not a runtime topic')
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, help='Write result JSON and same-name .md report')
    parser.add_argument('--write-demo-inputs', type=Path, help='Export reproducible demo snapshot files')
    args = parser.parse_args(argv)
    if args.write_demo_inputs and not args.demo:
        parser.error('--write-demo-inputs requires --demo')
    if args.output and args.input and (args.output.resolve() == args.input.resolve()
            or args.output.with_suffix('.md').resolve() == args.input.resolve()):
        parser.error('Output must not overwrite input')
    try:
        cases = demo_cases() if args.demo else [json.loads(args.input.read_text())]
        report = dict(schema_version=1, read_only_runtime=True,
            source_root=str(args.source_root.resolve()), cases=[run_case(c,args.source_root) for c in cases])
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            args.output.with_suffix('.md').write_text(markdown(report))
        if args.write_demo_inputs:
            args.write_demo_inputs.mkdir(parents=True, exist_ok=True)
            for case in cases:
                (args.write_demo_inputs/(case['name']+'.json')).write_text(json.dumps(case)+'\n')
        print(markdown(report) if args.output else json.dumps(report,ensure_ascii=False,indent=2))
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(2, f'Input/source error: {error}\n')


if __name__ == '__main__':
    main()
