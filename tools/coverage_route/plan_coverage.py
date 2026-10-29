#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plan a near-optimal MID360 observation route over the Campus walkable graph.

    python3 tools/coverage_route/plan_coverage.py [--spacing 1.5] [--az 360] [--el 32] [--workers 8]

Pipeline
  1. candidate poses: every `spacing` metres on the walkable component
  2. per-pose visibility: MID360 rays vs mesh, keep hits on walkable cells
  3. achievable coverage = union over all candidates (occlusion-aware upper bound)
  4. greedy set cover (lazy) down to `--target` of achievable coverage
  5. open TSP over selected poses on geodesic distances (nearest neighbour + 2-opt)
  6. reconstruct the path on the 0.3 m walkable graph, sample it every 1 m,
     recompute continuous coverage, report per-cluster misses

Outputs (docs/dyn/coverage/route/):
  plan.json           metrics, selected poses, uncovered clusters
  route.csv           x,y,z per path point
  route_preview.png   walkable grey, covered green, missed red, route blue
"""

import argparse
import heapq
import json
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage/route')
sys.path.insert(0, HERE)
import mid360_visibility as mv  # noqa: E402

W = None   # worker globals
_GZ = _WALK = _RES = _ORIGIN = _RC = None


def _init_worker(gz, walk, res, origin):
    global _GZ, _WALK, _RES, _ORIGIN, _RC
    _GZ, _WALK, _RES, _ORIGIN = gz, walk, res, origin
    _RC = mv.CampusRaycaster()


def _visible(task):
    idx, az, el = task
    h, w = _WALK.shape
    iy, ix = divmod(idx, w)
    x = _ORIGIN[0] + (ix + 0.5) * _RES
    y = _ORIGIN[1] + (iy + 0.5) * _RES
    ids, _ = _RC.visible_ids((x, y), _GZ, _WALK, _RES, _ORIGIN,
                             az_n=az, el_n=el)
    return idx, ids


def build_graph(walk, gz, res):
    h, w = walk.shape
    iy, ix = np.divmod(np.arange(walk.size), w)
    rows, cols, weights = [], [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        y2, x2 = iy + dy, ix + dx
        valid = (y2 >= 0) & (y2 < h) & (x2 >= 0) & (x2 < w)
        a = np.nonzero(valid & walk.reshape(-1))[0]
        b = y2[a] * w + x2[a]
        dz = np.abs(gz.reshape(-1)[a] - gz.reshape(-1)[b])
        dist = res * (1.41421356237 if dx else 1.0)
        ok = walk.reshape(-1)[b] & np.isfinite(dz) & \
            (dz <= max(0.20, np.tan(np.radians(30)) * dist))
        rows.append(a[ok])
        cols.append(b[ok])
        weights.append(np.full(int(ok.sum()), dist))
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    g = coo_matrix((np.concatenate(weights), (rows, cols)),
                   shape=(walk.size, walk.size))
    return g.tocsr()


def coarsen(walk, gz, res, target_res):
    """Aggregate the fine mask/z to a routing grid (majority walkable, median z)."""
    s = max(1, int(round(target_res / res)))
    h, w = walk.shape
    H, W = (h + s - 1) // s, (w + s - 1) // s
    wp = np.zeros((H * s, W * s), dtype=bool)
    zp = np.full((H * s, W * s), np.nan, dtype=np.float64)
    wp[:h, :w] = walk
    zp[:h, :w] = np.where(walk, gz, np.nan)
    frac = wp.reshape(H, s, W, s).mean(axis=(1, 3))
    with np.errstate(all='ignore'):
        zc = np.nanmedian(zp.reshape(H, s, W, s), axis=(1, 3))
    cw = frac >= 0.5
    cz = np.where(cw, zc, np.nan)
    return cw, cz, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spacing', type=float, default=1.5)
    ap.add_argument('--az', type=int, default=360)
    ap.add_argument('--el', type=int, default=32)
    ap.add_argument('--target', type=float, default=0.99,
                    help='fraction of achievable coverage')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--max-candidates', type=int, default=0)
    ap.add_argument('--route-res', type=float, default=0.9)
    args = ap.parse_args()

    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    n = walk.size
    walk_flat = walk.reshape(-1)
    t0 = time.time()

    # candidate poses on a regular grid (nearest walkable cell)
    step = max(1, int(round(args.spacing / res)))
    cand = []
    for iy in range(step // 2, h, step):
        for ix in range(step // 2, wd, step):
            if walk[iy, ix]:
                cand.append(iy * wd + ix)
    cand = np.array(cand, dtype=np.int64)
    if args.max_candidates and len(cand) > args.max_candidates:
        cand = cand[np.linspace(0, len(cand) - 1, args.max_candidates).astype(int)]
    print(f'candidates={len(cand)} spacing={args.spacing} m  ({time.time()-t0:.1f}s)')

    tasks = [(int(i), args.az, args.el) for i in cand]
    t1 = time.time()
    with Pool(args.workers, initializer=_init_worker,
              initargs=(gz, walk, res, origin)) as pool:
        res_list = pool.map(_visible, tasks, chunksize=8)
    vis = {idx: ids for idx, ids in res_list}
    print(f'visibility computed in {time.time()-t1:.1f}s, '
          f'avg cells/pose={np.mean([len(v) for v in vis.values()]):.1f}')

    avail = np.zeros(n, dtype=bool)
    for ids in vis.values():
        avail[ids] = True
    n_target = int(walk.sum())
    n_avail = int((avail & walk_flat).sum())
    print(f'achievable coverage={n_avail}/{n_target} walkable cells '
          f'({100*n_avail/n_target:.1f}%, {n_avail*res*res:.0f} m2)')
    goal = int(math.ceil(n_avail * args.target))

    # lazy greedy set cover
    covered = np.zeros(n, dtype=bool)
    heap = [(-len(ids), int(idx), 0) for idx, ids in vis.items()]
    heapq.heapify(heap)
    selected, stale = [], 0
    while len(heap) and covered.sum() < goal:
        neg, idx, ver = heapq.heappop(heap)
        ids = vis[idx]
        gain = int((~covered[ids]).sum())
        if ver and gain < -neg:
            heapq.heappush(heap, (-gain, idx, ver + 1))
            stale += 1
            continue
        if gain == 0:
            break
        selected.append(idx)
        covered[ids] = True
    cov = int((covered & walk_flat).sum())
    print(f'selected={len(selected)} (stale pops {stale}), '
          f'coverage={cov}/{n_target} ({100*cov/n_target:.1f}%)')

    # uncovered clusters among achievable cells
    missed = (walk_flat & avail & ~covered).reshape(h, wd)
    lab, nlab = ndimage.label(missed)
    clusters = []
    for k in range(1, nlab + 1):
        m = lab == k
        if m.sum() * res * res >= 1.0:
            ys, xs = np.where(m)
            clusters.append({'cells': int(m.sum()),
                             'area_m2': round(float(m.sum() * res * res), 1),
                             'center': [round(float(origin[0] + xs.mean() * res), 1),
                                        round(float(origin[1] + ys.mean() * res), 1)]})
    clusters.sort(key=lambda c: -c['area_m2'])
    print(f'uncovered clusters >=1 m2: {len(clusters)}, '
          f'total {sum(c["area_m2"] for c in clusters):.0f} m2')

    # routing on a coarser grid (1.3M fine nodes make per-pair Dijkstra too slow)
    cwalk, cgz, s = coarsen(walk, gz, res, args.route_res)
    ch, cw = cwalk.shape
    cell = s * res
    print(f'routing grid={ch}x{cw} @ {cell:.2f} m')

    def coarse_of(fine_id):
        fy, fx = divmod(int(fine_id), wd)
        cy, cx = fy // s, fx // s
        if cwalk[cy, cx]:
            return cy * cw + cx
        best, bd = None, 1e9
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                y2, x2 = cy + dy, cx + dx
                if 0 <= y2 < ch and 0 <= x2 < cw and cwalk[y2, x2]:
                    d = dy * dy + dx * dx
                    if d < bd:
                        bd, best = d, y2 * cw + x2
        return best

    spawn_cell = coarse_of(int(np.clip((0 - origin[1]) / res, 0, h - 1)) * wd +
                           int(np.clip((0 - origin[0]) / res, 0, wd - 1)))
    if spawn_cell is None:
        fy, fx = np.divmod(np.nonzero(cwalk.reshape(-1))[0], cw)
        d = np.hypot(fy - (0 - origin[1]) / cell, fx - (0 - origin[0]) / cell)
        spawn_cell = int(np.argmin(d))
    sel_nodes = []
    for fid in selected:
        cid = coarse_of(fid)
        if cid is not None and cid not in sel_nodes:
            sel_nodes.append(cid)
    nodes = [spawn_cell] + sel_nodes
    graph = build_graph(cwalk, cgz, cell)
    D = np.zeros((len(nodes), len(nodes)), dtype=np.float32)
    t2 = time.time()
    for i, src in enumerate(nodes):
        dist = dijkstra(graph, directed=False, indices=src)
        D[i] = dist[nodes]
        if i % 25 == 0:
            print(f'  geodesics {i}/{len(nodes)} ({time.time()-t2:.0f}s)', flush=True)
    if not np.isfinite(D).all():
        print('WARNING: coarse routing graph has unreachable selected nodes')
    print(f'pairwise geodesics {len(nodes)}x{len(nodes)} in {time.time()-t2:.1f}s; '
          f'max {np.nanmax(D[np.isfinite(D)]):.0f} m')

    # open TSP: nearest neighbour + 2-opt (start = spawn, index 0)
    m = len(nodes)
    order = [0]
    left = set(range(1, m))
    while left:
        cur = order[-1]
        nxt = min(left, key=lambda j: D[cur, j])
        order.append(nxt)
        left.remove(nxt)
    def plen(o):
        return float(sum(D[o[i], o[i + 1]] for i in range(len(o) - 1)))
    improved = True
    passes = 0
    while improved and passes < 200:
        passes += 1
        improved = False
        for i in range(1, m - 1):
            for j in range(i + 1, m):
                a, b = order[i - 1], order[i]
                c = order[j]
                if j + 1 < m:
                    delta = D[a, c] + D[b, order[j + 1]] - D[a, b] - D[c, order[j + 1]]
                else:
                    delta = D[a, c] - D[a, b]
                if delta < -1e-6:
                    order[i:j + 1] = order[i:j + 1][::-1]
                    improved = True
                    break
            if improved:
                break
    best = plen(order)
    print(f'TSP nodes={m} length={best:.0f} m (2-opt, {passes} passes)', flush=True)

    # reconstruct geometry on the routing grid
    path_xy, path_z = [], []
    for a, b in zip(order, order[1:]):
        src, dst = nodes[a], nodes[b]
        dist, pred = dijkstra(graph, directed=False, indices=src,
                              return_predecessors=True)
        if not np.isfinite(dist[dst]):
            raise RuntimeError('unreachable TSP edge')
        seq = []
        cur = dst
        while cur != src and cur >= 0:
            seq.append(cur)
            cur = pred[cur]
        seq.append(src)
        seq.reverse()
        ys, xs = np.divmod(seq, cw)
        path_xy.extend(np.stack([origin[0] + (xs + 0.5) * cell,
                                 origin[1] + (ys + 0.5) * cell], axis=1))
        path_z.extend(cgz.reshape(-1)[seq])
    path_xy = np.asarray(path_xy)
    path_z = np.asarray(path_z)
    print(f'geometry reconstructed: {len(path_xy)} points', flush=True)
    seg = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    plen_geom = float(seg.sum())
    print(f'geometry points={len(path_xy)}, length={plen_geom:.0f} m')

    # continuous coverage: sample every 1 m along the path
    acc = np.concatenate([[0], np.cumsum(seg)])
    samples = []
    for s in np.arange(0, plen_geom, 1.0):
        k = int(np.searchsorted(acc, s, side='right') - 1)
        k = min(max(k, 0), len(path_xy) - 1)
        samples.append((float(path_xy[k, 0]), float(path_xy[k, 1])))
    # dedupe to 0.5 m
    ded = []
    for p in samples:
        if not ded or math.hypot(p[0] - ded[-1][0], p[1] - ded[-1][1]) >= 0.5:
            ded.append(p)
    print(f'route samples for coverage: {len(ded)}', flush=True)

    def sample_pose(p):
        ix = int(np.clip((p[0] - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((p[1] - origin[1]) / res, 0, h - 1))
        return iy * wd + ix
    tasks2 = [(sample_pose(p), args.az, args.el) for p in ded]
    with Pool(args.workers, initializer=_init_worker,
              initargs=(gz, walk, res, origin)) as pool:
        res2 = pool.map(_visible, tasks2, chunksize=16)
    cov2 = np.zeros(n, dtype=bool)
    for _, ids in res2:
        cov2[ids] = True
    cov2 &= walk_flat
    print(f'continuous coverage={cov2.sum()}/{n_target} '
          f'({100*cov2.sum()/n_target:.1f}%, {cov2.sum()*res*res:.0f} m2), '
          f'achievable-limited {100*n_avail/n_target:.1f}%')

    os.makedirs(OUT, exist_ok=True)
    np.savetxt(os.path.join(OUT, 'route.csv'),
               np.column_stack([path_xy, path_z]), delimiter=',',
               header='x,y,z', comments='')
    plan = {
        'spacing_m': args.spacing, 'rays': [args.az, args.el],
        'candidates': int(len(cand)), 'selected': len(selected),
        'walkable_area_m2': round(float(n_target * res * res), 1),
        'achievable_coverage': round(100 * n_avail / n_target, 2),
        'selected_coverage': round(100 * cov / n_target, 2),
        'continuous_coverage': round(100 * cov2.sum() / n_target, 2),
        'path_length_geom_m': round(plen_geom, 1),
        'tsp_length_m': round(best, 1),
        'uncovered_clusters': clusters[:20],
        'uncovered_cluster_count': len(clusters),
        'spawn': [0, 0],
        'selected_xy': [[round(float(origin[0] + (nodes[i] % wd + 0.5) * res), 2),
                         round(float(origin[1] + (nodes[i] // wd + 0.5) * res), 2)]
                        for i in order],
    }
    with open(os.path.join(OUT, 'plan.json'), 'w') as f:
        json.dump(plan, f, indent=1, ensure_ascii=False)

    from PIL import Image
    img = np.zeros((h, wd, 3), dtype=np.uint8)
    img[walk] = (80, 80, 80)
    img[walk & cov2.reshape(walk.shape)] = (40, 160, 60)
    img[walk & avail.reshape(walk.shape) & ~cov2.reshape(walk.shape)] = (210, 60, 40)
    for p in path_xy[::2]:
        ix = int(np.clip((p[0] - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((p[1] - origin[1]) / res, 0, h - 1))
        img[iy, ix] = (60, 120, 255)
    Image.fromarray(np.flipud(img)).save(os.path.join(OUT, 'route_preview.png'))
    print('wrote', OUT)


if __name__ == '__main__':
    main()
