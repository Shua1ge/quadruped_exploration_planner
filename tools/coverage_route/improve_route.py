#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Improve an initial route by coverage-guided detour insertion.

    python3 tools/coverage_route/improve_route.py [--target 0.99] [--max-inserts 150]

Inputs : docs/dyn/coverage/route/route.csv (initial route)
         docs/dyn/coverage/candidate_vis.pkl, achievable.npz
Outputs: docs/dyn/coverage/route/route_improved.csv, plan_improved.json,
         route_improved.png

Method: coverage is continuous along the path.  Repeatedly insert the candidate
viewpoint with the best (newly covered cells) / (detour length), recomputing the
new segments' visibility with MID360 rays.  The reported final number is a full
recompute over the final route, not the cumulative optimistic count.
"""

import argparse
import heapq
import json
import math
import os
import pickle
import sys
import time
from multiprocessing import Pool

import numpy as np
from scipy.sparse.csgraph import dijkstra

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage/route')
sys.path.insert(0, HERE)
import mid360_visibility as mv  # noqa: E402
from plan_coverage import _init_worker, _visible, build_graph, coarsen  # noqa: E402


def sample_points(seq, cw, origin, cell, step=1.0):
    """Polyline sample every `step` metres from coarse cell ids."""
    ys, xs = np.divmod(np.asarray(seq), cw)
    pts = np.stack([origin[0] + (xs + 0.5) * cell,
                    origin[1] + (ys + 0.5) * cell], axis=1)
    if len(pts) < 2:
        return [tuple(pts[0])] if len(pts) else []
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    acc = np.concatenate([[0], np.cumsum(seg)])
    out = []
    for s in np.arange(0, acc[-1], step):
        k = int(np.searchsorted(acc, s, side='right') - 1)
        k = min(max(k, 0), len(pts) - 1)
        out.append((float(pts[k, 0]), float(pts[k, 1])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=float, default=0.99,
                    help='fraction of achievable coverage to reach')
    ap.add_argument('--max-inserts', type=int, default=150)
    ap.add_argument('--min-gain', type=int, default=20,
                    help='minimum newly covered cells (0.3 m cells)')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--az', type=int, default=360)
    ap.add_argument('--el', type=int, default=32)
    ap.add_argument('--route-res', type=float, default=1.2)
    ap.add_argument('--route', default='route.csv')
    args = ap.parse_args()

    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    walk_flat = walk.reshape(-1)
    n_target = int(walk_flat.sum())
    av = np.load(os.path.join(OUT, '..', 'achievable.npz'))
    avail = av['avail'].reshape(-1)
    n_avail = int((avail & walk_flat).sum())
    goal = int(math.ceil(n_avail * args.target))
    print(f'walkable={n_target}, achievable={n_avail} '
          f'({100*n_avail/n_target:.1f}%), goal={goal} '
          f'({100*goal/n_target:.1f}% of walkable)')

    with open(os.path.join(OUT, '..', 'candidate_vis.pkl'), 'rb') as f:
        cv = pickle.load(f)
    vis = cv['vis']
    print(f'candidates={len(vis)}')

    # coarse routing graph
    cwalk, cgz, s = coarsen(walk, gz, res, args.route_res)
    ch, cw = cwalk.shape
    cell = s * res
    graph = build_graph(cwalk, cgz, cell)
    print(f'routing grid={ch}x{cw} @ {cell:.2f} m')

    def coarse_of_fine(fid):
        fy, fx = divmod(int(fid), wd)
        return (fy // s) * cw + (fx // s)

    # initial route cells
    route_pts = np.loadtxt(os.path.join(OUT, args.route), delimiter=',',
                           skiprows=1)
    cells = []
    for x, y in route_pts[:, :2]:
        cx = int(np.clip((x - origin[0]) / cell, 0, cw - 1))
        cy = int(np.clip((y - origin[1]) / cell, 0, ch - 1))
        cid = cy * cw + cx
        if not cwalk[cy, cx]:
            continue
        if not cells or cells[-1] != cid:
            cells.append(cid)
    print(f'initial route cells={len(cells)}')

    pool = Pool(args.workers, initializer=_init_worker,
                initargs=(gz, walk, res, origin))

    def coverage_of(pts):
        tasks = []
        for x, y in pts:
            ix = int(np.clip((x - origin[0]) / res, 0, wd - 1))
            iy = int(np.clip((y - origin[1]) / res, 0, h - 1))
            tasks.append((iy * wd + ix, args.az, args.el))
        cov = np.zeros(walk.size, dtype=bool)
        for _, ids in pool.map(_visible, tasks, chunksize=16):
            cov[ids] = True
        return cov & walk_flat

    def path_between(a, b):
        dist, pred = dijkstra(graph, directed=False, indices=a,
                              return_predecessors=True)
        if not np.isfinite(dist[b]):
            return None
        seq = []
        cur = b
        while cur != a and cur >= 0:
            seq.append(cur)
            cur = pred[cur]
        seq.append(a)
        return seq[::-1]

    # initial coverage
    t0 = time.time()
    pts = []
    for a, b in zip(cells, cells[1:]):
        seq = path_between(a, b)
        if seq:
            pts.extend(sample_points(seq, cw, origin, cell))
    covered = coverage_of(pts)
    print(f'initial continuous coverage={covered.sum()}/{n_target} '
          f'({100*covered.sum()/n_target:.1f}%, {covered.sum()*res*res:.0f} m2), '
          f'{time.time()-t0:.0f}s')

    # candidate coarse cells and fine ids
    cand_ids = np.array(sorted(vis.keys()), dtype=np.int64)

    inserts = 0
    while covered.sum() < goal and inserts < args.max_inserts:
        uncovered = walk_flat & avail & ~covered
        # gain per candidate
        gains = np.array([int(uncovered[vis[int(c)]].sum()) for c in cand_ids])
        order = np.argsort(-gains)[:120]
        best = None
        cells_arr = np.asarray(cells)
        cy_, cx_ = np.divmod(cells_arr, cw)
        seg_e = np.hypot(np.diff(cx_), np.diff(cy_)) * cell
        for k in order:
            gain = int(gains[k])
            if gain < args.min_gain:
                break
            cid = int(cand_ids[k])
            cc = coarse_of_fine(cid)
            if cc in cells or not cwalk.reshape(-1)[cc]:
                continue
            dist = dijkstra(graph, directed=False, indices=cc)
            dd = dist[cells_arr].astype(np.float64)
            # append at the end
            if np.isfinite(dd[-1]):
                sc = gain / max(float(dd[-1]), 1.0)
                if best is None or sc > best[0]:
                    best = (sc, len(cells), cid, cc, gain, float(dd[-1]))
            # insert before index i (1..len-1)
            if len(seg_e):
                delta = dd[:-1] + dd[1:] - seg_e
                finite = np.isfinite(delta)
                if finite.any():
                    j = int(np.argmin(np.where(finite, delta, np.inf)))
                    sc = gain / max(float(delta[j]), 1.0)
                    if best is None or sc > best[0]:
                        best = (sc, j + 1, cid, cc, gain, float(delta[j]))
        if best is None:
            print('no candidate with sufficient gain; stopping')
            break
        score, pos, cid, cc, gain, delta = best
        new_pts = []
        if pos >= len(cells):
            seq = path_between(cells[-1], cc)
            if seq:
                new_pts.extend(sample_points(seq, cw, origin, cell))
            cells.append(cc)
        else:
            seq1 = path_between(cells[pos - 1], cc)
            seq2 = path_between(cc, cells[pos])
            if seq1:
                new_pts.extend(sample_points(seq1, cw, origin, cell))
            if seq2:
                new_pts.extend(sample_points(seq2, cw, origin, cell))
            cells.insert(pos, cc)
        fy, fx = divmod(cid, wd)
        new_pts.append((origin[0] + (fx + 0.5) * res,
                        origin[1] + (fy + 0.5) * res))
        cov_new = coverage_of(new_pts) if new_pts else np.zeros_like(covered)
        before = int(covered.sum())
        covered |= cov_new
        inserts += 1
        print(f'insert {inserts}: gain~{gain} cells, new={int(covered.sum())-before}, '
              f'detour~{delta:.0f} m, route nodes={len(cells)}, '
              f'coverage={100*covered.sum()/n_target:.2f}%', flush=True)

    # full recompute over the final route (honest number)
    pts = []
    for a, b in zip(cells, cells[1:]):
        seq = path_between(a, b)
        if seq:
            pts.extend(sample_points(seq, cw, origin, cell))
    final_cov = coverage_of(pts)
    print(f'final continuous coverage={final_cov.sum()}/{n_target} '
          f'({100*final_cov.sum()/n_target:.2f}%, '
          f'{100*final_cov.sum()/n_avail:.1f}% of achievable)')

    # write geometry
    geom, zs = [], []
    for a, b in zip(cells, cells[1:]):
        seq = path_between(a, b)
        if not seq:
            continue
        ys, xs = np.divmod(np.asarray(seq), cw)
        geom.extend(np.stack([origin[0] + (xs + 0.5) * cell,
                              origin[1] + (ys + 0.5) * cell], axis=1))
        zs.extend(cgz.reshape(-1)[seq])
    geom = np.asarray(geom)
    zs = np.asarray(zs)
    seg = np.linalg.norm(np.diff(geom, axis=0), axis=1)
    length = float(seg.sum())
    os.makedirs(OUT, exist_ok=True)
    np.savetxt(os.path.join(OUT, 'route_improved.csv'),
               np.column_stack([geom, zs]), delimiter=',', header='x,y,z',
               comments='')
    missed = (walk_flat & avail & ~final_cov)
    from scipy import ndimage
    lab, nlab = ndimage.label(missed.reshape(h, wd))
    sizes = ndimage.sum(missed.reshape(h, wd), lab, range(1, nlab + 1)) * res * res
    big = np.sort(sizes)[::-1][:15]
    plan = {
        'initial_coverage_pct': round(100 * covered.sum() / n_target, 2),
        'final_coverage_pct': round(100 * final_cov.sum() / n_target, 2),
        'final_coverage_of_achievable_pct': round(100 * final_cov.sum() / n_avail, 2),
        'achievable_pct': round(100 * n_avail / n_target, 2),
        'walkable_area_m2': round(float(n_target * res * res), 1),
        'route_length_m': round(length, 1),
        'inserts': inserts,
        'route_nodes': len(cells),
        'missed_clusters_top_m2': big.round(1).tolist(),
        'missed_cluster_count': int(nlab),
        'target_frac_of_achievable': args.target,
        'rays': [args.az, args.el],
        'route_res_m': cell,
    }
    with open(os.path.join(OUT, 'plan_improved.json'), 'w') as f:
        json.dump(plan, f, indent=1, ensure_ascii=False)

    from PIL import Image
    img = np.zeros((h, wd, 3), dtype=np.uint8)
    img[walk] = (80, 80, 80)
    img[walk & final_cov.reshape(walk.shape)] = (40, 160, 60)
    img[walk & avail.reshape(h, wd) & ~final_cov.reshape(h, wd)] = (210, 60, 40)
    for p in geom[::3]:
        ix = int(np.clip((p[0] - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((p[1] - origin[1]) / res, 0, h - 1))
        img[iy, ix] = (60, 120, 255)
    Image.fromarray(np.flipud(img)).save(os.path.join(OUT, 'route_improved.png'))
    print('wrote', OUT)
    pool.close()
    pool.join()


if __name__ == '__main__':
    main()
