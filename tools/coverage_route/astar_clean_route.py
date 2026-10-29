#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reconnect route waypoints with fine-grid A* that forbids corner cutting.

    python3 tools/coverage_route/astar_clean_route.py [--route route_final.csv] [--waypoint 4.0]

The improved route was optimized on a 1.2 m grid; straight segments can clip
non-walkable fine cells at corners.  This pass keeps the route's geometry but
walks between waypoints on the 0.3 m walkable graph (diagonal moves require both
orthogonal cells free, height step limited as in build_walkable.py).
"""

import argparse
import heapq
import json
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage/route')
sys.path.insert(0, HERE)
import mid360_visibility as mv  # noqa: E402

STEP = 0.20
SLOPE = np.tan(np.radians(30.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default='route_final.csv')
    ap.add_argument('--waypoint', type=float, default=4.0)
    ap.add_argument('--out', default='route_on_graph.csv')
    args = ap.parse_args()
    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    p = np.loadtxt(os.path.join(OUT, args.route), delimiter=',', skiprows=1)[:, :2]
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    acc = np.concatenate([[0], np.cumsum(seg)])
    marks = list(np.arange(0, acc[-1], args.waypoint)) + [acc[-1]]
    wps = []
    for s in marks:
        k = int(np.searchsorted(acc, s, side='right') - 1)
        k = min(max(k, 0), len(p) - 1)
        wps.append(p[k])
    print(f'route length={acc[-1]:.0f} m, waypoints={len(wps)}')

    dist, (iy0, ix0) = ndimage.distance_transform_edt(~walk, return_indices=True)

    def cell_of(pt):
        ix = int(np.clip((pt[0] - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((pt[1] - origin[1]) / res, 0, h - 1))
        if not walk[iy, ix]:
            ix, iy = int(ix0[iy, ix]), int(iy0[iy, ix])
        return iy * wd + ix

    def astar(a, b, max_expand=300000):
        if a == b:
            return [a]
        ay, ax = divmod(a, wd)
        by, bx = divmod(b, wd)
        hfun = lambda c: np.hypot((c // wd) - by, (c % wd) - bx) * res
        openq = [(hfun(a), 0.0, a)]
        g = {a: 0.0}
        par = {a: -1}
        expanded = 0
        while openq and expanded < max_expand:
            f, gc, c = heapq.heappop(openq)
            if c == b:
                seq = [c]
                while par[seq[-1]] >= 0:
                    seq.append(par[seq[-1]])
                return seq[::-1]
            if gc > g.get(c, 1e18):
                continue
            expanded += 1
            cy, cx = divmod(c, wd)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    y2, x2 = cy + dy, cx + dx
                    if not (0 <= y2 < h and 0 <= x2 < wd) or not walk[y2, x2]:
                        continue
                    if dy and dx and not (walk[cy, x2] and walk[y2, cx]):
                        continue  # no corner cutting
                    d = res * (1.41421356237 if dy and dx else 1.0)
                    if abs(gz[y2, x2] - gz[cy, cx]) > max(STEP, SLOPE * d):
                        continue
                    nid = y2 * wd + x2
                    ng = gc + d
                    if ng < g.get(nid, 1e18):
                        g[nid] = ng
                        par[nid] = c
                        heapq.heappush(openq, (ng + hfun(nid), ng, nid))
        return None

    cells = []
    fails = 0
    for a_pt, b_pt in zip(wps, wps[1:]):
        a, b = cell_of(a_pt), cell_of(b_pt)
        seq = astar(a, b)
        if seq is None:
            fails += 1
            continue
        if cells and cells[-1] == seq[0]:
            seq = seq[1:]
        cells.extend(seq)
    print(f'waypoint segments failed={fails}, cells={len(cells)}')
    ys, xs = np.divmod(np.asarray(cells), wd)
    out = np.stack([origin[0] + (xs + 0.5) * res,
                    origin[1] + (ys + 0.5) * res,
                    gz.reshape(-1)[np.asarray(cells)]], axis=1)
    # resample to 1.0 m for a lighter file
    keep = [0]
    for i in range(1, len(out)):
        if np.linalg.norm(out[i, :2] - out[keep[-1], :2]) >= 1.0:
            keep.append(i)
    if keep[-1] != len(out) - 1:
        keep.append(len(out) - 1)
    out = out[keep]
    s = np.linalg.norm(np.diff(out[:, :2], axis=0), axis=1)
    print(f'final points={len(out)}, length={s.sum():.0f} m')
    np.savetxt(os.path.join(OUT, args.out), out, delimiter=',',
               header='x,y,z', comments='', fmt='%.3f')
    rep = {'input': args.route, 'waypoint_m': args.waypoint,
           'segments_failed': fails, 'points': int(len(out)),
           'length_m': round(float(s.sum()), 1),
           'start': [round(float(out[0, 0]), 2), round(float(out[0, 1]), 2)],
           'end': [round(float(out[-1, 0]), 2), round(float(out[-1, 1]), 2)]}
    with open(os.path.join(OUT, 'astar_clean_report.json'), 'w') as f:
        json.dump(rep, f, indent=1)
    print('wrote', args.out, 'and astar_clean_report.json')


if __name__ == '__main__':
    main()
