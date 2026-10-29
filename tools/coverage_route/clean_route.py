#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Snap the improved route to fine walkable cells and audit segment occupancy.

    python3 tools/coverage_route/clean_route.py [--route route_improved.csv]

Writes route_final.csv and clean_report.json.  This does not re-optimize the
route; it only removes coarse-cell centre artifacts and reports how much of the
final polyline crosses non-walkable fine cells.
"""

import argparse
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default='route_improved.csv')
    ap.add_argument('--max-step', type=float, default=1.2)
    args = ap.parse_args()
    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape

    # nearest walkable fine cell for every cell (used only where needed)
    dist, (iy0, ix0) = ndimage.distance_transform_edt(
        ~walk, return_indices=True)

    p = np.loadtxt(os.path.join(OUT, args.route), delimiter=',', skiprows=1)
    snapped, moved = [], 0
    for x, y, _ in p:
        ix = int(np.clip((x - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((y - origin[1]) / res, 0, h - 1))
        if walk[iy, ix]:
            cx, cy = ix, iy
        else:
            d = dist[iy, ix] * res
            if d > 2.0:
                continue
            cx, cy = int(ix0[iy, ix]), int(iy0[iy, ix])
            moved += 1
        nx = origin[0] + (cx + 0.5) * res
        ny = origin[1] + (cy + 0.5) * res
        nz = gz[cy, cx]
        if not snapped or np.hypot(nx - snapped[-1][0], ny - snapped[-1][1]) >= 0.30:
            snapped.append((nx, ny, nz))
    snapped = np.asarray(snapped)
    print(f'input points={len(p)}, snapped={len(snapped)}, moved={moved}')

    # resample to <= max_step
    out = [snapped[0]]
    for a, b in zip(snapped, snapped[1:]):
        d = np.linalg.norm(b - a)
        n = max(1, int(np.ceil(d / args.max_step)))
        for k in range(1, n + 1):
            t = k / n
            out.append(a + (b - a) * t)
    out = np.asarray(out)
    seg = np.linalg.norm(np.diff(out[:, :2], axis=0), axis=1)
    print(f'final points={len(out)}, length={seg.sum():.0f} m')

    # occupancy audit: sample every 0.3 m
    xy = out[:, :2]
    acc = np.concatenate([[0], np.cumsum(seg)])
    bad = tot = 0
    bad_xy = []
    for s in np.arange(0, acc[-1], 0.3):
        k = int(np.searchsorted(acc, s, side='right') - 1)
        k = min(max(k, 0), len(xy) - 1)
        t = (s - acc[k]) / max(seg[k], 1e-9) if k < len(seg) else 0.0
        p2 = xy[k] + (xy[k + 1] - xy[k]) * min(max(t, 0), 1) if k + 1 < len(xy) else xy[k]
        ix = int(np.clip((p2[0] - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((p2[1] - origin[1]) / res, 0, h - 1))
        tot += 1
        if not walk[iy, ix]:
            bad += 1
            if len(bad_xy) < 10:
                bad_xy.append([round(float(p2[0]), 1), round(float(p2[1]), 1)])
    print(f'occupancy audit: {bad}/{tot} samples on non-walkable cells '
          f'({100*bad/max(tot,1):.2f}%)')
    if bad_xy:
        print('first offenders', bad_xy)

    np.savetxt(os.path.join(OUT, 'route_final.csv'), out, delimiter=',',
               header='x,y,z', comments='', fmt='%.3f')
    report = {
        'input': args.route, 'input_points': int(len(p)),
        'snapped_points': int(len(snapped)), 'moved_points': int(moved),
        'final_points': int(len(out)),
        'length_m': round(float(seg.sum()), 1),
        'occupancy_samples': int(tot), 'occupancy_bad': int(bad),
        'occupancy_bad_pct': round(100 * bad / max(tot, 1), 2),
        'start': [round(float(out[0, 0]), 2), round(float(out[0, 1]), 2)],
        'end': [round(float(out[-1, 0]), 2), round(float(out[-1, 1]), 2)],
    }
    with open(os.path.join(OUT, 'clean_report.json'), 'w') as f:
        json.dump(report, f, indent=1)
    print('wrote route_final.csv, clean_report.json')


if __name__ == '__main__':
    main()
