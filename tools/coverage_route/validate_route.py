#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent coverage re-check of a route (denser rays, dilated tolerance).

    python3 tools/coverage_route/validate_route.py --route route_improved.csv --az 720 --el 64

Reporst strict cell-hit coverage and coverage after dilating the observed mask by
1 and 2 cells (0.3 / 0.6 m), which reflects that a mapped point still marks the
neighbouring cells.
"""

import argparse
import os
import sys
import time
from multiprocessing import Pool

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage/route')
sys.path.insert(0, HERE)
import mid360_visibility as mv  # noqa: E402
from plan_coverage import _init_worker, _visible  # noqa: E402
import json


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default='route_improved.csv')
    ap.add_argument('--az', type=int, default=720)
    ap.add_argument('--el', type=int, default=64)
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--workers', type=int, default=8)
    args = ap.parse_args()
    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    n = int(walk.sum())
    pts = np.loadtxt(os.path.join(OUT, args.route), delimiter=',', skiprows=1)[:, :2]
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    acc = np.concatenate([[0], np.cumsum(seg)])
    samples = []
    for s in np.arange(0, acc[-1], args.step):
        k = int(np.searchsorted(acc, s, side='right') - 1)
        k = min(max(k, 0), len(pts) - 1)
        p = pts[k]
        if not samples or np.hypot(p[0] - samples[-1][0], p[1] - samples[-1][1]) >= 0.4:
            samples.append((float(p[0]), float(p[1])))
    print(f'route points={len(pts)}, length={acc[-1]:.0f} m, samples={len(samples)}, '
          f'rays={args.az}x{args.el}')
    tasks = []
    for x, y in samples:
        ix = int(np.clip((x - origin[0]) / res, 0, wd - 1))
        iy = int(np.clip((y - origin[1]) / res, 0, h - 1))
        tasks.append((iy * wd + ix, args.az, args.el))
    t = time.time()
    with Pool(args.workers, initializer=_init_worker,
              initargs=(gz, walk, res, origin)) as pool:
        res_list = pool.map(_visible, tasks, chunksize=8)
    cov = np.zeros(walk.size, dtype=bool)
    for _, ids in res_list:
        cov[ids] = True
    cov &= walk.reshape(-1)
    print(f'raycast {time.time()-t:.0f}s')
    out = {'route': args.route, 'length_m': round(float(acc[-1]), 1),
           'samples': len(samples), 'rays': [args.az, args.el],
           'walkable_cells': n, 'coverage': {}}
    cov3 = cov.reshape(h, wd)
    for dil in (0, 1, 2, 3):
        m = cov3 if dil == 0 else ndimage.binary_dilation(
            cov3, structure=np.ones((2 * dil + 1, 2 * dil + 1), bool))
        c = int((m & walk).sum())
        out['coverage'][f'dilate_{dil}_cells'] = {
            'pct_walkable': round(100 * c / n, 2),
            'area_m2': round(float(c * res * res), 1)}
        print(f'dilate {dil} ({dil*res:.1f} m): {c}/{n} = {100*c/n:.2f}%')
    np.savez_compressed(os.path.join(OUT, 'route_coverage.npz'),
                        cov=cov3, walk=walk)
    with open(os.path.join(OUT, 'validation.json'), 'w') as f:
        json.dump(out, f, indent=1)
    print('wrote validation.json')


if __name__ == '__main__':
    main()
