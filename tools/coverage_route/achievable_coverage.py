#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Achievable MID360 coverage over a dense candidate set (occlusion upper bound).

    python3 tools/coverage_route/achievable_coverage.py [--spacing 2.0] [--workers 8]

Writes docs/dyn/coverage/achievable.npz and prints coverage + uncovered clusters.
"""

import argparse
import os
import pickle
import sys
import time
from multiprocessing import Pool

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage')
sys.path.insert(0, HERE)
import mid360_visibility as mv  # noqa: E402
from plan_coverage import _init_worker, _visible  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spacing', type=float, default=2.0)
    ap.add_argument('--az', type=int, default=360)
    ap.add_argument('--el', type=int, default=32)
    ap.add_argument('--workers', type=int, default=8)
    args = ap.parse_args()
    w = np.load(mv.WALK)
    gz = w['ground_z'].astype(np.float64)
    walk = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    walk_flat = walk.reshape(-1)
    step = max(1, int(round(args.spacing / res)))
    cand = [iy * wd + ix
            for iy in range(step // 2, h, step)
            for ix in range(step // 2, wd, step) if walk[iy, ix]]
    cand = np.array(cand, dtype=np.int64)
    print(f'candidates={len(cand)} @ {args.spacing} m')
    t = time.time()
    with Pool(args.workers, initializer=_init_worker,
              initargs=(gz, walk, res, origin)) as pool:
        vis = dict(pool.map(_visible, [(int(i), args.az, args.el) for i in cand],
                            chunksize=8))
    print(f'visibility in {time.time()-t:.0f}s, '
          f'avg cells/pose={np.mean([len(v) for v in vis.values()]):.0f}')
    avail = np.zeros(walk.size, dtype=bool)
    for ids in vis.values():
        avail[ids] = True
    n_target = int(walk.sum())
    n_avail = int((avail & walk_flat).sum())
    print(f'achievable={n_avail}/{n_target} cells '
          f'({100*n_avail/n_target:.1f}%, {n_avail*res*res:.0f} m2)')
    missed = (walk_flat & ~avail).reshape(h, wd)
    lab, nlab = ndimage.label(missed)
    sizes = ndimage.sum(missed, lab, range(1, nlab + 1)) * res * res
    big = np.sort(sizes)[::-1]
    print(f'unreachable clusters: {nlab}, top areas m2: {big[:10].round(1).tolist()}')
    np.savez_compressed(os.path.join(OUT, 'achievable.npz'),
                        avail=avail.reshape(h, wd), walk=walk)
    with open(os.path.join(OUT, 'candidate_vis.pkl'), 'wb') as f:
        pickle.dump({'spacing': args.spacing, 'az': args.az, 'el': args.el,
                     'res': res, 'origin': origin, 'shape': (h, wd),
                     'vis': {int(k): v for k, v in vis.items()}}, f)
    print('wrote achievable.npz and candidate_vis.pkl')


if __name__ == '__main__':
    main()
