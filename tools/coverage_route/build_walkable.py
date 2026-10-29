#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build a walkable ground graph for the Campus mesh (outdoor, Go2-sized).

    python3 tools/coverage_route/build_walkable.py [--res 0.3] [--slope 30] [--step 0.20]

Method
  1. Near-horizontal triangles (slope <= slope_max+5) become ground samples.
  2. Per cell take the 10th percentile height -> ground layer (roofs/canopies
     are above it and stay disconnected).
  3. Vertical triangles whose z overlaps the body band above the local ground
     (0.15..0.90 m) are obstacles; scan holes count as obstacles too.
  4. Obstacles and holes are dilated by the Go2 half-width (0.35 m).
  5. 8-neighbour edges require |dz| <= step and slope <= slope_max.
  6. Keep the connected component containing the spawn cell (0,0).

Outputs (docs/dyn/coverage/):
  walkable.npz          ground z, free/walkable masks, metadata
  walkable_preview.png  green walkable / grey ground / red blocked / spawn dot
"""

import argparse
import os
import sys

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from mesh_probe import load_stl, DEFAULT_STL, OUT_DIR  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stl', default=DEFAULT_STL)
    ap.add_argument('--res', type=float, default=0.3)
    ap.add_argument('--slope', type=float, default=30.0, help='max walkable slope (deg)')
    ap.add_argument('--step', type=float, default=0.20, help='max step between cells (m)')
    ap.add_argument('--footprint', type=float, default=0.35, help='Go2 half-width (m)')
    ap.add_argument('--spawn', default='0,0')
    args = ap.parse_args()
    sx, sy = [float(x) for x in args.spawn.split(',')]
    res = args.res

    v = load_stl(args.stl)
    e1 = v[:, 1] - v[:, 0]
    e2 = v[:, 2] - v[:, 0]
    cr = np.cross(e1, e2)
    norm = np.linalg.norm(cr, axis=1)
    ok = norm > 1e-12
    unit = np.zeros_like(cr)
    unit[ok] = cr[ok] / norm[ok, None]
    slope = np.degrees(np.arccos(np.clip(np.abs(unit[:, 2]), 0, 1)))

    lo = v.reshape(-1, 3).min(0)
    hi = v.reshape(-1, 3).max(0)
    w = int(np.ceil((hi[0] - lo[0]) / res))
    h = int(np.ceil((hi[1] - lo[1]) / res))
    print(f'mesh {len(v):,} tri, raster {w}x{h} @ {res} m')

    def cell_of(xy):
        ix = np.clip(((xy[:, 0] - lo[0]) / res).astype(np.int64), 0, w - 1)
        iy = np.clip(((xy[:, 1] - lo[1]) / res).astype(np.int64), 0, h - 1)
        return iy * w + ix

    # ---- ground samples -------------------------------------------------
    horiz = ok & (slope <= args.slope + 5.0)
    tri = v[horiz]
    mids = 0.5 * (tri[:, [0, 1, 2], :] + tri[:, [1, 2, 0], :])
    pts = np.concatenate([tri.reshape(-1, 3), mids.reshape(-1, 3),
                          tri.mean(axis=1)], axis=0)
    cid = cell_of(pts[:, :2])
    order = np.lexsort((pts[:, 2], cid))
    cid_s, z_s = cid[order], pts[order, 2]
    uniq, starts = np.unique(cid_s, return_index=True)
    counts = np.diff(np.append(starts, len(z_s)))
    pick = starts + np.maximum((counts * 0.10).astype(np.int64), 0)
    ground_z = np.full(h * w, np.nan, dtype=np.float32)
    ground_z[uniq] = z_s[pick]
    ground = ~np.isnan(ground_z)
    print(f'ground cells={ground.sum():,} ({ground.sum()*res*res:,.0f} m^2), '
          f'samples={len(pts):,}')

    # close sub-cell sampling gaps: nearest ground height within 0.6 m
    gz3 = ground_z.reshape(h, w)
    missing = np.isnan(gz3)
    if missing.any():
        dist, (iy0, ix0) = ndimage.distance_transform_edt(
            missing, return_indices=True)
        fill = missing & (dist * res <= 0.6)
        gz3[fill] = gz3[iy0[fill], ix0[fill]]
    ground = ~np.isnan(gz3).reshape(-1)
    ground_z = gz3.reshape(-1)
    print(f'ground after gap fill={ground.sum():,} ({ground.sum()*res*res:,.0f} m^2)')

    # suppress scan spikes before edge tests (curbs survive 3x3 median)
    solid = np.where(np.isnan(gz3), 0.0, gz3)
    med = ndimage.median_filter(solid, size=3)
    gz3 = np.where(np.isnan(gz3), gz3, med)
    ground_z = gz3.reshape(-1)

    # ---- obstacles: vertical faces in the body band ---------------------
    vert = ok & (slope > 45.0)
    vpts = np.concatenate([v[vert].reshape(-1, 3),
                           v[vert].mean(axis=1)], axis=0)
    vcid = cell_of(vpts[:, :2])
    gz = ground_z[vcid]
    body = (~np.isnan(gz)) & (vpts[:, 2] >= gz + 0.15) & (vpts[:, 2] <= gz + 0.90)
    obstacle = np.zeros(h * w, dtype=bool)
    obstacle[vcid[body]] = True
    print(f'obstacle samples={len(vpts):,}, body-band cells={obstacle.sum():,}')

    # no-ground cells and obstacles are both barriers; keep footprint distance
    barrier = (obstacle | ~ground).reshape(h, w)
    blocked = (ndimage.distance_transform_edt(~barrier) * res) <= args.footprint
    free = ground & ~blocked.reshape(-1)
    print(f'free cells={free.sum():,} ({free.sum()*res*res:,.0f} m^2)')

    # ---- 8-neighbour graph with height/slope limits ---------------------
    def idx(iy, ix):
        return iy * w + ix
    iy, ix = np.divmod(np.arange(h * w), w)
    rows, cols = [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        y2, x2 = iy + dy, ix + dx
        valid = (y2 >= 0) & (y2 < h) & (x2 >= 0) & (x2 < w)
        a = np.nonzero(valid & free)[0]
        b = idx(y2[a], x2[a])
        dz = np.abs(ground_z[a] - ground_z[b])
        dist = res * (1.41421356237 if dx else 1.0)
        ok_edge = np.isfinite(dz) & free[b] & \
            (dz <= max(args.step, np.tan(np.radians(args.slope)) * dist))
        rows.append(a[ok_edge])
        cols.append(b[ok_edge])
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    n = h * w
    graph = coo_matrix((np.ones(len(rows), dtype=np.uint8), (rows, cols)),
                       shape=(n, n)).tocsr()
    ncomp, labels = connected_components(graph, directed=False)
    sid = int(cell_of(np.array([[sx, sy]]))[0])
    if not free[sid]:
        fr = np.nonzero(free)[0]
        fy, fx = np.divmod(fr, w)
        syc = int(np.clip((sy - lo[1]) / res, 0, h - 1))
        sxc = int(np.clip((sx - lo[0]) / res, 0, w - 1))
        d = np.hypot(fy - syc, fx - sxc) * res
        k = int(np.argmin(d))
        print(f'spawn cell not free; nearest free cell {d[k]:.2f} m away')
        if d[k] > 3.0:
            sys.exit('no free cell within 3 m of spawn')
        sid = int(fr[k])
    spawn_label = labels[sid]
    walkable = free & (labels == spawn_label)
    print(f'components={ncomp:,}, spawn z={ground_z[sid]:.2f}, '
          f'walkable cells={walkable.sum():,} ({walkable.sum()*res*res:,.0f} m^2)')

    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez_compressed(
        os.path.join(OUT_DIR, 'walkable.npz'),
        ground_z=ground_z.reshape(h, w), free=free.reshape(h, w),
        walkable=walkable.reshape(h, w), blocked=blocked,
        origin=lo[:2], res=res, shape=np.array([h, w]),
        spawn=np.array([sx, sy]))

    from PIL import Image
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[ground.reshape(h, w)] = (120, 120, 120)
    img[blocked] = (170, 40, 40)
    img[walkable.reshape(h, w)] = (40, 170, 60)
    sy_c = min(max(int((sy - lo[1]) / res), 0), h - 1)
    sx_c = min(max(int((sx - lo[0]) / res), 0), w - 1)
    img[max(0, sy_c - 3):sy_c + 4, max(0, sx_c - 3):sx_c + 4] = (255, 255, 255)
    Image.fromarray(np.flipud(img)).save(os.path.join(OUT_DIR, 'walkable_preview.png'))
    print('wrote', os.path.join(OUT_DIR, 'walkable.npz'),
          'and walkable_preview.png')


if __name__ == '__main__':
    main()
