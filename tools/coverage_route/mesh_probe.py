#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Campus mesh probe: bounds, slope/height statistics, top-down previews.

Usage:
    python3 tools/coverage_route/mesh_probe.py [campus_collision.stl] [--res 0.5]

Reads the binary STL produced by tools/worlds/prepare_campus.py and reports:
  * triangle/vertex counts, bounds, total area
  * area by slope band (0-10/10-20/20-30/30-40/>40 deg)
  * area of near-horizontal surface by 2 m height band (floors vs roofs)
  * height at spawn (0,0)
  * 0.5 m occupancy/height/slope PNGs under docs/dyn/coverage/

No walkability decision is made here; this only checks the assumptions the
planner will rely on (ground extent, multi-level structure, scan holes).
"""

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_STL = os.path.join(ROOT, 'assets/runtime_worlds/campus/models/campus_runtime/campus_collision.stl')
OUT_DIR = os.path.join(ROOT, 'docs/dyn/coverage')


def load_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
        data = np.frombuffer(f.read(n * rec.itemsize), dtype=rec, count=n)
    return data['v'].astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stl', nargs='?', default=DEFAULT_STL)
    ap.add_argument('--res', type=float, default=0.5)
    args = ap.parse_args()
    if not os.path.isfile(args.stl):
        sys.exit(f'missing STL: {args.stl} (run tools/worlds/prepare_campus.py first)')

    v = load_stl(args.stl)
    n = len(v)
    e1 = v[:, 1] - v[:, 0]
    e2 = v[:, 2] - v[:, 0]
    cr = np.cross(e1, e2)
    norm = np.linalg.norm(cr, axis=1)
    area = 0.5 * norm
    ok = norm > 1e-12
    unit = np.zeros_like(cr)
    unit[ok] = cr[ok] / norm[ok, None]
    nz = np.abs(unit[:, 2])
    slope = np.degrees(np.arccos(np.clip(nz, -1, 1)))

    print(f'triangles={n:,}  vertices={v.reshape(-1,3).shape[0]:,}  '
          f'area={area.sum():,.1f} m^2')
    lo, hi = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
    print('bounds x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    bins = [0, 10, 20, 30, 40, 90]
    print('area by slope (deg):')
    for a, b in zip(bins, bins[1:]):
        m = (slope >= a) & (slope < b)
        print(f'  {a:>2}-{b:<2}: {area[m].sum():12,.0f} m^2 ({100*area[m].sum()/area.sum():5.1f}%)')

    horiz = (slope < 20) & ok
    zc = v[:, :, 2].mean(axis=1)
    print('near-horizontal (<20 deg) area by 2 m height band:')
    for z0 in np.arange(np.floor(lo[2] / 2) * 2, hi[2] + 2, 2):
        m = horiz & (zc >= z0) & (zc < z0 + 2)
        if area[m].sum() > 200:
            print(f'  z {z0:6.1f}..{z0+2:6.1f}: {area[m].sum():12,.0f} m^2')

    # spawn height: horizontal triangles within 1.5 m of (0,0)
    cx = (v[:, 0, 0] + v[:, 1, 0] + v[:, 2, 0]) / 3
    cy = (v[:, 0, 1] + v[:, 1, 1] + v[:, 2, 1]) / 3
    near = horiz & (cx * cx + cy * cy < 1.5 ** 2)
    if near.any():
        zs = np.sort(zc[near])
        print(f'spawn (0,0) horizontal surface z: min={zs.min():.2f} '
              f'p50={np.median(zs):.2f} max={zs.max():.2f} (n={near.sum()})')

    # top-down rasters (horizontal surfaces only; roofs included on purpose)
    res = args.res
    w = int(np.ceil((hi[0] - lo[0]) / res))
    h = int(np.ceil((hi[1] - lo[1]) / res))
    hmax = np.full((h, w), np.nan, dtype=np.float32)
    hmin = np.full((h, w), np.nan, dtype=np.float32)
    smax = np.full((h, w), np.nan, dtype=np.float32)
    ground = np.zeros((h, w), dtype=bool)
    ix = np.clip(((cx - lo[0]) / res).astype(int), 0, w - 1)
    iy = np.clip(((cy - lo[1]) / res).astype(int), 0, h - 1)
    for i in np.nonzero(horiz)[0]:
        x, y, z = ix[i], iy[i], zc[i]
        if np.isnan(hmax[y, x]) or z > hmax[y, x]:
            hmax[y, x] = z
        if np.isnan(hmin[y, x]) or z < hmin[y, x]:
            hmin[y, x] = z
        if np.isnan(smax[y, x]) or slope[i] > smax[y, x]:
            smax[y, x] = slope[i]
        ground[y, x] = True

    os.makedirs(OUT_DIR, exist_ok=True)
    from PIL import Image
    def colorize(arr, vmin, vmax):
        a = np.array(arr, dtype=np.float32)
        m = ~np.isnan(a)
        out = np.zeros(a.shape + (3,), dtype=np.uint8)
        if m.any():
            t = np.clip((a[m] - vmin) / max(1e-6, vmax - vmin), 0, 1)
            out[m, 0] = (255 * (1 - t)).astype(np.uint8)
            out[m, 1] = (255 * (1 - np.abs(t - 0.5) * 2)).astype(np.uint8)
            out[m, 2] = (255 * t).astype(np.uint8)
        return out
    Image.fromarray(colorize(hmax, np.nanmin(hmax), np.nanmax(hmax))).save(
        os.path.join(OUT_DIR, 'height_max.png'))
    Image.fromarray(colorize(smax, 0, 60)).save(
        os.path.join(OUT_DIR, 'slope_max.png'))
    Image.fromarray((ground.astype(np.uint8) * 255)).save(
        os.path.join(OUT_DIR, 'horizontal_mask.png'))

    cells = int(ground.sum())
    print(f'raster {res} m: {w}x{h} cells, horizontal-covered={cells:,} '
          f'(~{cells*res*res:,.0f} m^2)')
    print('wrote', OUT_DIR)


if __name__ == '__main__':
    main()
