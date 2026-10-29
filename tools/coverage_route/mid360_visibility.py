#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MID360 visibility model on the Campus mesh (offline coverage planning).

    python3 tools/coverage_route/mid360_visibility.py [--pose 0,0] [--az 720] [--el 64]

Sensor (go2_description/config/mid360.yaml):
  * 360 deg azimuth, elevation -7..+52 deg, range 0.1..40 m
  * mount xyz (0.187, 0, 0.0803) relative to trunk/IMU, pitch +13 deg
    (positive pitch about +Y tilts the boresight down in ROS convention)
  * Go2 trunk height above ground at spawn = 0.45 m -> sensor z ~= ground + 0.530 m

Output: per-cell hit counts for one pose, PNG preview, and a JSON summary.
"""

import argparse
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'docs/dyn/coverage')
STL = os.path.join(ROOT, 'assets/runtime_worlds/campus/models/campus_runtime/campus_collision.stl')
WALK = os.path.join(OUT, 'walkable.npz')

MOUNT = np.array([0.1870, 0.0, 0.0803])
PITCH = np.deg2rad(13.0)
BASE_HEIGHT = 0.45
RANGE_MIN, RANGE_MAX = 0.1, 40.0


def load_world():
    w = np.load(WALK)
    gz = w['ground_z'].astype(np.float64)
    walkable = w['walkable'].astype(bool)
    res = float(w['res'])
    origin = w['origin']
    return gz, walkable, res, origin


def ray_dirs(az_n=720, el_n=64):
    az = np.linspace(-np.pi, np.pi, az_n, endpoint=False)
    el = np.deg2rad(np.linspace(-7.0, 52.0, el_n))
    azg, elg = np.meshgrid(az, el, indexing='xy')
    x = np.cos(elg) * np.cos(azg)
    y = np.cos(elg) * np.sin(azg)
    z = np.sin(elg)
    d = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    c, s = np.cos(PITCH), np.sin(PITCH)
    ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return d @ ry.T


class CampusRaycaster:
    def __init__(self, stl=STL):
        mesh = trimesh.load(stl, process=False)
        self.intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh)
        self.mesh = mesh

    def visible_ids(self, pose_xy, gz, walkable, res, origin, yaw=0.0,
                    az_n=720, el_n=64, z_tol=0.15):
        """Same model as visible_cells but returns unique walkable cell ids."""
        px, py = pose_xy
        h, w = walkable.shape
        ix = int(np.clip((px - origin[0]) / res, 0, w - 1))
        iy = int(np.clip((py - origin[1]) / res, 0, h - 1))
        g0 = gz[iy, ix]
        if not np.isfinite(g0):
            win = gz[max(0, iy - 5):iy + 6, max(0, ix - 5):ix + 6]
            if not np.isfinite(win).any():
                return np.zeros(0, dtype=np.int64), {'hits': 0}
            wy0, wx0 = max(0, iy - 5), max(0, ix - 5)
            yy, xx = np.nonzero(np.isfinite(win))
            dd = (yy - (iy - wy0)) ** 2 + (xx - (ix - wx0)) ** 2
            k = int(np.argmin(dd))
            iy, ix = wy0 + yy[k], wx0 + xx[k]
            px = origin[0] + (ix + 0.5) * res
            py = origin[1] + (iy + 0.5) * res
            g0 = gz[iy, ix]
        origin3 = np.array([px, py, g0 + BASE_HEIGHT]) + \
            np.array([MOUNT[0] * np.cos(yaw), MOUNT[0] * np.sin(yaw), MOUNT[2]])
        dirs = ray_dirs(az_n, el_n)
        if yaw:
            c, s = np.cos(yaw), np.sin(yaw)
            dirs = dirs @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
        loc, _, _ = self.intersector.intersects_location(
            np.broadcast_to(origin3, dirs.shape), dirs, multiple_hits=False)
        if len(loc) == 0:
            return np.zeros(0, dtype=np.int64), {'hits': 0}
        dist = np.linalg.norm(loc - origin3, axis=1)
        keep = (dist >= RANGE_MIN) & (dist <= RANGE_MAX)
        loc = loc[keep]
        cx = np.clip(((loc[:, 0] - origin[0]) / res).astype(int), 0, w - 1)
        cy = np.clip(((loc[:, 1] - origin[1]) / res).astype(int), 0, h - 1)
        good = (np.abs(loc[:, 2] - gz[cy, cx]) <= z_tol) & walkable[cy, cx]
        cid = cy[good].astype(np.int64) * w + cx[good]
        ids = np.unique(cid)
        stats = {'rays': int(len(dirs)), 'hits': int(len(loc)),
                 'ground_hits': int(good.sum()), 'visible_cells': int(len(ids)),
                 'visible_area_m2': float(len(ids) * res * res)}
        return ids, stats

    def visible_cells(self, pose_xy, gz, walkable, res, origin, yaw=0.0,
                      az_n=720, el_n=64, z_tol=0.15):
        px, py = pose_xy
        sy, sx = np.divmod(np.arange(walkable.size), walkable.shape[1])
        ix = int(np.clip((px - origin[0]) / res, 0, walkable.shape[1] - 1))
        iy = int(np.clip((py - origin[1]) / res, 0, walkable.shape[0] - 1))
        g0 = gz[iy, ix]
        if not np.isfinite(g0):
            raise ValueError('pose not on ground raster')
        origin3 = np.array([px, py, g0 + BASE_HEIGHT]) + \
            np.array([MOUNT[0] * np.cos(yaw), MOUNT[0] * np.sin(yaw), MOUNT[2]])
        dirs = ray_dirs(az_n, el_n)
        if yaw:
            c, s = np.cos(yaw), np.sin(yaw)
            rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            dirs = dirs @ rz.T
        origins = np.broadcast_to(origin3, dirs.shape)
        loc, ray_idx, tri_idx = self.intersector.intersects_location(
            origins, dirs, multiple_hits=False)
        if len(loc) == 0:
            return np.zeros_like(gz, dtype=np.int32), {'hits': 0}
        dist = np.linalg.norm(loc - origin3, axis=1)
        keep = (dist >= RANGE_MIN) & (dist <= RANGE_MAX)
        loc, dist = loc[keep], dist[keep]
        cx = np.clip(((loc[:, 0] - origin[0]) / res).astype(int), 0, walkable.shape[1] - 1)
        cy = np.clip(((loc[:, 1] - origin[1]) / res).astype(int), 0, walkable.shape[0] - 1)
        cid = cy * walkable.shape[1] + cx
        near_ground = np.abs(loc[:, 2] - gz[cy, cx]) <= z_tol
        good = near_ground & walkable[cy, cx]
        hits = np.zeros(walkable.size, dtype=np.int32)
        np.add.at(hits, cid[good], 1)
        stats = {
            'rays': int(len(dirs)), 'hits': int(len(loc)),
            'ground_hits': int(good.sum()),
            'visible_cells': int((hits > 0).sum()),
            'nearest_ground_hit_m': float(dist[good].min()) if good.any() else None,
            'visible_area_m2': float((hits > 0).sum() * res * res),
        }
        return hits, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', default='0,0')
    ap.add_argument('--az', type=int, default=720)
    ap.add_argument('--el', type=int, default=64)
    ap.add_argument('--compare-sim', action='store_true',
                    help='also cast the 625x32 simulator raster for comparison')
    args = ap.parse_args()
    px, py = [float(x) for x in args.pose.split(',')]
    gz, walkable, res, origin = load_world()
    rc = CampusRaycaster()
    hits, stats = rc.visible_cells((px, py), gz, walkable, res, origin,
                                   az_n=args.az, el_n=args.el)
    if args.compare_sim:
        _, sim_stats = rc.visible_cells((px, py), gz, walkable, res, origin,
                                        az_n=625, el_n=32)
        stats['sim_625x32'] = sim_stats
    print(json.dumps(stats, indent=1))

    from PIL import Image
    img = np.zeros(walkable.shape + (3,), dtype=np.uint8)
    img[walkable] = (90, 90, 90)
    vis = hits.reshape(walkable.shape) > 0
    img[vis] = (40, 170, 60)
    ix = int((px - origin[0]) / res)
    iy = int((py - origin[1]) / res)
    img[max(0, iy - 2):iy + 3, max(0, ix - 2):ix + 3] = (255, 255, 255)
    os.makedirs(OUT, exist_ok=True)
    Image.fromarray(np.flipud(img)).save(os.path.join(OUT, 'visibility_spawn.png'))
    with open(os.path.join(OUT, 'visibility_spawn.json'), 'w') as f:
        json.dump(stats, f, indent=1)
    print('wrote', os.path.join(OUT, 'visibility_spawn.png'))


if __name__ == '__main__':
    main()
