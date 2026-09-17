#!/usr/bin/env python3
"""Crop a SubT mine/terrain OBJ into a self-contained SCAN-Planner scene.

The Edgar mine mesh (Fuel `openrobotics/edgar`) is authored in centimetres with
OBJ +Y as Gazebo world +Z, so `model.sdf` applies a 90 deg roll and a 0.01
scale.  This tool bakes that transform so the output OBJ is already in the ROS
world frame (Z up, metres), selects one XY window and one Z band, shifts the
geometry so a chosen floor reference sits at Z=0, and emits the four artefacts
the existing scenes use:

    scene.obj          pared mesh, world frame, Z shifted
    scene_surface.pcd  full surface truth sampled at --resolution
    scene_planar.pcd   obstacle surface whose height above the LOCAL floor
                       lies inside the robot waist band (default 0.08..0.85 m)
    scene.world        Gazebo Fortress world: mesh + four sealing boundary walls
    scene_meta.json    window, band, shift, spawn, counts

Why a local floor model: a real mine drift is sloped, so an absolute Z band
would only see obstacles near the spawn elevation.  Height above the local
floor is the quantity that actually decides whether GO2 can walk somewhere.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage


# --------------------------------------------------------------------------- #
# OBJ
# --------------------------------------------------------------------------- #

def parse_obj(path: Path):
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith("v "):
                a = line.split()
                vertices.append((float(a[1]), float(a[2]), float(a[3])))
            elif line.startswith("f "):
                idx = [item.split("/")[0] for item in line.split()[1:]]
                if len(idx) < 3:
                    continue
                # fan-triangulate any n-gon
                poly = [int(i) - 1 if int(i) > 0 else len(vertices) + int(i)
                        for i in idx]
                for k in range(1, len(poly) - 1):
                    faces.append((poly[0], poly[k], poly[k + 1]))
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def to_world(obj_vertices: np.ndarray) -> np.ndarray:
    """OBJ (cm, Y-up-ish) -> ROS world (m, Z up): scale 0.01 then roll +90 deg about X."""
    return np.column_stack((
        obj_vertices[:, 0] * 0.01,
        -obj_vertices[:, 2] * 0.01,
        obj_vertices[:, 1] * 0.01,
    ))


def vertex_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Area-weighted smooth vertex normals; DART/assimp needs vn to accept a mesh."""
    fn = face_normals(vertices[faces])
    if len(faces) == 0:
        return np.zeros_like(vertices)
    cr = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                  vertices[faces[:, 2]] - vertices[faces[:, 0]])
    acc = np.zeros_like(vertices)
    for k in range(3):
        np.add.at(acc, faces[:, k], cr)
    norm = np.linalg.norm(acc, axis=1)
    norm[norm == 0.0] = 1.0
    return acc / norm[:, None]


def write_obj(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    used = np.unique(faces)
    remap = {int(old): new + 1 for new, old in enumerate(used)}
    vn = vertex_normals(vertices, faces)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# Cropped from DARPA SubT (osrf/subt assets, CC0) by tools/crop_subt_mine.py\n")
        stream.write("o scene\n")
        for old in used:
            x, y, z = vertices[old]
            stream.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for old in used:
            x, y, z = vn[old]
            stream.write(f"vn {x:.4f} {y:.4f} {z:.4f}\n")
        for tri in faces:
            a, b, c = (remap[int(i)] for i in tri)
            stream.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #

def face_normals(tri: np.ndarray) -> np.ndarray:
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norm = np.linalg.norm(cr, axis=1)
    norm[norm == 0.0] = 1.0
    return cr / norm[:, None]


def sample_triangles(tri: np.ndarray, spacing: float) -> np.ndarray:
    """Deterministic barycentric lattice sampling of a triangle soup."""
    out = []
    a = tri[:, 1] - tri[:, 0]
    b = tri[:, 2] - tri[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    # number of subdivisions so that edge steps stay near `spacing`
    scale = np.sqrt(np.maximum(area, 0.0)) / max(spacing, 1e-6)
    counts = np.clip(np.ceil(scale).astype(int), 1, 400)
    for k in range(len(tri)):
        m = counts[k]
        if m <= 1:
            out.append(tri[k].mean(axis=0))
            continue
        idx = np.arange(m + 1)
        u, v = np.meshgrid(idx, idx)
        keep = (u + v) <= m
        u = u[keep] / m
        v = v[keep] / m
        p = (tri[k, 0][None, :]
             + u[:, None] * a[k][None, :]
             + v[:, None] * b[k][None, :])
        out.append(p)
    return np.vstack(out) if out else np.zeros((0, 3))


def local_floor_height(floor_tri: np.ndarray, resolution: float = 0.5):
    """Coarse 2D grid of floor Z plus the distance to the nearest real sample.

    Returns (grid, distance, (x0, y0, resolution)).  The distance field lets the
    caller reject lookups that would have to borrow a floor height from across a
    wall, which is exactly how a naive hole-fill invents obstacles high above a
    floor on the far side of solid rock.
    """
    cen = floor_tri.mean(axis=1)
    # x0/y0 must be metres: the old form forgot the * resolution, so it returned a
    # grid index instead.  That made (max - x0) negative -- and nx/ny negative --
    # for any window whose coordinates are large and positive.
    x0 = float(np.floor(cen[:, 0].min() / resolution) * resolution) - 2.0 * resolution
    y0 = float(np.floor(cen[:, 1].min() / resolution) * resolution) - 2.0 * resolution
    nx = int(math.ceil((cen[:, 0].max() - x0) / resolution)) + 3
    ny = int(math.ceil((cen[:, 1].max() - y0) / resolution)) + 3
    ix = ((cen[:, 0] - x0) / resolution).astype(int)
    iy = ((cen[:, 1] - y0) / resolution).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, zz = ix[ok], iy[ok], cen[ok, 2]
    flat = ix * ny + iy
    total = np.bincount(flat, weights=zz, minlength=nx * ny)
    count = np.bincount(flat, minlength=nx * ny)
    acc = np.divide(total, count, out=np.full(nx * ny, np.nan),
                    where=count > 0).reshape(nx, ny)
    valid = ~np.isnan(acc)
    if not valid.any():
        raise RuntimeError("no floor faces found for the floor model")
    dist, nearest = ndimage.distance_transform_edt(~valid, return_indices=True)
    filled = acc[nearest[0], nearest[1]]
    filled = ndimage.median_filter(filled, size=3)
    return filled, dist * resolution, (x0, y0, resolution)


def floor_lookup(grid: np.ndarray, dist: np.ndarray, meta, xy: np.ndarray,
                 max_dist: float):
    """Floor height under xy, or NaN where no floor sample is close enough."""
    x0, y0, res = meta
    nx, ny = grid.shape
    ix = np.clip(((xy[:, 0] - x0) / res).astype(int), 0, nx - 1)
    iy = np.clip(((xy[:, 1] - y0) / res).astype(int), 0, ny - 1)
    ok = dist[ix, iy] <= max_dist
    out = np.where(ok, grid[ix, iy], np.nan)
    return out


def write_pcd(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            stream.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


# --------------------------------------------------------------------------- #
# Scene assembly
# --------------------------------------------------------------------------- #

def traversability(planar_xy: np.ndarray, window, resolution: float,
                   robot_radius: float):
    """Occupancy, free-space labels and per-cell clearance at planning resolution."""
    x0, y0, size = window
    n = int(size / resolution)
    occ = np.zeros((n, n), bool)
    ix = ((planar_xy[:, 0] - x0) / resolution).astype(int)
    iy = ((planar_xy[:, 1] - y0) / resolution).astype(int)
    ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    occ[ix[ok], iy[ok]] = True
    k = int(round(robot_radius / resolution))
    inflated = ndimage.binary_dilation(occ, iterations=k) if k > 0 else occ
    free = ~inflated
    labels, count = ndimage.label(free, structure=np.ones((3, 3)))
    if count == 0:
        return occ, free, labels, np.zeros((n, n)), (x0, y0, resolution)
    sizes = ndimage.sum(free, labels, range(1, count + 1))
    largest = int(np.argmax(sizes)) + 1
    main = labels == largest
    # clearance measured from the inflated set: how far the body can wander
    clearance = ndimage.distance_transform_edt(free) * resolution
    return occ, main, labels, clearance, (x0, y0, resolution)


def pick_spawn(planar_xy: np.ndarray, window, floor_dist: np.ndarray,
               floor_meta, max_floor_dist: float, robot_radius: float
               ) -> tuple[float, float, float]:
    """Most central cell of the main traversable component, near the window centre.

    The spawn must survive the same inflation the planner will use; a wide alcove
    reached through a narrow passage is not a valid start even though its own
    clearance is large.
    """
    x0, y0, size = window
    resolution = 0.2
    _, main, _, clearance, _ = traversability(
        planar_xy, window, resolution, robot_radius)

    fx0, fy0, fres = floor_meta
    fnx, fny = floor_dist.shape

    def near_floor(wx: float, wy: float) -> bool:
        fi = int((wx - fx0) / fres)
        fj = int((wy - fy0) / fres)
        if not (0 <= fi < fnx and 0 <= fj < fny):
            return False
        return floor_dist[fi, fj] <= max_floor_dist

    n = main.shape[0]
    lo, hi = n // 3, 2 * n // 3
    best = (-1.0, None)
    for i in range(lo, hi):
        for j in range(lo, hi):
            if not main[i, j]:
                continue
            wx = x0 + (i + 0.5) * resolution
            wy = y0 + (j + 0.5) * resolution
            if not near_floor(wx, wy):
                continue
            if clearance[i, j] > best[0]:
                best = (clearance[i, j], (i, j))
    if best[1] is None:
        # fall back to any cell of the main component
        ys, xs = np.nonzero(main)
        if len(ys) == 0:
            return x0 + 0.5 * size, y0 + 0.5 * size, 0.0
        k = int(np.argmax(clearance[ys, xs]))
        i, j = int(ys[k]), int(xs[k])
    else:
        i, j = best[1]
    return (x0 + (i + 0.5) * resolution,
            y0 + (j + 0.5) * resolution,
            float(clearance[i, j]))


def write_world(path: Path, name: str, mesh_rel: str, window, z_lo: float, z_hi: float) -> None:
    x0, y0, size = window
    cx, cy = x0 + 0.5 * size, y0 + 0.5 * size
    zc = 0.5 * (z_lo + z_hi)
    zh = z_hi - z_lo
    walls = []
    specs = [
        (f"{cx}", f"{y0 - 0.25}", size + 0.5, 0.5),
        (f"{cx}", f"{y0 + size + 0.25}", size + 0.5, 0.5),
        (f"{x0 - 0.25}", f"{cy}", 0.5, size + 0.5),
        (f"{x0 + size + 0.25}", f"{cy}", 0.5, size + 0.5),
    ]
    for k, (px, py, sx, sy) in enumerate(specs):
        walls.append(f"""
    <model name='boundary_{k}'>
      <static>true</static>
      <pose>{px} {py} {zc:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>{sx} {sy} {zh:.3f}</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>{sx} {sy} {zh:.3f}</size></box></geometry>
          <material><ambient>0.3 0.3 0.8 0.4</ambient><diffuse>0.3 0.3 0.8 0.5</diffuse></material></visual>
      </link>
    </model>""")

    path.write_text(f"""<?xml version="1.0" ?>
<sdf version='1.6'>
  <world name='{name}'>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose>0 0 40 0 -0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.3 -0.5 -1</direction>
    </light>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.05 0.05 0.08 1</background><shadows>1</shadows></scene>
    <gravity>0 0 -9.8</gravity>
    <model name='{name}_geom'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><mesh><uri>{mesh_rel}</uri></mesh></geometry>
        </collision>
        <visual name='visual'>
          <geometry><mesh><uri>{mesh_rel}</uri></mesh></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>{''.join(walls)}
  </world>
</sdf>
""", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", required=True, type=Path,
                        help="source OBJ in the original (cm, rotated) frame")
    parser.add_argument("--name", required=True, help="scene name / world name")
    parser.add_argument("--window", nargs=3, required=True,
                        metavar=("X0", "Y0", "SIZE"), type=float,
                        help="world-frame XY crop window")
    parser.add_argument("--z-range", nargs=2, required=True, type=float,
                        metavar=("Z0", "Z1"), help="world-frame Z band to keep")
    parser.add_argument("--resolution", type=float, default=0.2,
                        help="PCD sampling spacing [m]")
    parser.add_argument("--waist", nargs=2, type=float, default=(0.08, 0.85),
                        metavar=("LO", "HI"),
                        help="obstacle height above LOCAL floor for scene_planar.pcd")
    parser.add_argument("--max-floor-dist", type=float, default=1.5,
                        help="max distance [m] a point may borrow a floor height from; "
                             "farther points are dropped instead of letting hole-filling "
                             "invent a floor across solid rock")
    parser.add_argument("--robot-radius", type=float, default=0.65,
                        help="inflation used for spawn selection and the reported "
                             "traversable area; mine passages may need 0.55")
    parser.add_argument("--spawn-height", type=float, default=0.3,
                        help="robot Z above the local floor at the spawn point")
    parser.add_argument("--no-center", action="store_true",
                        help="keep source XY instead of recentring the window on "
                             "the world origin (the Explorer grid needs centring)")
    parser.add_argument("--openings", type=Path,
                        help="JSON list of {\"x\":..,\"y\":..,\"half\":..} boxes in the "
                             "SOURCE XY frame; non-floor faces inside are deleted to "
                             "carve crosscuts and turn the tree of drifts into a cyclic "
                             "network. Floor faces are always preserved so the robot "
                             "cannot fall through.")
    parser.add_argument("--openings-keep-floor", action="store_true",
                        help="also delete floor faces inside the openings (off by default)")
    parser.add_argument("--floor-source-z", type=float,
                        help="world-frame Z reference that becomes 0 after the shift "
                             "(default: median floor height inside the window)")
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    x0, y0, size = args.window
    zlo, zhi = args.z_range
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    obj_v, faces = parse_obj(args.source)
    world = to_world(obj_v)
    tri = world[faces]
    cen = tri.mean(axis=1)
    nrm = face_normals(tri)

    inside_xy = ((cen[:, 0] >= x0) & (cen[:, 0] < x0 + size)
                 & (cen[:, 1] >= y0) & (cen[:, 1] < y0 + size))
    zmin = tri[:, :, 2].min(axis=1)
    zmax = tri[:, :, 2].max(axis=1)
    inside_z = (zmax >= zlo) & (zmin <= zhi)
    keep = inside_xy & inside_z
    if keep.sum() < 100:
        raise SystemExit(f"crop kept only {int(keep.sum())} faces; check window/band")

    removed_by_openings = 0
    if args.openings:
        specs = json.loads(args.openings.read_text(encoding="utf-8"))
        for spec in specs:
            hx = float(spec["half"])
            box = ((np.abs(cen[:, 0] - float(spec["x"])) <= hx)
                   & (np.abs(cen[:, 1] - float(spec["y"])) <= hx))
            cut = box if args.openings_keep_floor else box & (nrm[:, 2] <= 0.6)
            removed_by_openings += int((keep & cut).sum())
            keep &= ~cut

    kept = tri[keep]
    kept_n = nrm[keep]
    floor_sel = kept_n[:, 2] > 0.6
    if not floor_sel.any():
        raise SystemExit("no floor faces inside the crop")
    floor_tri = kept[floor_sel]
    floor_grid, floor_dist, grid_meta = local_floor_height(floor_tri)

    # Work in the source frame while choosing the spawn: relative height above
    # the local floor and 2-D geometry are both shift-invariant, so the Z anchor
    # can be decided last and anchored on the spawn the robot actually uses.
    surf = sample_triangles(kept, args.resolution)
    fh = floor_lookup(floor_grid, floor_dist, grid_meta, surf[:, :2],
                      args.max_floor_dist)
    rel = surf[:, 2] - fh
    has_floor = ~np.isnan(fh)
    planar = surf[has_floor & (rel >= args.waist[0]) & (rel <= args.waist[1])]
    if len(planar) == 0:
        raise SystemExit("planar PCD is empty; widen --waist or --max-floor-dist")

    sx, sy, spawn_clearance = pick_spawn(planar, (x0, y0, size), floor_dist,
                                         grid_meta, args.max_floor_dist,
                                         args.robot_radius)
    spawn_floor = float(floor_lookup(floor_grid, floor_dist, grid_meta,
                                     np.array([[sx, sy]]),
                                     args.max_floor_dist)[0])

    # anchor Z so the spawn floor becomes 0 unless the caller names a reference
    shift = (-args.floor_source_z if args.floor_source_z is not None
             else -spawn_floor)
    # The Explorer builds ExplorationGrid(size, size, res) with no origin, i.e.
    # hard-centred on the world origin, so recentre the window by default.
    if args.no_center:
        cx, cy = 0.0, 0.0
    else:
        cx, cy = x0 + 0.5 * size, y0 + 0.5 * size

    verts_shift = world.copy()
    verts_shift[:, 0] -= cx
    verts_shift[:, 1] -= cy
    verts_shift[:, 2] += shift
    kept_shift = kept.copy()
    kept_shift[:, :, 0] -= cx
    kept_shift[:, :, 1] -= cy
    kept_shift[:, :, 2] += shift
    surf[:, 0] -= cx
    surf[:, 1] -= cy
    surf[:, 2] += shift
    planar[:, 0] -= cx
    planar[:, 1] -= cy
    planar[:, 2] += shift
    spawn = (sx - cx, sy - cy, spawn_floor + shift + args.spawn_height)

    write_obj(out / "scene.obj", verts_shift, faces[keep])
    write_pcd(out / "scene_surface.pcd", surf)
    write_pcd(out / "scene_planar.pcd", planar)

    _, main, _, _, _ = traversability(planar, (x0 - cx, y0 - cy, size), 0.2,
                                      args.robot_radius)
    mesh_rel = f"model://cropped_scenes/{args.name}/scene.obj"
    write_world(out / "scene.world", args.name, mesh_rel,
                (x0 - cx, y0 - cy, size),
                float(kept_shift[:, :, 2].min()), float(kept_shift[:, :, 2].max()))

    zb = kept_shift[:, :, 2]
    meta = {
        "source": str(args.source),
        "window": [x0, y0, size],
        "window_centred_xy_shift": [round(cx, 4), round(cy, 4)],
        "z_range_source_frame": [zlo, zhi],
        "floor_shift_z": round(shift, 4),
        "resolution": args.resolution,
        "waist_band_above_local_floor": list(args.waist),
        "max_floor_borrow_dist": args.max_floor_dist,
        "faces_source": int(len(faces)),
        "faces_kept": int(keep.sum()),
        "faces_removed_by_openings": removed_by_openings,
        "floor_faces_kept": int(floor_sel.sum()),
        "surface_pcd_points": int(len(surf)),
        "planar_pcd_points": int(len(planar)),
        "robot_radius_used": args.robot_radius,
        "traversable_area_m2": round(float(main.sum()) * 0.04, 1),
        "spawn_clearance_m": round(float(spawn_clearance), 3),
        "mesh_z_after_shift": [round(float(zb.min()), 3), round(float(zb.max()), 3)],
        "spawn": [round(float(spawn[0]), 3), round(float(spawn[1]), 3),
                  round(float(spawn[2]), 3)],
    }
    (out / "scene_meta.json").write_text(
        json.dumps(meta, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(meta, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
