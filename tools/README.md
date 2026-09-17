# Keypoint recorder (ROS 2)

Build and source the workspace, then run:

```bash
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
```

The generated file is a ROS 2 parameter file containing the flat
`fsm.waypoints: [x0, y0, z0, ...]` array. Pass it to `run.launch.py` with
`keypoints_file:=/absolute/path/to/keypoints.yaml` when `navi_mode:=2`.

## Open-RMF core-map generator

Regenerate the wall-only Airport Terminal PCD from the attributed Open-RMF
Traffic Editor source with:

```bash
python3 tools/generate_rmf_building_core_pcd.py \
  --building src/planner/plan_manage/maps/airport_terminal_source/airport_terminal.building.yaml \
  --output src/planner/plan_manage/maps/airport_terminal_core.pcd
```

The converter reads the level's measurement scale, centres the building at the
ROS world origin, flips image Y into ROS +Y, and samples only structural walls.
Furniture and people are deliberately excluded from the core map so their
unknown collision dimensions cannot create false blocked corridors.

The same converter generates Clinic L1 without projecting L2 into it:

```bash
python3 tools/generate_rmf_building_core_pcd.py \
  --building src/planner/plan_manage/maps/clinic_source/clinic.building.yaml \
  --level L1 \
  --spacing 0.20 \
  --output src/planner/plan_manage/maps/clinic_L1_core.pcd
```

## RoboTerrain Inspection planar baseline

The long-range Inspection baseline is derived from the upstream COLLADA
collision mesh. The current kinematic simulator has constant body height, so
the converter projects steep faces into a closed planar obstacle map while
retaining the original DAE for a later Gazebo terrain benchmark:

```bash
python3 tools/generate_collada_pcd.py --mode planar \
  --dae src/planner/plan_manage/maps/roboterrain_inspection_source/inspection_world.dae \
  --centre-x 1.062655 --centre-y 18.165435 \
  --resolution 0.20 --vertical-spacing 0.20 \
  --output src/planner/plan_manage/maps/roboterrain_inspection_planar.pcd
```

## SubT mine cropper

`tools/crop_subt_mine.py` cuts a window out of a SubT mine OBJ and emits the
same five artefacts the other scenes use. It exists because the scripts that
produced `mine_edgar`, `mine_mbplanner`, `tunnel_trim`, `cave_trim` and
`urban_trim` are no longer in the repository — only their products and metadata
survived.

It handles the two things a flat-scene converter gets wrong on real mines:

- **Sloped floor.** `scene_planar.pcd` is the obstacle surface whose height
  above the *local* floor is inside the robot waist band (default 0.08..0.85 m),
  not an absolute Z slice. A per-XY floor height model is built from the
  near-horizontal faces, and points farther than `--max-floor-dist` from any
  real floor sample are dropped rather than measured against a floor borrowed
  from the far side of a wall.
- **Ceilings.** Only walls with |n_z| < 0.6 count as obstacles; ceiling face
  centroids would otherwise cover the whole tunnel footprint and seal every
  passage.

It also bakes the Edgar authoring transform (centimetres, 90 deg roll), anchors
Z so the spawn floor is 0, and recentres the window on the world origin because
`ExplorationGrid` has no origin parameter and is hard-centred.

```bash
python3 tools/crop_subt_mine.py \
  --source ~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models/edgar/3/meshes/edgar.obj \
  --name mine_edgar_200 --window -195 -13 200 --z-range -31 -4 \
  --resolution 0.2 --robot-radius 0.55 \
  --output-dir src/planner/plan_manage/maps/cropped_scenes/mine_edgar_200
```

See `maps/cropped_scenes/mine_edgar_200/README.md` for the scene this produced
and the Explorer/SCAN parameter overrides narrow mine passages require.

