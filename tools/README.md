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
