# RoboTerrain Inspection source

`inspection_world.dae` is copied from
[jackvice/RoboTerrain](https://github.com/jackvice/RoboTerrain), package path
`ros2_ws/src/roverrobotics_ros2/roverrobotics_gazebo/meshes/inspection_world.dae`.
The upstream repository identifies the project license as Apache-2.0; retain
the upstream copyright and license notices when redistributing this asset.

The mesh bounds before centring are approximately:

```text
x: -31.15458 .. 33.27989 m
y: -31.64693 .. 67.97780 m
z:  -2.98810 ..  7.45629 m
```

`../roboterrain_inspection_planar.pcd` is the deterministic 2-D planning
derivative. It is generated from steep collision faces at 0.20 m resolution,
centred with `(x, y) = (1.062655, 18.165435)`, extruded to 2.5 m, and enclosed
by a rectangular guard boundary. It is used by the default constant-height
kinematic benchmark.

`../roboterrain_inspection_surface.pcd` samples the complete XYZ mesh surface
at 0.20 m spacing. The Gazebo physics benchmark uses this cloud for simulated
LiDAR rendering while `worlds/inspection.world` loads the original COLLADA
mesh for contact and visuals. The two representations use the same centring
translation, so rendered obstacles and physical collisions remain aligned.

Regenerate from the repository root:

```bash
python3 tools/generate_collada_pcd.py --mode planar \
  --dae src/planner/plan_manage/maps/roboterrain_inspection_source/inspection_world.dae \
  --centre-x 1.062655 --centre-y 18.165435 \
  --resolution 0.20 --vertical-spacing 0.20 \
  --output src/planner/plan_manage/maps/roboterrain_inspection_planar.pcd

python3 tools/generate_collada_pcd.py --mode surface \
  --dae src/planner/plan_manage/maps/roboterrain_inspection_source/inspection_world.dae \
  --centre-x 1.062655 --centre-y 18.165435 \
  --resolution 0.20 \
  --output src/planner/plan_manage/maps/roboterrain_inspection_surface.pcd
```

Launch the deterministic baseline with `physics:=false`, or the original
terrain mesh with Gazebo contacts and terrain-relative obstacle extraction:

```bash
ros2 launch scan_planner roboterrain_inspection_exploration.launch.py physics:=false
ros2 launch scan_planner roboterrain_inspection_exploration.launch.py physics:=true
```

The physics mode uses Gazebo's model velocity interface as a planning benchmark
adapter. Gravity and mesh contact are physical, but this is not a force-level
quadruped gait controller and must not be reported as one.
