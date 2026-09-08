# Flat Quad Maze benchmark

This benchmark adapts the `height_maze/quad.world` collision layout from
[`engcang/gazebo_maps`](https://github.com/engcang/gazebo_maps), distributed
under BSD-3-Clause.  The original layout remains attributed to its authors;
the generated PCD contains wall surfaces and conservative circular tree
obstacles only, not Gazebo visual meshes or ground texture.

The test intentionally uses the complete ground-truth obstacle cloud.  RViz
goals go to `global_astar_planner.py`, which rejects occupied or disconnected
goals and publishes a collision-free `nav_msgs/Path` on `/initial_path`.
The global grid uses a conservative `0.65 m` inflation radius, covering SCAN's
offset double-cylinder body model plus voxel and tracking margin. SCAN-Planner
runs with `navi_mode=3` and smooths that reference path into local
B-spline trajectories.  SCAN's pre-publication collision validation remains the
final guard before the controller receives a trajectory.

Start with:

```bash
source /opt/ros/humble/setup.bash
source /home/t1an/ros2_ws/scan_planner_ws/install/setup.bash
ros2 launch scan_planner quad_benchmark.launch.py use_gpu:=true
```

Use RViz `2D Goal Pose`.  Watch `/global_planning/status` for global planning
results and `/planning/status` for SCAN execution state.  `GOAL_OCCUPIED` and
`NO_PATH` do not publish a replacement reference path.

Suggested fixed cases (verify coordinates visually before recording results):

| Case | Start | Goal | Purpose |
|---|---:|---:|---|
| Q01 | `(-24, 10)` | `(24, 10)` | Long cross-maze route |
| Q02 | `(-24, 10)` | `(24, -10)` | Multiple turns and branches |
| Q03 | `(-24, 10)` | `(-4, -12)` | Central obstacle cluster |
| Q04 | `(-24, 10)` | click on a wall | Required `GOAL_OCCUPIED` rejection |

For each accepted case, record global path length, planning time, SCAN replan
count, minimum inflated-obstacle clearance, collision count, and completion.

## Open-RMF Airport Terminal core map

`airport_terminal_core.pcd` is a wall-only conversion of the Open-RMF Airport
Terminal L1 scene. It is intentionally separate from the quad benchmark: the
airport spans about `282.1 x 83.4 m`, while the Explorer and SCAN data flow is
unchanged and still reveals the truth map only through local simulated LiDAR.

Build and launch it with:

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner --symlink-install
source install/setup.bash
ros2 launch scan_planner airport_terminal_exploration.launch.py
```

The default start is RMF navigation vertex `junction_n20`, transformed to
approximately `(8.54, 11.97, 0.30)` in the centred ROS world. Keep the original
quad launch for regression comparisons. Source attribution and regeneration
details are in `airport_terminal_source/README.md` and `tools/README.md`.

## Open-RMF Clinic L1 core map

`clinic_L1_core.pcd` contains only the structural walls of Clinic level L1.
Level L2 is deliberately excluded so its walls cannot overlap the Explorer's
two-dimensional occupancy grid. The centred L1 source extent is approximately
`52.80 x 46.45 m`; the launch reserves a `58 x 52 m` map with edge margin.

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner --symlink-install
source install/setup.bash
ros2 launch scan_planner clinic_l1_exploration.launch.py
```

The default start is RMF navigation vertex `L1_right_nurse_center`, transformed
to approximately `(11.69, 8.50, 0.30)` in the centred ROS world. Regeneration
and source attribution are documented in `tools/README.md` and
`clinic_source/README.md`.
