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
