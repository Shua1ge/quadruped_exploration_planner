# Quadruped Exploration Planner

A hierarchical exploration and navigation framework for quadruped robots in large-scale unknown environments.

This project integrates **global exploration planning**, **topological environment representation**, **local map fusion**, and **collision-aware local trajectory planning** to enable autonomous navigation in complex scenarios.

## Overview

The system adopts a hierarchical architecture:

```
Sensors
   |
   v
Local Map Fusion
   |
   +----------------+
   |                |
   v                v
Global Topology   Local Planner
(Graph-based)     (SCAN-based)
   |                |
   +----------------+
            |
            v
     Quadruped Control
```

## Main Features

### Global Exploration

* Frontier-based exploration
* Sparse topological graph construction
* Long-range route planning
* Region-level exploration management

### Local Navigation

* Dense local map fusion
* Collision-aware trajectory generation
* A* path correction
* B-spline trajectory optimization

### ROS 2 Support

* ROS 2 Humble compatible
* Simulation and real robot deployment support
* Designed for quadruped platforms

## Package Structure

```
src/

planner/
 ├── plan_manage/        # Exploration and planning logic
 ├── plan_env/           # Local map and environment representation
 ├── path_searching/     # Path search module
 ├── bspline_opt/        # Trajectory optimization
 └── scan_planner_msgs/  # ROS2 messages

simulator/
 └── Simulation environments
```

## Installation

Requirements:

* Ubuntu 22.04
* ROS 2 Humble

Build:

```bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

## Future Development

This project is being extended toward efficient long-horizon quadruped navigation, including:

* Long-term spatial memory
* Learning-based local planning
* Efficient topology-aware navigation
* Embedded deployment

## Acknowledgement

This project is based on the open-source SCAN-Planner framework and extends it with exploration and hierarchical planning components.
