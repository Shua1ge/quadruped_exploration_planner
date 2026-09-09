"""Autonomous stage-one frontier exploration in the flat quad maze."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    quad_pcd = os.path.join(scan_share, "maps", "quad_flat.pcd")
    return LaunchDescription([
        DeclareLaunchArgument("use_gpu", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("selection_strategy", default_value="hierarchical"),
        DeclareLaunchArgument(
            "metrics_file", default_value="/tmp/quad_exploration_adaptive_metrics.csv"),
        DeclareLaunchArgument("init_x", default_value="-24.0"),
        DeclareLaunchArgument("init_y", default_value="10.0"),
        DeclareLaunchArgument("init_z", default_value="0.3"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scan_share, "launch", "run.launch.py")),
            launch_arguments={
                "navi_mode": "3",
                "sensor_type": "lidar",
                "controller_mode": "closed_loop",
                "use_gpu": LaunchConfiguration("use_gpu"),
                "use_pcd_map": "true",
                "pcd_map_file": quad_pcd,
                "map_size_x": "64.0",
                "map_size_y": "40.0",
                "map_size_z": "5.0",
                "init_x": LaunchConfiguration("init_x"),
                "init_y": LaunchConfiguration("init_y"),
                "init_z": LaunchConfiguration("init_z"),
                "use_sim_time": "false",
            }.items(),
        ),
        Node(
            package="scan_planner",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            output="screen",
            parameters=[{
                "auto_start": LaunchConfiguration("auto_start"),
                "selection_strategy": LaunchConfiguration("selection_strategy"),
                "metrics_file": LaunchConfiguration("metrics_file"),
                "resolution": 0.20,
                "map_size_x": 64.0,
                "map_size_y": 40.0,
                "mapping_range": 7.5,
                "no_return_range": 3.0,
                "ray_count": 720,
                "hit_dilation_bins": 3,
                "obstacle_min_z": 0.08,
                "obstacle_max_z": 0.85,
                # The controller already checks the GO2 body collision shape;
                # retain mapping/tracking margin without double-padding it.
                "inflation_radius": 0.65,
                "viewpoint_standoff": 1.0,
                "min_frontier_size": 6,
                "min_goal_distance": 2.0,
                "preferred_goal_path_length": 3.5,
                "long_horizon_min_gain_ratio": 0.65,
                "blacklist_radius": 1.5,
                "global_reroute_failure_updates": 3,
                "max_replans_per_context": 2,
                "replan_retry_updates": 10,
                "map_update_period": 0.5,
                "goal_timeout": 120.0,
                "region_size": 8.0,
                "region_max_path_ratio": 1.5,
                "region_max_detour_ratio": 1.75,
                "region_match_distance": 6.0,
                "sparse_routing_enabled": True,
                "sparse_attachment_radius": 1.5,
                "sparse_attachment_limit": 8,
                "region_release_updates": 3,
                # Keep brief perception gaps from causing region thrashing,
                # but never wait forever when no usable viewpoint remains.
                "region_unreachable_timeout": 5.0,
                "max_global_regions": 10,
                "metrics_period": 2.0,
                "observation_preplanning_enabled": True,
                # The viewpoint guides motion; information gain completes the
                # task before pose arrival when enough target cells are seen.
                "observation_radius": 3.0,
                "observation_prepare_ratio": 0.60,
                "observation_done_ratio": 0.80,
                "observation_done_updates": 3,
                "min_expected_observation_cells": 8,
                "planning_period": 0.25,
                "frame_id": "world",
            }],
            remappings=[
                ("cloud", "/quad_0/cloud"),
                ("body_pose", "/quad_0/body_pose"),
                ("initial_path", "/initial_path"),
                ("planning/status", "/planning/status"),
                ("simulation/collision", "/simulation/collision"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="global_representation_node.py",
            name="global_representation",
            output="screen",
            parameters=[{
                "max_oracle_queries": 8,
                "snapshot_period_revisions": 10,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scan_share, "launch", "rviz.launch.py"))),
    ])
