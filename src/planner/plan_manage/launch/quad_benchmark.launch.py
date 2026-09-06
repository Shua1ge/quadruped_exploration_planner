"""Run SCAN-Planner on the deterministic flat quad maze benchmark."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    run_launch = os.path.join(scan_share, "launch", "run.launch.py")
    rviz_launch = os.path.join(scan_share, "launch", "rviz.launch.py")
    quad_pcd = os.path.join(scan_share, "maps", "quad_flat.pcd")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gpu", default_value="true"),
            DeclareLaunchArgument("init_x", default_value="-24.0"),
            DeclareLaunchArgument("init_y", default_value="10.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(run_launch),
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
                executable="global_astar_planner.py",
                name="global_astar_planner",
                output="screen",
                parameters=[{
                    "resolution": 0.20,
                    # Covers SCAN's 0.35 m double-cylinder radius, its 0.18 m
                    # fore/aft offset, voxel rounding, and modest tracking error.
                    "inflation_radius": 0.65,
                    "obstacle_min_z": 0.08,
                    "obstacle_max_z": 0.85,
                    "map_padding": 1.0,
                    "path_spacing": 0.50,
                    "frame_id": "world",
                }],
                remappings=[
                    ("global_map", "/map_generator/global_cloud"),
                    ("body_pose", "/quad_0/body_pose"),
                    ("goal", "/move_base_simple/goal"),
                    ("initial_path", "/initial_path"),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
            ),
        ]
    )
