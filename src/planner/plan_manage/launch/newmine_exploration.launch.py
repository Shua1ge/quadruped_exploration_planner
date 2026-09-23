"""Long-horizon exploration in the native NewMine Gazebo world."""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


MAP_SIZE = 280.0
# Gazebo loads the original world coordinate system.  The staging platform and
# cave entrance are authored around the NewMine world origin.
DEFAULT_INIT_X = 0.0
DEFAULT_INIT_Y = 0.0


def _reject_existing_simulation(context):
    """Validate the selected mode and reject stale Gazebo processes."""
    enabled = lambda name: LaunchConfiguration(name).perform(context).lower() in (
        "1", "true", "yes", "on")
    use_gazebo_physics = enabled("use_gazebo_physics")
    use_sdf_map = enabled("use_sdf_map")
    if enabled("use_pcd_map"):
        raise RuntimeError(
            "NewMine exploration does not accept a PCD truth map; use_pcd_map must be false")
    if use_gazebo_physics and use_sdf_map:
        raise RuntimeError(
            "Gazebo physics and private SDF raycasting are mutually exclusive")
    if not use_gazebo_physics and not use_sdf_map:
        raise RuntimeError("Lightweight exploration requires use_sdf_map=true")
    if not use_gazebo_physics:
        return []

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    ros_context = rclpy.Context()
    # Launch arguments (for example ``headless:=true``) are not ROS remaps.
    # Do not let rclpy parse the parent launch process arguments here.
    rclpy.init(args=[], context=ros_context)
    node = rclpy.create_node(
        f"newmine_launch_preflight_{os.getpid()}", context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        clock_publishers = node.count_publishers("/clock")
        graph_nodes = {name for name, _namespace in node.get_node_names_and_namespaces()}
        stale_sim_nodes = sorted(
            graph_nodes.intersection({
                "go2_gz_bridge",
                "gz_ros2_control",
                "controller_manager",
            }))
        if clock_publishers or stale_sim_nodes:
            details = []
            if clock_publishers:
                details.append(f"/clock has {clock_publishers} publisher(s)")
            if stale_sim_nodes:
                details.append("existing simulation nodes: " + ", ".join(stale_sim_nodes))
            raise RuntimeError(
                "NewMine launch refused: " + "; ".join(details) + ". Stop the "
                "previous Gazebo/ROS launch completely before starting a new run. "
                "A stale Gazebo server can retain the old Go2 even after its clock "
                "bridge exits, causing go2/go2_0 and duplicate controllers.")
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)
    return []


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world_file",
            default_value="/home/t1an/ros2_ws/mine_tunnel_world/worlds/NewMine.sdf"),
        DeclareLaunchArgument("use_gpu", default_value="true"),
        DeclareLaunchArgument("use_gazebo_physics", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_pcd_map", default_value="false"),
        DeclareLaunchArgument("use_sdf_map", default_value="false"),
        DeclareLaunchArgument("sdf_world_file", default_value=""),
        DeclareLaunchArgument("sdf_map_mode", default_value="surface"),
        DeclareLaunchArgument("sdf_sample_resolution", default_value="0.20"),
        DeclareLaunchArgument("sdf_recenter", default_value="false"),
        DeclareLaunchArgument("publish_sdf_global_cloud", default_value="false"),
        DeclareLaunchArgument("collision_check_enable", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("selection_strategy", default_value="hierarchical"),
        DeclareLaunchArgument("locomotion_mode", default_value="planning_kinematic"),
        # "policy" lets the zero-command ONNX policy perform the lying-to-stand;
        # "deterministic" keeps the time-scheduled PREPOSE/STAND interpolation.
        DeclareLaunchArgument("getup_mode", default_value="policy"),
        DeclareLaunchArgument(
            "metrics_file", default_value="/tmp/newmine_long_horizon_01.csv"),
        DeclareLaunchArgument("init_x", default_value=str(DEFAULT_INIT_X)),
        DeclareLaunchArgument("init_y", default_value=str(DEFAULT_INIT_Y)),
        DeclareLaunchArgument("init_z", default_value="0.45"),
        OpaqueFunction(function=_reject_existing_simulation),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_share, "launch", "run.launch.py")),
            launch_arguments={
                "navi_mode": "3",
                "sensor_type": "lidar",
                "controller_mode": "closed_loop",
                "locomotion_mode": LaunchConfiguration("locomotion_mode"),
                "getup_mode": LaunchConfiguration("getup_mode"),
                "use_gpu": LaunchConfiguration("use_gpu"),
                "use_pcd_map": LaunchConfiguration("use_pcd_map"),
                "use_sdf_map": LaunchConfiguration("use_sdf_map"),
                "sdf_world_file": LaunchConfiguration("sdf_world_file"),
                "sdf_map_mode": LaunchConfiguration("sdf_map_mode"),
                "sdf_sample_resolution": LaunchConfiguration("sdf_sample_resolution"),
                "sdf_recenter": LaunchConfiguration("sdf_recenter"),
                "map_size_x": str(MAP_SIZE),
                "map_size_y": str(MAP_SIZE),
                "map_size_z": "10.0",
                "init_x": LaunchConfiguration("init_x"),
                "init_y": LaunchConfiguration("init_y"),
                "init_z": LaunchConfiguration("init_z"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_gazebo_physics": LaunchConfiguration("use_gazebo_physics"),
                "gazebo_world": LaunchConfiguration("world_file"),
                "gazebo_resource_path": "/home/t1an/ros2_ws/mine_tunnel_world/models",
                "headless": LaunchConfiguration("headless"),
                "collision_check_enable": LaunchConfiguration("collision_check_enable"),
            }.items()),
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
                "map_size_x": MAP_SIZE,
                "map_size_y": MAP_SIZE,
                "mapping_range": 7.5,
                "no_return_range": 3.0,
                "ray_count": 720,
                "hit_dilation_bins": 3,
                "obstacle_min_z": 0.08,
                "obstacle_max_z": 0.85,
                "obstacle_z_relative_to_body": True,
                "cloud_is_world": PythonExpression([
                    "'", LaunchConfiguration("use_gazebo_physics"),
                    "'.lower() != 'true'",
                ]),
                # Shared with run.launch.py / GridMap and robot.xacro.
                "lidar_extrinsic_x": 0.10,
                "lidar_extrinsic_y": 0.0,
                "lidar_extrinsic_z": 0.12,
                # NewMine contains 2.9--3.0 m portals and short blind bends.
                # Keep the physical margin conservative for a Go2 footprint,
                # but allow rolling observation poses inside the portal.  The
                # flat-world 0.65/1.0/2.0 defaults otherwise extract a valid
                # frontier here and then reject every possible viewpoint.
                # Must not drop below grid_map.double_cylinder_radius (0.35, see
                # planner.yaml): an explorer that inflates less than the planner
                # would select goals the planner then rejects as collisions.
                "inflation_radius": 0.35,
                "footprint_radius": 0.35,
                "footprint_offset": 0.18,
                "viewpoint_standoff": 0.60,
                # Extra annulus width, tried only when the nominal stand-off
                # yields no pose at all.  Discarding the frontier instead is
                # what stalls exploration inside the 2.8 m tunnels.
                "viewpoint_relaxation": 1.0,
                # At 0.20 m resolution the footprint inflation can
                # leave a real narrow-tunnel frontier only two cells wide.  A
                # threshold of four erased the sole forward frontier while
                # coverage was still 0.21%.  Keep singleton noise excluded;
                # reachability, clearance, gain, and final path validation
                # still guard every two-cell candidate.
                "min_frontier_size": 2,
                "min_goal_distance": 0.80,
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
                "region_unreachable_timeout": 5.0,
                "max_global_regions": 12,
                "metrics_period": 2.0,
                "observation_preplanning_enabled": True,
                "observation_radius": 3.0,
                "observation_prepare_ratio": 0.60,
                "observation_done_ratio": 0.80,
                "observation_done_updates": 3,
                "observation_closure_updates": 10,
                "min_expected_observation_cells": 4,
                "planning_period": 0.25,
                "frame_id": "world",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[
                ("cloud", "/quad_0/cloud"),
                ("body_pose", "/quad_0/body_pose"),
                ("initial_path", "/initial_path"),
                ("planning/status", "/planning/status"),
                ("planning/local_execution_event", "/planning/local_execution_event"),
                ("simulation/collision", "/simulation/collision"),
            ]),
        Node(
            package="scan_planner",
            executable="global_representation_node.py",
            name="global_representation",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_oracle_queries": 8,
                "snapshot_period_revisions": 10,
                "blocked_confirmation_patches": 3,
                "min_free_cells_for_destructive_update": 16,
            }]),
        Node(
            package="scan_planner",
            executable="sdf_map_publisher.py",
            name="sdf_map_publisher",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_gazebo_physics"),
                "'.lower() == 'false' and '",
                LaunchConfiguration("use_sdf_map"),
                "'.lower() == 'true' and '",
                LaunchConfiguration("publish_sdf_global_cloud"),
                "'.lower() == 'true'",
            ])),
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "world_file": LaunchConfiguration("sdf_world_file"),
                "frame_id": "world",
                "mode": LaunchConfiguration("sdf_map_mode"),
                "resolution": LaunchConfiguration("sdf_sample_resolution"),
                "recenter": LaunchConfiguration("sdf_recenter"),
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_share, "launch", "rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("show_rviz")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items()),
    ])
