"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_gazebo_physics = _as_bool(
        LaunchConfiguration("use_gazebo_physics").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    locomotion_mode = LaunchConfiguration("locomotion_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if locomotion_mode not in ("planning_kinematic", "rl_effort"):
        raise RuntimeError("locomotion_mode must be 'planning_kinematic' or 'rl_effort'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if use_gazebo_physics:
        if is_real:
            raise RuntimeError("use_gazebo_physics cannot be combined with is_real_world")
        if controller_mode != "closed_loop":
            raise RuntimeError("use_gazebo_physics requires controller_mode=closed_loop")
        if not use_sim_time:
            raise RuntimeError("use_gazebo_physics requires use_sim_time=true")
        gazebo_world = LaunchConfiguration("gazebo_world").perform(context)
        if not gazebo_world or not os.path.isfile(gazebo_world):
            raise RuntimeError(
                "use_gazebo_physics=true requires gazebo_world to reference an existing file"
            )

    lidar_extrinsic = {}
    if is_real:
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        if use_gazebo_physics and sensor_type == "lidar":
            # Gazebo publishes the lidar cloud in the robot-mounted sensor
            # frame.  The lidar is rigidly mounted close to the trunk origin,
            # so the bridged model odometry supplies the ray origin and
            # orientation used to project it into world coordinates.
            sensor_pose = body_pose
            need_extrinsic = True
            lidar_extrinsic = {
                "grid_map.lidar_extrinsic_x": 0.10,
                "grid_map.lidar_extrinsic_y": 0.0,
                "grid_map.lidar_extrinsic_z": 0.12,
                "grid_map.lidar_extrinsic_roll": 0.0,
                "grid_map.lidar_extrinsic_pitch": 0.0,
                "grid_map.lidar_extrinsic_yaw": 0.0,
            }
        else:
            sensor_pose = ("/quad_0/camera_pose" if sensor_type == "depth"
                           else "/quad_0/lidar_pose")
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = not use_gazebo_physics
        if not (use_gazebo_physics and sensor_type == "lidar"):
            need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
        **lidar_extrinsic,
    }
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    ]
    if not use_gazebo_physics:
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                         " use_gazebo:=false"]
                    )
                },
            ],
        ))

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/planning/cmd_vel_raw"),
                ],
            )
        )
        if not is_real and not use_gazebo_physics:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        if use_gazebo_physics:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(go2_share, "launch", "go2_sim.launch.py")
                    ),
                    launch_arguments={
                        "world": LaunchConfiguration("gazebo_world"),
                        "resource_path": LaunchConfiguration("gazebo_resource_path"),
                        "terrain_velocity_control": (
                            "true" if locomotion_mode == "planning_kinematic" else "false"),
                        "locomotion_mode": locomotion_mode,
                        "headless": LaunchConfiguration("headless"),
                        "x": LaunchConfiguration("init_x"),
                        "y": LaunchConfiguration("init_y"),
                        "z": LaunchConfiguration("init_z"),
                    }.items(),
                )
            )
            # Add planar cmd_vel adapter for Gazebo physics
            # Extract world name from SDF file path
            gazebo_world_path = LaunchConfiguration("gazebo_world").perform(context)
            world_name = "virtual_stix_geometry"  # NewMine test.sdf default
            if "empty" in gazebo_world_path or "apply_link_wrench" in gazebo_world_path:
                world_name = "apply_link_wrench"
            
            # Add TF bridge to connect Go2 TF tree for RViz
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_tf_bridge.py",
                    name="go2_tf_bridge",
                    output="screen",
                    parameters=[common],
                )
            )
            
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_locomotion_supervisor.py",
                    name="go2_locomotion_supervisor",
                    output="screen",
                    parameters=[
                        common,
                        {
                            "locomotion_mode": locomotion_mode,
                            "getup_mode": LaunchConfiguration("getup_mode"),
                            "model_path": "/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx",
                        },
                    ],
                    remappings=[("body_pose", body_pose)],
                )
            )
            actions.append(
                Node(
                    package="odom_visualization",
                    executable="odom_visualization",
                    name="odom_visualization",
                    output="screen",
                    parameters=[common, {
                        "frame_id": "world",
                        "child_frame_id": "go2/base_footprint",
                        "publish_tf": False,
                    }],
                    remappings=[
                        ("body_pose", body_pose),
                        ("pose", "/quad_0/pose"),
                        ("path", "/quad_0/path"),
                        ("velocity", "/quad_0/velocity"),
                        ("trajectory", "/quad_0/trajectory"),
                        ("robot", "/quad_0/robot"),
                        ("height", "/quad_0/height"),
                    ],
                )
            )
            if _as_bool(LaunchConfiguration("wrench_actuator").perform(context)):
                actions.append(
                    Node(
                        package="scan_planner",
                        executable="planar_cmd_vel_adapter",
                        name="planar_cmd_vel_adapter",
                        output="screen",
                        parameters=[
                            common,
                            {
                                "world_name": world_name,
                                "model_name": "go2",
                                "link_name": "base",
                                "cmd_timeout": 0.5,
                                "kp_linear": 150.0,
                                "kd_linear": 30.0,
                                "kp_angular": 20.0,
                                "kd_angular": 5.0,
                                "max_force": 200.0,
                                "max_torque": 30.0,
                            },
                        ],
                        remappings=[
                            ("cmd_vel", "/quad_0/cmd_vel"),
                            ("body_pose", body_pose),
                        ],
                    )
                )
            
            # Leg animation disabled: go2_gait_publisher publishes JointState
            # (sensor feedback) but Gazebo expects JointTrajectory (commands).
            # VelocityControl provides stable base motion without leg dynamics.
            # To enable proper leg control, replace VelocityControl with
            # effort controllers + go2_policy_node (RL-based locomotion).
        else:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                )
            )
        if not use_gazebo_physics:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "use_sdf_map",
                            "sdf_world_file",
                            "sdf_map_mode",
                            "sdf_sample_resolution",
                            "sdf_recenter",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                            "collision_check_enable",
                            "lidar_pitch",
                        )
                    }.items(),
                )
            )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("locomotion_mode", default_value="planning_kinematic"),
            DeclareLaunchArgument("getup_mode", default_value="policy"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("use_sdf_map", default_value="false"),
            DeclareLaunchArgument("sdf_world_file", default_value=""),
            DeclareLaunchArgument("sdf_map_mode", default_value="planar"),
            DeclareLaunchArgument("sdf_sample_resolution", default_value="0.20"),
            DeclareLaunchArgument("sdf_recenter", default_value="true"),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_gazebo_physics", default_value="false"),
            DeclareLaunchArgument("wrench_actuator", default_value="false"),
            DeclareLaunchArgument("gazebo_world", default_value=""),
            DeclareLaunchArgument("gazebo_resource_path", default_value=""),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("collision_check_enable", default_value="true"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
