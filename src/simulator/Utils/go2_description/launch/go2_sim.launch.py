"""Spawn Go2 in Gazebo Fortress with gz_ros2_control."""

import os
import shlex
import time
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    Shutdown,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _world_name(world_file):
    root = ET.parse(world_file).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None or not world.get("name"):
        raise RuntimeError(f"No named <world> element in {world_file}")
    return world.get("name")


def _world_control(world_file, request):
    return ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", f"/world/{_world_name(world_file)}/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", request,
        ],
        output="screen",
    )


def _resume_after_controller(event, context):
    if event.returncode != 0:
        return [LogInfo(msg=(
            "Controllers failed to activate; Gazebo remains paused."
        )), Shutdown(reason="controller activation failed")]

    # The activation helper advances the world in bounded batches while the
    # controller switch request is pending and leaves it paused afterwards.
    # Resume only after the strict switch command has succeeded;
    # otherwise /clock and the GPU lidar stop after their startup samples and
    # Explorer waits forever for a second local-map update.
    world_file = LaunchConfiguration("world").perform(context)
    return [
        LogInfo(msg="Controllers activated successfully; resuming Gazebo physics"),
        TimerAction(
            period=0.1,
            actions=[_world_control(world_file, "pause: false")],
        ),
    ]

def _start_trajectory_after_broadcaster(event, _context, trajectory_controller):
    if event.returncode != 0:
        return [LogInfo(msg=(
            "joint_state_broadcaster failed to configure; Gazebo remains paused."
        )), Shutdown(reason="joint state broadcaster configuration failed")]
    return [trajectory_controller]


def _resume_after_spawn(event, context):
    """Kinematic mode has no controller to wait for.

    Legs are locked at the nominal stance by the model itself and motion comes
    from the model-level velocity actuator, so physics can resume as soon as the
    robot exists.  Waiting for a controller_manager here would hang forever,
    because that mode loads no gz_ros2_control plugin at all.
    """
    if event.returncode != 0:
        return [LogInfo(msg="robot spawn failed; Gazebo remains paused"),
                Shutdown(reason="robot spawn failed")]
    if LaunchConfiguration("locomotion_mode").perform(context) != "planning_kinematic":
        return []
    world_file = LaunchConfiguration("world").perform(context)
    return [
        LogInfo(msg="planning_kinematic: no controllers; resuming Gazebo physics"),
        TimerAction(
            period=1.0,
            actions=[_world_control(world_file, "pause: false")],
        ),
    ]


def _activate_controllers_after_load(event, _context, activate_controllers):
    if event.returncode != 0:
        return [LogInfo(msg=(
            "command controller failed to configure; Gazebo remains paused."
        )), Shutdown(reason="command controller configuration failed")]

    return [activate_controllers]


def _await_state_publisher(_context, spawn):
    """Gate creation on a real URDF service response, not node discovery.

    This cannot guarantee a later plugin RPC succeeds. Controller startup has
    its own deadline and failure shuts down the launch rather than leaving a
    paused world with a half-created robot.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rcl_interfaces.srv import GetParameters

    ros_context = rclpy.Context()
    # Do not let rclpy parse the parent launch arguments (for example
    # ``headless:=true`` is not a ROS remap).
    rclpy.init(args=[], context=ros_context)
    node = rclpy.create_node(
        f"go2_sim_preflight_{os.getpid()}", context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.2):
                continue
            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            attempt_deadline = min(deadline, time.monotonic() + 2.0)
            while not future.done() and time.monotonic() < attempt_deadline:
                executor.spin_once(timeout_sec=0.1)
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()
                continue
            response = future.result()
            if response and response.values:
                try:
                    root = ET.fromstring(response.values[0].string_value)
                    if root.tag == "robot" and root.find("link") is not None:
                        node.get_logger().info(
                            "[ROBOT_DESCRIPTION_READY] URDF parameter round-trip verified; spawning Go2")
                        return [spawn]
                except ET.ParseError:
                    pass
            executor.spin_once(timeout_sec=0.1)
        else:
            return [Shutdown(reason="robot_description service did not return a valid URDF within 20s")]
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)


def _gazebo_process(context, server_args, world):
    """Avoid ros_gz_sim's shell wrapper: launch must own the server PID."""
    plugin_path = os.pathsep.join(filter(None, [
        os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
        os.environ.get("LD_LIBRARY_PATH", ""),
    ]))
    return [ExecuteProcess(
        cmd=["ign", "gazebo", *shlex.split(server_args.perform(context)),
             world.perform(context), "--force-version", "6"],
        shell=False, output="screen",
        additional_env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": plugin_path},
        on_exit=Shutdown(reason="Gazebo server exited"),
    )]


def generate_launch_description():
    startup = {"ready": False}

    def resume_controller(event, context):
        startup["ready"] = event.returncode == 0
        return _resume_after_controller(event, context)

    def resume_spawn(event, context):
        if event.returncode == 0 and LaunchConfiguration("locomotion_mode").perform(context) == "planning_kinematic":
            startup["ready"] = True
        return _resume_after_spawn(event, context)

    def startup_deadline(_context):
        return [] if startup["ready"] else [Shutdown(
            reason="Go2 startup exceeded 90s (spawn / controller initialization); shutting down all launch processes")]

    # xacro package:// URIs are converted by Gazebo to
    # model://go2_description/..., so Gazebo must search the parent of this
    # package's share directory.  Without this, the robot is spawned but its
    # DAE visual meshes cannot be rendered.
    default_resource_root = os.path.dirname(get_package_share_directory("go2_description"))
    description_share = FindPackageShare("go2_description")
    model = PathJoinSubstitution([description_share, "xacro", "robot.xacro"])
    world = LaunchConfiguration("world")
    # Start the server and GUI as separate Gazebo processes.  The combined
    # ``ign gazebo <world>`` launcher relies on a private starting-world
    # handshake between the client and server.  After an interrupted run an
    # orphaned / restarting GUI can consume that handshake, leaving the next
    # launch with a GUI which waits forever while no server advertises
    # /gazebo/worlds.  ros_gz_sim/create then prints "Requesting list of world
    # names" indefinitely and SCAN never receives odometry.
    # rl_effort must stay paused while its controllers are configured and
    # activated.  Otherwise the unactuated legs collapse during controller
    # startup.  planning_kinematic has rigid legs and keeps its prior run-on-
    # start behavior; _resume_after_spawn remains its explicit fallback.
    server_args = PythonExpression([
        "'-s ' + ('-r ' if '", LaunchConfiguration("locomotion_mode"),
        "' == 'planning_kinematic' else '') + ",
        "('--headless-rendering ' if '", LaunchConfiguration("headless"),
        "'.lower() in ('1', 'true', 'yes', 'on') else '') + '-v 3 '",
    ])
    bridge_config = PathJoinSubstitution([description_share, "config", "bridge.yaml"])
    robot_description = {
        "robot_description": ParameterValue(Command([
            "xacro ", model,
            " use_gazebo:=true",
            " terrain_velocity_control:=", LaunchConfiguration("terrain_velocity_control"),
            # Kinematic proxy: rigid legs hold the trunk at spawn height so the
            # LiDAR keeps a trustworthy scan plane, and no leg controller exists.
            " lock_legs:=", PythonExpression([
                "'true' if '", LaunchConfiguration("locomotion_mode"),
                "' == 'planning_kinematic' else 'false'"]),
        ]), value_type=str),
        "use_sim_time": True,
    }

    gazebo_server = OpaqueFunction(function=_gazebo_process, args=[server_args, world])
    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g", "-v", "3", "--force-version", "6"],
        shell=False, output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "go2",
            "-allow_renaming", "false",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )
    # The legs are unactuated until these controllers are active, and an
    # unactuated Go2 folds onto the ground within a few seconds -- it cannot
    # stand back up afterwards, which then breaks local planning
    # (the low body marks its own pose as occupied).  Waiting for the
    # controller manager instead of failing its first service call keeps this
    # window short.  planning_kinematic has no leg controller at all (the legs
    # are fixed), so every controller action below is skipped in that mode.
    body_control_mode = PythonExpression([
        "'planning_kinematic' == '", LaunchConfiguration("locomotion_mode"), "'"])
    controllers_enabled = UnlessCondition(body_control_mode)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "180",
            "--switch-timeout", "60",
            "--inactive",
        ],
        output="screen",
        condition=controllers_enabled,
    )
    command_controller_name = PythonExpression([
        "'joint_group_effort_controller'",
    ])
    command_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            command_controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "180",
            "--switch-timeout", "60",
            "--inactive",
        ],
        output="screen",
        condition=controllers_enabled,
    )
    activate_controllers = Node(
        package="go2_description",
        executable="activate_controllers_while_stepping.py",
        arguments=[
            "--world", world,
            "--controller", "joint_state_broadcaster",
            "--controller", command_controller_name,
        ],
        output="screen",
        condition=controllers_enabled,
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=lambda event, context: [joint_state_broadcaster] if event.returncode == 0 else [],
        )
    )
    resume_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=resume_spawn,
        )
    )
    start_command_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=lambda event, context: _start_trajectory_after_broadcaster(
                event, context, command_controller),
        )
    )
    activate_after_load = RegisterEventHandler(
        OnProcessExit(
            target_action=command_controller,
            on_exit=lambda event, context: _activate_controllers_after_load(
                event, context, activate_controllers),
        )
    )
    resume_physics = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_controllers,
            on_exit=resume_controller,
        )
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    return LaunchDescription(
        [
            # Declare resource_path FIRST before it's referenced
            DeclareLaunchArgument("resource_path", default_value=""),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([description_share, "worlds", "empty.sdf"]),
            ),
            DeclareLaunchArgument("terrain_velocity_control", default_value="false"),
            DeclareLaunchArgument("locomotion_mode", default_value="planning_kinematic"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            # Fortress uses IGN_GAZEBO_RESOURCE_PATH.  Set the newer alias as
            # well, keeping this launch usable with newer Gazebo releases.
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[LaunchConfiguration("resource_path"), os.pathsep,
                       default_resource_root, os.pathsep,
                       EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
            ),
            gazebo_server,
            gazebo_gui,
            state_publisher,
            bridge,
            # The preflight blocks while it waits for DDS discovery, so it must
            # run after the event loop has actually started
            # robot_state_publisher; an OpaqueFunction scheduled at visit time
            # would run before that process exists and could never succeed.
            TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=_await_state_publisher, args=[spawn])],
            ),
            start_controllers,
            resume_after_spawn,
            start_command_controller,
            activate_after_load,
            resume_physics,
            TimerAction(period=90.0, actions=[OpaqueFunction(function=startup_deadline)]),
        ]
    )
