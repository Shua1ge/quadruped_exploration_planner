#!/usr/bin/env python3
"""Run the isolated SCAN-Planner NewMine stack with the go2_rl_lab policy.

The rl_effort supervisor starts disarmed.  It does not begin the policy
get-up until /robot/enter_navigation is called after joint state, IMU, and
odometry are available.  This wrapper arms it, runs for a bounded time, and
shuts down the whole launch process group.

Examples:
    ./run_newmine_rl.py --duration 90
    ./run_newmine_rl.py --duration 300
    ./run_newmine_rl.py --duration 90 --gui --show-rviz
"""

import argparse
import math
import shlex
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent

def find_workspace():
    override = os.environ.get("SCAN_DYN_WS")
    if override:
        return Path(override).expanduser().resolve()
    return next((p for p in REPO.parents if (p / "install/setup.bash").is_file()),
                REPO.parent.parent)

WS = find_workspace()
WORLD = REPO / "resources/newmine/NewMine.sdf"
SETUP = (
    "source /opt/ros/humble/setup.bash; "
    "source install/setup.bash; "
)


def run_checked(cmd, env, timeout=15.0):
    try:
        return subprocess.run(
            cmd, cwd=WS, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout, check=False)
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return subprocess.CompletedProcess(cmd, 124, output)


def main():
    global WS
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=WS,
                        help="built colcon workspace; also accepts SCAN_DYN_WS")
    parser.add_argument("--policy-model", type=Path,
                        default=REPO / "models/go2_rl_lab_policy.pt")
    parser.add_argument("--duration", type=float, default=90.0,
                        help="seconds to run after navigation is armed; "
                             "0 means run until Ctrl-C or launch exit")
    parser.add_argument("--domain", type=int, default=146,
                        help="ROS_DOMAIN_ID for this isolated run")
    parser.add_argument("--world", type=Path, default=WORLD)
    parser.add_argument("--map-size-x", type=float, default=None,
                        help="Explorer map width in metres (launch default: 280)")
    parser.add_argument("--map-size-y", type=float, default=None,
                        help="Explorer map height in metres (launch default: 280)")
    parser.add_argument("--init-x", type=float, default=None)
    parser.add_argument("--init-y", type=float, default=None)
    parser.add_argument("--init-z", type=float, default=None)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the launch command without starting or arming anything")
    parser.add_argument("--gui", action="store_true",
                        help="start the Ignition GUI (not headless)")
    parser.add_argument("--show-rviz", action="store_true")
    parser.add_argument("--task-mode", action="store_true", default=True)
    parser.add_argument("--global-mode", choices=("off", "shadow", "route_cost"),
                        default="shadow", help="Global usage: default observes only")
    parser.add_argument("--sparse-routing", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="legacy sparse-route switch; used only with --global-mode route_cost")
    parser.add_argument("--actual-route", type=Path, default=None,
                        help="actual odometry CSV; default: metrics name + _actual_route.csv")
    parser.add_argument("--record-route", action=argparse.BooleanOptionalAction,
                        default=True, help="record measured 3D route independently at 5 Hz")
    parser.add_argument("--metrics", type=Path, default=None)
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args()

    WS = args.workspace.expanduser().resolve()
    args.world = args.world.expanduser().resolve()
    args.policy_model = args.policy_model.expanduser().resolve()
    if not args.dry_run and not (WS / "install/setup.bash").is_file():
        parser.error(f"workspace is not built: {WS}; see docs/dyn/SETUP_ZH.md")
    if not args.policy_model.is_file():
        parser.error(f"policy file not found: {args.policy_model}")
    if not args.world.is_file():
        parser.error(f"world file not found: {args.world}")
    for name in ("map_size_x", "map_size_y", "init_x", "init_y", "init_z"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or
                                  (name.startswith("map_size_") and value <= 0)):
            parser.error(f"invalid --{name.replace('_', '-')}: {value}")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log or Path(f"/tmp/scan_newmine_rl_{stamp}.log")
    metrics_path = args.metrics or Path(f"/tmp/scan_newmine_rl_{stamp}.csv")

    route_path = (args.actual_route or
                  metrics_path.with_name(metrics_path.stem + "_actual_route.csv")).expanduser().resolve()
    if args.record_route and route_path in (metrics_path.expanduser().resolve(), log_path.expanduser().resolve()):
        parser.error("--actual-route must differ from --metrics and --log")
    if args.record_route and not args.dry_run and (route_path.exists() or route_path.with_suffix('.json').exists()):
        parser.error(f"actual route output already exists: {route_path}; choose --actual-route")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain)
    env["IGN_PARTITION"] = f"scan_newmine_{args.domain}"

    resource_roots = [str(REPO / "resources/newmine/models"),
                      str(args.world.parent / "models")]
    if env.get("IGN_GAZEBO_RESOURCE_PATH"):
        resource_roots.append(env["IGN_GAZEBO_RESOURCE_PATH"])
    env["IGN_GAZEBO_RESOURCE_PATH"] = os.pathsep.join(resource_roots)
    launch_args = [
        "ros2", "launch", "scan_planner", "newmine_exploration.launch.py",
        f"world_file:={args.world}",
        "use_gazebo_physics:=true", "use_sim_time:=true",
        "use_pcd_map:=false", "use_sdf_map:=false",
        f"headless:={'false' if args.gui else 'true'}",
        f"show_rviz:={'true' if args.show_rviz else 'false'}",
        "auto_start:=true",
        f"task_mode:={'true' if args.task_mode else 'false'}",
        f"global_usage_mode:={args.global_mode}",
        f"sparse_routing_enabled:={'true' if args.sparse_routing else 'false'}",
        "locomotion_mode:=rl_effort",
        "locomotion_policy:=go2_rl_lab",
        f"policy_model_path:={args.policy_model}",
        f"gazebo_resource_path:={env['IGN_GAZEBO_RESOURCE_PATH']}",
        f"metrics_file:={metrics_path}",
    ]
    for name in ("map_size_x", "map_size_y", "init_x", "init_y", "init_z"):
        value = getattr(args, name)
        if value is not None:
            launch_args.append(f"{name}:={value}")
    if args.dry_run:
        print(shlex.join(launch_args))
        return 0
    launch_cmd = ["bash", "-lc", SETUP + "exec " + shlex.join(launch_args)]

    log_file = open(log_path, "w", encoding="utf-8")
    print(f"SCAN NewMine RL: domain={args.domain} headless={not args.gui} "
          f"rviz={args.show_rviz} duration={args.duration}s")
    print(f"  log={log_path}")
    print(f"  metrics={metrics_path}")

    proc = subprocess.Popen(
        launch_cmd, cwd=WS, env=env, stdout=log_file,
        stderr=subprocess.STDOUT, start_new_session=True, text=True)
    recorder = None
    armed = False
    try:
        if args.record_route:
            record_cmd = ["python3", str(REPO / "tools/record_actual_route.py"),
                          "--output", str(route_path), "--world", str(args.world)]
            recorder = subprocess.Popen(
                ["bash", "-lc", SETUP + "exec " + shlex.join(record_cmd)],
                cwd=WS, env=env, stdout=log_file, stderr=subprocess.STDOUT,
                start_new_session=True, text=True)
            print(f"  actual_route={route_path}")
        for attempt in range(1, 19):
            time.sleep(5.0)
            if proc.poll() is not None:
                print(f"launch exited before arming (code={proc.returncode})",
                      file=sys.stderr)
                break
            result = run_checked(
                ["bash", "-lc", SETUP +
                 "ros2 service call /robot/enter_navigation "
                 "std_srvs/srv/Trigger '{}'"], env)
            output = result.stdout or ""
            print(f"arming attempt {attempt} at {attempt * 5}s: "
                  f"{output.strip().splitlines()[-1] if output.strip() else 'no response'}")
            if "success=True" in output or "already enabled" in output:
                armed = True
                break
            if result.returncode == 124:
                print("  service call timed out; retrying", file=sys.stderr)
        if not armed:
            print("navigation did not arm; inspect the launch log", file=sys.stderr)
            return 4
        if args.duration > 0:
            print(f"navigation armed; running for {args.duration:g}s")
            time.sleep(args.duration)
        else:
            print("navigation armed; running until Ctrl-C or launch exit")
            while proc.poll() is None:
                time.sleep(1.0)
        return 0
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    finally:
        if proc.poll() is None:
            print(f"stopping launch process group {proc.pid}")
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.wait(timeout=5.0)
        if recorder is not None:
            if recorder.poll() is None:
                try:
                    os.killpg(recorder.pid, signal.SIGINT)
                    recorder.wait(timeout=5.)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(recorder.pid, signal.SIGTERM)
                    recorder.wait(timeout=5.)
            if recorder.returncode not in (0, 130, -signal.SIGINT):
                print(f"route recorder exited with code={recorder.returncode}; inspect {log_path}",
                      file=sys.stderr)
            print(f"actual_route={route_path}")
        log_file.flush()
        log_file.close()
        print(f"metrics={metrics_path}")


if __name__ == "__main__":
    sys.exit(main())
