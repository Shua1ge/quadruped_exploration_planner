#!/usr/bin/env python3
"""Validate an exported Git commit without starting simulation or motion."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from urllib.parse import unquote


def clean_environment(log_directory):
    env = os.environ.copy()
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
                "PYTHONPATH", "LD_LIBRARY_PATH", "SCAN_DYN_WS"):
        env.pop(key, None)
    env.update(ROS_LOG_DIR=str(log_directory), PYTHONDONTWRITEBYTECODE="1",
               PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", CMAKE_BUILD_PARALLEL_LEVEL="1",
               MAKEFLAGS="-j1")
    return env


def ros_command(command, setup=None):
    script = 'source /opt/ros/humble/setup.bash || exit $?\n'
    if setup is not None:
        script += 'source "$1" || exit $?\nshift\n'
    script += 'exec "$@"'
    return ["bash", "--noprofile", "--norc", "-c", script, "release-check",
            *([str(setup)] if setup is not None else []), *map(str, command)]


def export_commit(repo, commit, destination):
    destination.mkdir(parents=True)
    archive = subprocess.Popen(["git", "archive", commit], cwd=repo,
                               stdout=subprocess.PIPE)
    try:
        unpack = subprocess.run(["tar", "-xf", "-", "-C", str(destination)],
                                stdin=archive.stdout, check=False)
    finally:
        archive.stdout.close()
    if archive.wait() or unpack.returncode:
        raise RuntimeError("git archive extraction failed")


LINK_CHECK = r'''
from pathlib import Path
import re
from urllib.parse import unquote
root = Path.cwd()
broken = []
for path in [root / 'README.md', *sorted((root / 'docs/dyn').glob('*.md'))]:
    for target in re.findall(r'\]\(([^)]+)\)', path.read_text()):
        if '://' in target or target.startswith(('#', 'mailto:')):
            continue
        local = unquote(target.split('#')[0])
        if not (path.parent / local).exists():
            broken.append((str(path.relative_to(root)), target))
assert not broken, broken
print('README and dyn documentation local links resolve')
'''

INSTALLED_CHECK = r'''
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix
root, workspace = map(Path, sys.argv[1:])
prefix = Path(get_package_prefix('scan_planner'))
assert prefix.resolve() == (workspace / 'install/scan_planner').resolve(), prefix
executables = prefix / 'lib/scan_planner'
for name in ('scan_planner_node', 'closed_loop_controller', 'mid360_ray_bridge',
             'frontier_explorer.py', 'global_representation_node.py',
             'go2_locomotion_supervisor.py'):
    assert (executables / name).is_file(), name
sys.path.insert(0, str(executables))
import frontier_explorer
import global_representation_node
import go2_locomotion_supervisor
import numpy as np
from explorer_core.grid import FREE
from explorer_core.wavefront_graph import IncrementalWavefrontGraph
graph = IncrementalWavefrontGraph()
graph.update(np.full((4, 4), FREE, dtype=np.int8), .2, (0., 0.))
graph.routing_view()
from go2_rl_lab_policy import Go2RlLabPolicyAdapter
policy = Go2RlLabPolicyAdapter(root / 'models/go2_rl_lab_policy.pt')
for _ in range(10):
    action = policy.run(['action'], {'obs_now': np.zeros((1, 45), dtype=np.float32)})[0]
    assert action.shape == (12,) and np.isfinite(action).all()
policy.reset()
print('Independent install, imports, tiny Global query, 10 policy inferences and reset OK')
print('No ROS node instantiated; no simulation or motion output')
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--ref", default="HEAD", help="committed candidate; working files are excluded")
    parser.add_argument("--build", action="store_true", help="also build and check an independent install")
    parser.add_argument("--jobs", type=int, default=2, choices=range(1, 9))
    parser.add_argument("--output", type=Path, required=True, help="new report directory")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f"output already exists: {output}")
    output.mkdir(parents=True)
    (output / "ros_log").mkdir()
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), ref=args.ref,
                  validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  build_requested=args.build, simulation_tested=False,
                  campus_download_tested=False, long_run_tested=False,
                  checks=[], result="FAILED")
    env = clean_environment(output / "ros_log")

    def run(name, command, cwd, timeout=120):
        log = output / f"{name}.log"
        started = time.monotonic()
        with log.open("w") as stream:
            proc = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = proc.wait(timeout=timeout)
                reason = ""
            except subprocess.TimeoutExpired:
                # Only the process group created for this check is terminated.
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                code, reason = 124, "timeout"
        report["checks"].append(dict(name=name, returncode=code, reason=reason,
                                     elapsed_s=round(time.monotonic()-started, 3),
                                     log=str(log)))
        print(f"{name}: {'PASS' if code == 0 else 'FAIL'} ({log})", flush=True)
        if code:
            raise RuntimeError(f"{name} failed; see {log}")

    try:
        repo = args.repo.expanduser().resolve()
        commit = subprocess.check_output(
            ["git", "rev-parse", "--verify", "--end-of-options", args.ref+"^{commit}"],
            cwd=repo, text=True).strip()
        report["commit"] = commit
        report["excluded_worktree_changes"] = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=repo, text=True).splitlines()
        workspace = Path(tempfile.mkdtemp(prefix="dyn-release-", dir="/tmp"))
        root = workspace / "src/SCAN-Planner"
        export_commit(repo, commit, root)
        report["workspace"] = str(workspace)
        run("resource_closure", ["/usr/bin/python3", "tools/test_snapshot_portability.py"], root)
        run("documentation_links", ["/usr/bin/python3", "-c", LINK_CHECK], root)
        run("launch_dry_run", ["/usr/bin/python3", "run_newmine_rl.py", "--dry-run",
                               "--workspace", str(workspace)], root)
        if args.build:
            run("clean_build", ros_command([
                "colcon", "build", "--symlink-install", "--parallel-workers", str(args.jobs),
                "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DUSE_GPU=OFF",
                "-DPython3_EXECUTABLE=/usr/bin/python3", "-DPYTHON_EXECUTABLE=/usr/bin/python3"
            ]), workspace, timeout=1200)
            setup = workspace / "install/setup.bash"
            run("launch_arguments", ros_command([
                "ros2", "launch", "scan_planner", "newmine_exploration.launch.py", "--show-args"
            ], setup), root)
            run("startup_unit_tests", ros_command([
                "/usr/bin/python3", "-m", "unittest", "discover", "-s",
                "src/simulator/Utils/go2_description/test", "-p", "test_*.py", "-v"
            ], setup), root)
            run("robot_description", ros_command([
                "ros2", "run", "xacro", "xacro",
                "src/simulator/Utils/go2_description/xacro/robot.xacro",
                "locomotion_mode:=rl_effort"
            ], setup), root)
            run("installed_imports_policy", ros_command([
                "/usr/bin/python3", "-c", INSTALLED_CHECK, str(root), str(workspace)
            ], setup), root)
        report["result"] = ("BUILD_AND_OFFLINE_CHECKS_PASSED" if args.build
                            else "SOURCE_CHECKS_PASSED")
    except (OSError, subprocess.SubprocessError, RuntimeError) as error:
        report["error"] = str(error)
        print(str(error), file=sys.stderr)
    finally:
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
    print(f"{report['result']}: {output / 'report.json'}")
    return 1 if report["result"] == "FAILED" else 0


if __name__ == "__main__":
    sys.exit(main())
