#!/usr/bin/env python3
"""Activate controllers, drain finite steps, then verify continuous simulation."""

import argparse
from dataclasses import dataclass
import json
import queue
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController


def _world_name(world_file: str) -> str:
    root = ET.parse(world_file).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None or not world.get("name"):
        raise ValueError(f"no named <world> element in {world_file}")
    return world.get("name")


class StartupError(RuntimeError):
    """A bounded startup operation failed its actual completion condition."""


@dataclass(frozen=True)
class WorldStats:
    iterations: int
    sim_ns: int
    paused: bool
    received_at: float

    @classmethod
    def from_json(cls, line):
        msg = json.loads(line)
        sim = msg.get("simTime", {})
        return cls(int(msg.get("iterations", 0)),
                   int(sim.get("sec", 0)) * 1_000_000_000 + int(sim.get("nsec", 0)),
                   bool(msg.get("paused", False)), time.monotonic())


class WorldStatsStream:
    """Observe native completion, independently of the ROS simulation clock."""

    def __init__(self, world_name):
        self._messages = queue.Queue(maxsize=64)
        self._error = None
        self._process = subprocess.Popen(
            ["ign", "topic", "-e", "--json-output",
             "-t", f"/world/{world_name}/stats"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        try:
            for line in self._process.stdout:
                if not line.strip():
                    continue
                sample = WorldStats.from_json(line)
                # Startup needs recent state, not an unbounded stats backlog.
                try:
                    self._messages.put_nowait(sample)
                except queue.Full:
                    self._messages.get_nowait()
                    self._messages.put_nowait(sample)
        except (ValueError, TypeError, KeyError) as error:
            self._error = f"invalid native world stats: {error}"

    def next(self, timeout):
        try:
            return self._messages.get(timeout=timeout)
        except queue.Empty:
            if self._error or self._process.poll() is not None:
                raise StartupError(self._error or "native world stats process exited")
            return None

    def close(self):
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)
        self._reader.join(timeout=2.0)
        self._process.stdout.close()


def _control_world(world_name: str, request: str) -> bool:
    try:
        result = subprocess.run(
            ["ign", "service", "-s", f"/world/{world_name}/control",
             "--reqtype", "ignition.msgs.WorldControl",
             "--reptype", "ignition.msgs.Boolean", "--timeout", "5000",
             "--req", request],
            check=False, capture_output=True, text=True, timeout=6.0)
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0 and "data: true" in result.stdout.lower()


def _wait_stats(stream, predicate, timeout, spin, reason):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        spin()
        sample = stream.next(min(0.05, max(0.0, deadline - time.monotonic())))
        if sample is not None and predicate(sample):
            return sample
    raise StartupError(reason)


def _advance_paused_batch(stream, current, steps, control, spin, timeout):
    # The service ACK only queues multi_step. Fortress will pause AGAIN when
    # those iterations finish, even if somebody sends unpause in the meantime.
    target = current.iterations + steps
    submitted_at = time.monotonic()
    if not control(f"pause: true multi_step: {steps}"):
        raise StartupError("STARTUP_STEP_REQUEST_REJECTED")
    return _wait_stats(
        stream, lambda s: s.received_at >= submitted_at and s.paused
        and s.iterations >= target, timeout, spin,
        f"STARTUP_STEP_DRAIN_TIMEOUT expected_iterations={target}")


def _resume_and_verify(stream, paused, control, spin, timeout):
    submitted_at = time.monotonic()
    if not control("pause: false"):
        raise StartupError("STARTUP_RESUME_REQUEST_REJECTED")
    first_running = None

    def running(sample):
        nonlocal first_running
        if sample.received_at < submitted_at:
            return False
        if sample.paused:
            if first_running is not None:
                raise StartupError("STARTUP_WORLD_REPAUSED_AFTER_RESUME")
            return False
        if (sample.iterations <= paused.iterations or
                sample.sim_ns <= paused.sim_ns):
            return False
        if first_running is None:
            first_running = sample
            return False
        # Distinct native messages must demonstrate sustained progress; an ACK
        # or a repeated cached clock cannot satisfy this condition.
        return (sample.received_at - first_running.received_at >= 0.3
                and sample.iterations > first_running.iterations
                and sample.sim_ns > first_running.sim_ns)

    return _wait_stats(stream, running, timeout, spin,
                       "STARTUP_RUNNING_VERIFICATION_TIMEOUT")


def _all_controllers_active(response, controller_names) -> bool:
    """Return whether every requested controller is reported active."""
    if response is None:
        return False
    states = {
        controller.name: controller.state
        for controller in response.controller
    }
    return all(states.get(name) == "active" for name in controller_names)


def _confirm_controllers_active(node, controller_names,
                                timeout_sec: float = 2.0) -> bool:
    """Resolve a late switch response from controller_manager's real state.

    A switch can be applied in the final stepped update while its service
    response is still in flight.  Treating that response delay as an activation
    failure leaves Gazebo paused even though all requested controllers are
    already active.
    """
    client = node.create_client(
        ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().warning(
            "controller state verification service did not become available")
        return False

    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().warning("controller state verification timed out")
        return False
    try:
        response = future.result()
    except Exception as error:  # service transport errors are reported as failure
        node.get_logger().warning(
            f"controller state verification failed: {error}")
        return False
    return _all_controllers_active(response, controller_names)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, help="Gazebo SDF world file")
    parser.add_argument("--controller", action="append", required=True)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--switch-timeout", type=float, default=15.0)
    parser.add_argument("--step-batch", type=int, default=25)
    parser.add_argument("--world-timeout", type=float, default=10.0,
                        help="Wall-time limit for each native completion check")
    args, ros_args = parser.parse_known_args()
    if (args.step_batch <= 0 or args.service_timeout <= 0 or
            args.switch_timeout <= 0 or args.world_timeout <= 0):
        parser.error("step batch and timeouts must be positive")

    try:
        world_name = _world_name(args.world)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"invalid Gazebo world file: {error}", file=sys.stderr)
        return 1

    # launch_ros appends ``--ros-args`` (and may add remaps or parameters) to
    # every Node executable.  Keep the helper's arguments separate and pass
    # the remaining ROS arguments to rclpy instead of rejecting them.
    rclpy.init(args=ros_args)
    node = rclpy.create_node("go2_controller_activator")
    client = node.create_client(
        SwitchController, "/controller_manager/switch_controller")
    stats = None
    try:
        if not client.wait_for_service(timeout_sec=args.service_timeout):
            node.get_logger().error("controller switch service did not become available")
            return 1

        stats = WorldStatsStream(world_name)

        def spin():
            if not rclpy.ok():
                raise StartupError("startup interrupted")
            rclpy.spin_once(node, timeout_sec=0.0)

        current = _wait_stats(stats, lambda s: s.paused, args.world_timeout,
                              spin, "STARTUP_INITIAL_PAUSE_TIMEOUT")
        request = SwitchController.Request()
        request.activate_controllers = args.controller
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(args.switch_timeout)
        request.timeout.nanosec = int(
            (args.switch_timeout - int(args.switch_timeout)) * 1_000_000_000)

        # call_async queues the request before the first Gazebo step.  The
        # controller manager then gets update cycles in bounded batches until
        # it applies the switch and completes the service response.
        future = client.call_async(request)
        deadline = time.monotonic() + args.switch_timeout + 2.0
        activation_confirmed = False
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if future.done():
                break
            if time.monotonic() >= deadline:
                # One final executor drain catches a response queued by the
                # last stepped controller-manager update.  If it is still late,
                # query the authoritative controller states before declaring a
                # failure.  This avoids a false-negative exit that prevents the
                # launch event handler from resuming Gazebo.
                rclpy.spin_once(node, timeout_sec=0.25)
                if future.done():
                    break
                activation_confirmed = _confirm_controllers_active(
                    node, args.controller)
                if activation_confirmed:
                    node.get_logger().warning(
                        "switch response missed its deadline, but all requested "
                        "controllers are active; accepting activation")
                    break
                node.get_logger().error(
                    "controller activation timed out and requested controllers "
                    "are not all active")
                return 1
            current = _advance_paused_batch(
                stats, current, args.step_batch,
                lambda request: _control_world(world_name, request), spin,
                args.world_timeout)
            # Even if the switch future completes midway through the batch,
            # no next batch or continuous unpause is issued before it drains.

        if not activation_confirmed:
            response = future.result()
            if response is None or not response.ok:
                node.get_logger().error(
                    "controller manager rejected controller activation")
                return 1
        node.get_logger().info(
            f"[STARTUP_STEP_DRAINED] controllers active, paused=true "
            f"iterations={current.iterations}")
        running = _resume_and_verify(
            stats, current, lambda request: _control_world(world_name, request),
            spin, args.world_timeout)
        node.get_logger().info(
            f"[SIMULATION_RUNNING_VERIFIED] paused=false "
            f"iterations={current.iterations}->{running.iterations} "
            f"sim_time={current.sim_ns / 1e9:.3f}->{running.sim_ns / 1e9:.3f}s")
        return 0
    except (StartupError, OSError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        if stats is not None:
            stats.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
