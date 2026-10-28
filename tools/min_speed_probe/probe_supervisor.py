#!/usr/bin/env python3
"""Run the existing supervisor with the new stateful policy, for simulation tests."""

import argparse
import sys
from pathlib import Path

from stateful_policy import ProbePolicyAdapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--archive", required=True,
                        help="original policies.zip supplies the matching actor for non-refresh steps")
    parser.add_argument("--refresh-every", type=int, default=5,
                        help="estimator refresh interval in processed actor steps (provisional)")
    args, ros_args = parser.parse_known_args()
    scripts = Path(__file__).resolve().parents[2] / "src/planner/plan_manage/scripts"
    sys.path.insert(0, str(scripts))
    import rclpy
    from go2_locomotion_supervisor import Go2LocomotionSupervisor

    rclpy.init(args=[
        "--ros-args", "-r", "body_pose:=/quad_0/body_pose",
        "-p", "locomotion_mode:=rl_effort", "-p", "use_sim_time:=true",
        "-p", "locomotion_policy:=parkour_moe_stateful",
        "-p", f"model_path:={args.model}",
        "-p", f"estimator_refresh_every:={args.refresh_every}",
        *ros_args,
    ])
    adapter = ProbePolicyAdapter(args.model, args.archive, args.refresh_every)

    def factory(model_path):
        if Path(model_path).resolve() != Path(args.model).resolve():
            raise ValueError("model_path override must match --model")
        return adapter

    node = Go2LocomotionSupervisor(policy_session_factory=factory)
    node.get_logger().warning(
        "[PROBE_ASSUMED_CONTRACT] simulation-only test: using the existing "
        f"45-D observation/joint order, default pose, action_scale={node.action_scale}, "
        f"kp={node.kp}, kd={node.kd}. New training config has not been supplied. "
        f"Blind mode; actor={1.0/node.policy_dt:.0f}Hz, "
        f"estimator refresh every {args.refresh_every} processed actor steps. "
        "This is not an approved deployment configuration.")

    def log_statistics():
        steps, refreshes, norm = adapter.statistics()
        node.get_logger().info(
            f"[PROBE_POLICY] actor_steps={steps} estimator_refreshes={refreshes} "
            f"shared_state_l2={norm:.3f} state={node.state} reason={node.reason}")

    node.create_timer(5.0, log_statistics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
