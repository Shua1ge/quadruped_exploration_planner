#!/usr/bin/env python3
"""Analyze min_speed_probe CSV: per-level tracking, stall and attitude metrics.

"Effective" level criteria (all over the steady window, defaults tunable):
  - forward progress: least-squares slope of heading-projected x
    >= min_ratio * cmd_vx
  - sustained motion: fraction of 1 s rolling windows with displacement
    < stall_m at most max_stall_frac (catches step-stall cycling)
  - attitude: roll/pitch RMS and peak within bounds, no fall
  - heading: |yaw drift rate| <= max_yaw_rate (policy must hold heading)

Prints a per-level table and the lowest passing level (with the margin to its
neighbors, since "minimum effective speed" is only meaningful together with
how close the next level is to failing).

Usage:
  python3 probe_analyze.py /tmp/min_speed_probe.csv
"""

import argparse
import csv
import math
import os
import sys

PASS_KEYS = ["vx_eff", "ratio", "stall_frac", "roll_rms", "pitch_rms",
             "roll_max", "pitch_max", "yaw_rate", "fall"]


def quat_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_rows(path):
    with open(path) as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key in ("t", "cmd_vx", "x", "y", "z", "roll", "pitch", "yaw",
                    "wx", "wy", "wz"):
            row[key] = float(row[key])
        row["level_idx"] = int(row["level_idx"])
    return rows


def lsq_slope(ts, xs):
    n = len(ts)
    mt = sum(ts) / n
    mx = sum(xs) / n
    denom = sum((t - mt) ** 2 for t in ts)
    if denom < 1e-9:
        return 0.0
    return sum((t - mt) * (x - mx) for t, x in zip(ts, xs)) / denom


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def analyze_level(rows, cmd, ramp, tail, stall_m, stall_win):
    if len(rows) < 20:
        return None
    yaw0 = rows[0]["yaw"]
    cos0, sin0 = math.cos(-yaw0), math.sin(-yaw0)
    t0 = rows[0]["t"]
    steady = []
    for row in rows:
        dt = row["t"] - t0
        if dt < ramp or dt > (rows[-1]["t"] - t0) - tail:
            continue
        dx = row["x"] - rows[0]["x"]
        dy = row["y"] - rows[0]["y"]
        steady.append({
            "t": dt,
            "x": cos0 * dx - sin0 * dy,
            "y": sin0 * dx + cos0 * dy,
            "roll": row["roll"], "pitch": row["pitch"], "z": row["z"],
            "yaw": row["yaw"] - yaw0,
        })
    if len(steady) < 10:
        return None
    ts = [s["t"] for s in steady]
    xs = [s["x"] for s in steady]
    vx_eff = lsq_slope(ts, xs)
    displacement = xs[-1] - xs[0]

    stall_windows = 0
    stalled_windows = 0
    step = stall_win / 4.0
    start = ts[0]
    end = ts[-1]
    cursor = start
    while cursor + stall_win <= end:
        inside = [s for s in steady if cursor <= s["t"] <= cursor + stall_win]
        if len(inside) >= 5:
            stall_windows += 1
            if abs(inside[-1]["x"] - inside[0]["x"]) < stall_m:
                stalled_windows += 1
        cursor += step
    stall_frac = stalled_windows / stall_windows if stall_windows else 1.0

    rolls = [s["roll"] for s in steady]
    pitches = [s["pitch"] for s in steady]
    zmin = min(s["z"] for s in steady)
    fall = zmin < 0.15 or max(abs(r) for r in rolls) > 0.6 \
        or max(abs(p) for p in pitches) > 0.8
    return {
        "cmd": cmd,
        "vx_eff": vx_eff,
        "ratio": vx_eff / cmd if cmd > 0 else 0.0,
        "displacement": displacement,
        "stall_frac": stall_frac,
        "roll_rms": rms(rolls),
        "pitch_rms": rms(pitches),
        "roll_max": max(abs(r) for r in rolls),
        "pitch_max": max(abs(p) for p in pitches),
        "yaw_rate": math.degrees(lsq_slope(ts, [s["yaw"] for s in steady])),
        "z_min": zmin,
        "fall": fall,
        "n_steady": len(steady),
    }


def passed(metrics, args):
    return (not metrics["fall"]
            and metrics["ratio"] >= args.min_ratio
            and metrics["stall_frac"] <= args.max_stall_frac
            and metrics["roll_rms"] <= args.roll_rms_max
            and metrics["pitch_rms"] <= args.pitch_rms_max
            and metrics["roll_max"] <= args.roll_max
            and metrics["pitch_max"] <= args.pitch_max
            and abs(metrics["yaw_rate"]) <= args.max_yaw_rate_deg
            and metrics["displacement"] >= args.min_disp)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--ramp", type=float, default=2.0,
                        help="seconds dropped at each level start")
    parser.add_argument("--tail", type=float, default=0.5)
    parser.add_argument("--stall-m", type=float, default=0.02)
    parser.add_argument("--stall-win", type=float, default=1.0)
    parser.add_argument("--min-ratio", type=float, default=0.5)
    parser.add_argument("--max-stall-frac", type=float, default=0.1)
    parser.add_argument("--roll-rms-max", type=float, default=0.12)
    parser.add_argument("--pitch-rms-max", type=float, default=0.15)
    parser.add_argument("--roll-max", type=float, default=0.3)
    parser.add_argument("--pitch-max", type=float, default=0.4)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=10.0)
    parser.add_argument("--min-disp", type=float, default=0.05)
    parser.add_argument("--metrics-out", default=None)
    args = parser.parse_args()

    rows = read_rows(args.csv_path)
    drives = {}
    for row in rows:
        if row["phase"] == "drive":
            drives.setdefault(row["level_idx"], []).append(row)

    results = []
    for level_idx in sorted(drives):
        cmd = drives[level_idx][0]["cmd_vx"]
        metrics = analyze_level(drives[level_idx], cmd, args.ramp, args.tail,
                                args.stall_m, args.stall_win)
        if metrics is None:
            print(f"level {level_idx} (vx={cmd:.2f}): too few steady samples")
            continue
        metrics["pass"] = passed(metrics, args)
        results.append(metrics)

    print(f"{'vx_cmd':>7} {'vx_eff':>7} {'ratio':>6} {'disp':>6} "
          f"{'stall%':>6} {'rollRMS':>7} {'pitchRMS':>8} {'yaw°/s':>7} "
          f"{'zmin':>5} {'pass':>5}")
    for m in results:
        print(f"{m['cmd']:7.2f} {m['vx_eff']:7.3f} {m['ratio']:6.2f} "
              f"{m['displacement']:6.2f} {m['stall_frac']*100:6.1f} "
              f"{m['roll_rms']:7.3f} {m['pitch_rms']:8.3f} "
              f"{m['yaw_rate']:7.1f} {m['z_min']:5.2f} "
              f"{str(m['pass']):>5}")

    good = [m for m in results if m["pass"]]
    if not good:
        print("\nno level passed: policy produced no effective sustained "
              "forward motion in the tested range")
    else:
        best = min(good, key=lambda m: m["cmd"])
        print(f"\n最低有效速度 ≈ {best['cmd']:.2f} m/s "
              f"(vx_eff={best['vx_eff']:.3f}, ratio={best['ratio']:.2f})")
        failed_below = [m for m in results if not m["pass"]
                        and m["cmd"] < best["cmd"]]
        if failed_below:
            near = max(failed_below, key=lambda m: m["cmd"])
            print(f"  下一档 {near['cmd']:.2f} m/s 失败项: "
                  + ", ".join(
                      k for k in ("ratio", "stall_frac", "roll_rms",
                                  "pitch_rms", "roll_max", "pitch_max",
                                  "yaw_rate", "fall")
                      if not _ok(k, near, args)))

    if args.metrics_out:
        with open(args.metrics_out, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print(f"per-level metrics -> {args.metrics_out}")
    elif results:
        print("(use --metrics-out to save the per-level table)")


def _ok(key, metrics, args):
    if key == "ratio":
        return metrics["ratio"] >= args.min_ratio
    if key == "stall_frac":
        return metrics["stall_frac"] <= args.max_stall_frac
    if key == "roll_rms":
        return metrics["roll_rms"] <= args.roll_rms_max
    if key == "pitch_rms":
        return metrics["pitch_rms"] <= args.pitch_rms_max
    if key == "roll_max":
        return metrics["roll_max"] <= args.roll_max
    if key == "pitch_max":
        return metrics["pitch_max"] <= args.pitch_max
    if key == "yaw_rate":
        return abs(metrics["yaw_rate"]) <= args.max_yaw_rate_deg
    if key == "fall":
        return not metrics["fall"]
    return True


if __name__ == "__main__":
    main()
