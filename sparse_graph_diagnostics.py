#!/usr/bin/env python3
"""
Sparse Global Graph diagnostics for quadruped exploration CSV logs.

Usage:
    python3 sparse_graph_diagnostics.py path/to/run.csv

Optional:
    python3 sparse_graph_diagnostics.py run.csv --window 100 --out-dir ./sparse_report
    python3 sparse_graph_diagnostics.py run.csv --no-show

Outputs three key figures:
  1) 01_graph_health.png
     Sparse graph nodes/edges over time -> graph growth, fragmentation symptoms, churn.
  2) 02_sparse_usability.png
     Windowed candidate hit, region-pair hit, and frontier attachment rates -> whether
     sparse routing remains usable as exploration grows.
  3) 03_fallback_reasons.png
     Candidate/region fallback reason composition -> tells you what to fix next.

The terminal summary also prints latency, validation, graph-churn, and final graph stats,
so latency does not need a fourth figure.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = {
    "elapsed_s",
    "planning_events",
    "sparse_graph_revision",
    "sparse_graph_nodes",
    "sparse_graph_edges",
    "sparse_candidate_queries",
    "sparse_candidate_hits",
    "sparse_candidate_fallbacks",
    "sparse_candidate_miss_start_unattached",
    "sparse_candidate_miss_target_unattached",
    "sparse_candidate_miss_disconnected",
    "sparse_candidate_miss_connector_rejected",
    "sparse_candidate_miss_blocked_disabled",
    "sparse_region_pair_queries",
    "sparse_region_pair_hits",
    "sparse_region_pair_fallbacks",
    "sparse_region_miss_start_unattached",
    "sparse_region_miss_target_unattached",
    "sparse_region_miss_disconnected",
    "sparse_region_miss_connector_rejected",
    "sparse_region_miss_blocked_disabled",
    "last_topology_attached_clusters",
    "last_topology_unattached_clusters",
    "last_sparse_candidate_ms",
    "last_sparse_region_ms",
    "last_candidate_tree_ms",
    "last_planning_ms",
    "dense_final_validation_searches",
    "sparse_final_validation_failures",
}

FALLBACK_LABELS = [
    ("start unattached", "start_unattached"),
    ("target unattached", "target_unattached"),
    ("disconnected", "disconnected"),
    ("connector rejected", "connector_rejected"),
    ("blocked disabled", "blocked_disabled"),
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Generate 3 key Sparse Global Graph diagnostic plots from a CSV log."
    )
    p.add_argument("csv_file", type=Path, help="Input CSV file")
    p.add_argument(
        "--window",
        type=float,
        default=100.0,
        help="Time-window size in seconds for usability plot (default: 100)",
    )
    p.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory (default: <csv_stem>_sparse_report beside CSV)",
    )
    p.add_argument(
        "--no-show",
        action="store_true",
        help="Save figures but do not open matplotlib windows",
    )
    return p.parse_args()


def to_float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    try:
        x = float(value)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def load_rows(path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        rows = list(reader)
        fields = list(reader.fieldnames)

    if not rows:
        raise ValueError("CSV contains no data rows")

    missing = sorted(REQUIRED_COLUMNS - set(fields))
    if missing:
        raise ValueError(
            "CSV is missing required columns:\n  - " + "\n  - ".join(missing)
        )

    rows.sort(key=lambda r: to_float(r, "elapsed_s"))
    return rows, fields


def percentile(values: Sequence[float], p: float) -> float:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return 0.0
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    frac = pos - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


def counter_total(rows: Sequence[Dict[str, str]], key: str) -> float:
    """Total increments of a cumulative counter, robust to counter resets."""
    vals = [max(0.0, to_float(r, key)) for r in rows]
    if not vals:
        return 0.0
    total = vals[0]
    for prev, cur in zip(vals[:-1], vals[1:]):
        if cur >= prev:
            total += cur - prev
        else:
            # Counter reset/restart: count the new counter value from zero.
            total += cur
    return total


def event_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, str]]:
    """Keep one row for each new planning event to avoid oversampling repeated log rows."""
    out: List[Dict[str, str]] = []
    last_event = None
    for r in rows:
        event = int(to_float(r, "planning_events"))
        if event != last_event:
            out.append(r)
            last_event = event
    return out


def revision_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, str]]:
    """Keep one row per sparse graph revision for graph-churn diagnostics."""
    out: List[Dict[str, str]] = []
    last_rev = None
    for r in rows:
        rev = int(to_float(r, "sparse_graph_revision"))
        if rev != last_rev:
            out.append(r)
            last_rev = rev
    return out


def positive_delta(prev: float, cur: float) -> float:
    return cur - prev if cur >= prev else cur


def windowed_rates(
    rows: Sequence[Dict[str, str]], window_s: float
) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Return window centers and candidate hit, region hit, attachment rates in %."""
    if window_s <= 0:
        raise ValueError("--window must be > 0")

    duration = to_float(rows[-1], "elapsed_s")
    nwin = max(1, int(math.ceil(duration / window_s)))

    cand_q = [0.0] * nwin
    cand_h = [0.0] * nwin
    region_q = [0.0] * nwin
    region_h = [0.0] * nwin

    # Aggregate increments of cumulative counters per time window.
    prev = rows[0]
    for cur in rows[1:]:
        t = to_float(cur, "elapsed_s")
        idx = min(nwin - 1, max(0, int(t // window_s)))
        cand_q[idx] += positive_delta(
            to_float(prev, "sparse_candidate_queries"),
            to_float(cur, "sparse_candidate_queries"),
        )
        cand_h[idx] += positive_delta(
            to_float(prev, "sparse_candidate_hits"),
            to_float(cur, "sparse_candidate_hits"),
        )
        region_q[idx] += positive_delta(
            to_float(prev, "sparse_region_pair_queries"),
            to_float(cur, "sparse_region_pair_queries"),
        )
        region_h[idx] += positive_delta(
            to_float(prev, "sparse_region_pair_hits"),
            to_float(cur, "sparse_region_pair_hits"),
        )
        prev = cur

    # If the CSV starts after counters have already incremented, count the first values.
    idx0 = min(nwin - 1, max(0, int(to_float(rows[0], "elapsed_s") // window_s)))
    cand_q[idx0] += max(0.0, to_float(rows[0], "sparse_candidate_queries"))
    cand_h[idx0] += max(0.0, to_float(rows[0], "sparse_candidate_hits"))
    region_q[idx0] += max(0.0, to_float(rows[0], "sparse_region_pair_queries"))
    region_h[idx0] += max(0.0, to_float(rows[0], "sparse_region_pair_hits"))

    # Attachment is a per-planning-event snapshot, so aggregate event snapshots only.
    attached = [0.0] * nwin
    attach_total = [0.0] * nwin
    for r in event_rows(rows):
        t = to_float(r, "elapsed_s")
        idx = min(nwin - 1, max(0, int(t // window_s)))
        a = max(0.0, to_float(r, "last_topology_attached_clusters"))
        u = max(0.0, to_float(r, "last_topology_unattached_clusters"))
        attached[idx] += a
        attach_total[idx] += a + u

    centers = [min((i + 0.5) * window_s, duration) for i in range(nwin)]
    candidate_rate = [100.0 * h / q if q > 0 else 0.0 for h, q in zip(cand_h, cand_q)]
    region_rate = [100.0 * h / q if q > 0 else 0.0 for h, q in zip(region_h, region_q)]
    attachment_rate = [
        100.0 * a / tot if tot > 0 else 0.0 for a, tot in zip(attached, attach_total)
    ]
    return centers, candidate_rate, region_rate, attachment_rate


def plot_graph_health(rows: Sequence[Dict[str, str]], out_path: Path) -> None:
    t = [to_float(r, "elapsed_s") for r in rows]
    nodes = [to_float(r, "sparse_graph_nodes") for r in rows]
    edges = [to_float(r, "sparse_graph_edges") for r in rows]

    fig = plt.figure(figsize=(10, 5.2))
    ax = fig.add_subplot(111)
    ax.plot(t, nodes, label="Sparse graph nodes", linewidth=1.8)
    ax.plot(t, edges, label="Sparse graph edges", linewidth=1.8)
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Count")
    ax.set_title("1. Sparse graph health: size and temporal stability")
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)


def plot_sparse_usability(
    rows: Sequence[Dict[str, str]], window_s: float, out_path: Path
) -> Tuple[List[float], List[float], List[float], List[float]]:
    centers, cand, region, attach = windowed_rates(rows, window_s)

    fig = plt.figure(figsize=(10, 5.2))
    ax = fig.add_subplot(111)
    ax.plot(centers, cand, marker="o", label="Candidate sparse hit rate")
    ax.plot(centers, region, marker="o", label="Region-pair sparse hit rate")
    ax.plot(centers, attach, marker="o", label="Frontier attachment rate")
    ax.set_xlabel(f"Elapsed time (window = {window_s:g} s)")
    ax.set_ylabel("Rate (%)")
    ax.set_ylim(bottom=0)
    ax.set_title("2. Sparse usability: does it remain useful as exploration grows?")
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    return centers, cand, region, attach


def plot_fallback_reasons(rows: Sequence[Dict[str, str]], out_path: Path) -> Tuple[List[float], List[float]]:
    labels = [x[0] for x in FALLBACK_LABELS]
    cand_vals = [
        counter_total(rows, f"sparse_candidate_miss_{suffix}")
        for _, suffix in FALLBACK_LABELS
    ]
    region_vals = [
        counter_total(rows, f"sparse_region_miss_{suffix}")
        for _, suffix in FALLBACK_LABELS
    ]

    cand_total = sum(cand_vals)
    region_total = sum(region_vals)
    cand_pct = [100.0 * v / cand_total if cand_total else 0.0 for v in cand_vals]
    region_pct = [100.0 * v / region_total if region_total else 0.0 for v in region_vals]

    x = list(range(len(labels)))
    width = 0.38
    fig = plt.figure(figsize=(10, 5.2))
    ax = fig.add_subplot(111)
    ax.bar([i - width / 2 for i in x], cand_pct, width, label="Candidate queries")
    ax.bar([i + width / 2 for i in x], region_pct, width, label="Region-pair queries")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("Share of sparse fallbacks (%)")
    ax.set_title("3. Sparse fallback reasons: what should be fixed next?")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    return cand_pct, region_pct


def fmt_pct(num: float, den: float) -> str:
    return f"{100.0 * num / den:.2f}%" if den > 0 else "n/a"


def latency_stats(rows: Sequence[Dict[str, str]], key: str) -> Tuple[float, float]:
    vals = [to_float(r, key) for r in event_rows(rows)]
    vals = [v for v in vals if v > 0.0 and math.isfinite(v)]
    if not vals:
        return 0.0, 0.0
    return median(vals), percentile(vals, 0.95)


def print_summary(
    rows: Sequence[Dict[str, str]],
    window_s: float,
    cand_window: Sequence[float],
    region_window: Sequence[float],
    attach_window: Sequence[float],
    cand_fallback_pct: Sequence[float],
    region_fallback_pct: Sequence[float],
) -> None:
    duration = to_float(rows[-1], "elapsed_s")
    final_nodes = int(to_float(rows[-1], "sparse_graph_nodes"))
    final_edges = int(to_float(rows[-1], "sparse_graph_edges"))

    cq = counter_total(rows, "sparse_candidate_queries")
    ch = counter_total(rows, "sparse_candidate_hits")
    rq = counter_total(rows, "sparse_region_pair_queries")
    rh = counter_total(rows, "sparse_region_pair_hits")

    ev = event_rows(rows)
    attached_sum = sum(max(0.0, to_float(r, "last_topology_attached_clusters")) for r in ev)
    attach_total = sum(
        max(0.0, to_float(r, "last_topology_attached_clusters"))
        + max(0.0, to_float(r, "last_topology_unattached_clusters"))
        for r in ev
    )

    validations = counter_total(rows, "dense_final_validation_searches")
    validation_fail = counter_total(rows, "sparse_final_validation_failures")

    sc_med, sc_p95 = latency_stats(rows, "last_sparse_candidate_ms")
    sr_med, sr_p95 = latency_stats(rows, "last_sparse_region_ms")
    dense_med, dense_p95 = latency_stats(rows, "last_candidate_tree_ms")
    plan_med, plan_p95 = latency_stats(rows, "last_planning_ms")

    rev = revision_rows(rows)
    node_drops = []
    edge_drops = []
    node_drop_fracs = []
    edge_drop_fracs = []
    for a, b in zip(rev[:-1], rev[1:]):
        na, nb = to_float(a, "sparse_graph_nodes"), to_float(b, "sparse_graph_nodes")
        ea, eb = to_float(a, "sparse_graph_edges"), to_float(b, "sparse_graph_edges")
        if nb < na:
            node_drops.append(na - nb)
            if na > 0:
                node_drop_fracs.append((na - nb) / na)
        if eb < ea:
            edge_drops.append(ea - eb)
            if ea > 0:
                edge_drop_fracs.append((ea - eb) / ea)

    # For a simple undirected graph, C >= N-E when E < N. Treat only as a diagnostic lower bound.
    component_lower_bound = max(1, final_nodes - final_edges) if final_nodes > 0 else 0

    dominant_cand_idx = max(range(len(cand_fallback_pct)), key=lambda i: cand_fallback_pct[i])
    dominant_region_idx = max(range(len(region_fallback_pct)), key=lambda i: region_fallback_pct[i])

    first_nonzero_cand = next((x for x in cand_window if x > 0), 0.0)
    late_cand_vals = [x for x in cand_window[-2:] if x > 0]
    late_cand = sum(late_cand_vals) / len(late_cand_vals) if late_cand_vals else 0.0

    print("\n" + "=" * 72)
    print("SPARSE GLOBAL GRAPH DIAGNOSTIC SUMMARY")
    print("=" * 72)
    print(f"Run duration                     : {duration:.1f} s ({duration/60.0:.1f} min)")
    print(f"Final sparse graph               : {final_nodes} nodes, {final_edges} edges")
    print(f"Component lower-bound diagnostic : >= {component_lower_bound} (simple undirected assumption)")
    print()
    print("USABILITY")
    print(f"Candidate sparse hit             : {int(ch)}/{int(cq)} = {fmt_pct(ch, cq)}")
    print(f"Region-pair sparse hit           : {int(rh)}/{int(rq)} = {fmt_pct(rh, rq)}")
    print(f"Event-weighted attachment        : {fmt_pct(attached_sum, attach_total)}")
    if first_nonzero_cand > 0:
        print(f"Early -> late candidate hit      : {first_nonzero_cand:.2f}% -> {late_cand:.2f}%")
    print()
    print("FALLBACK DIAGNOSIS")
    print(
        f"Dominant candidate miss          : {FALLBACK_LABELS[dominant_cand_idx][0]} "
        f"({cand_fallback_pct[dominant_cand_idx]:.1f}%)"
    )
    print(
        f"Dominant region miss             : {FALLBACK_LABELS[dominant_region_idx][0]} "
        f"({region_fallback_pct[dominant_region_idx]:.1f}%)"
    )
    print()
    print("GRAPH STABILITY")
    print(f"Largest one-revision node drop   : {max(node_drops, default=0):.0f}")
    print(f"Largest one-revision edge drop   : {max(edge_drops, default=0):.0f}")
    print(f"Largest node drop fraction       : {100*max(node_drop_fracs, default=0):.1f}%")
    print(f"Largest edge drop fraction       : {100*max(edge_drop_fracs, default=0):.1f}%")
    print()
    print("SAFETY / VALIDATION")
    print(
        f"Sparse final validation failures : {int(validation_fail)}/{int(validations)} "
        f"= {fmt_pct(validation_fail, validations)}"
    )
    print()
    print("LATENCY (median / P95)")
    print(f"Sparse candidate routing         : {sc_med:.2f} / {sc_p95:.2f} ms")
    print(f"Sparse region routing            : {sr_med:.2f} / {sr_p95:.2f} ms")
    print(f"Dense candidate tree             : {dense_med:.2f} / {dense_p95:.2f} ms")
    print(f"Full planning                    : {plan_med:.2f} / {plan_p95:.2f} ms")
    print()
    print("HOW TO JUDGE WHETHER A NEW VERSION IS BETTER")
    print("  1) Plot 1 should become smoother: fewer large node/edge collapses.")
    print("  2) Plot 2 should move upward and stay flat over time, not decay in long runs.")
    print("  3) Plot 3's start/target-unattached bars should shrink substantially.")
    print("  4) Final validation failure rate should NOT rise just to gain more sparse hits.")
    print("  5) Sparse latency can stay roughly the same; utilization matters more than shaving milliseconds.")
    print("=" * 72 + "\n")


def main() -> None:
    args = parse_args()
    csv_path = args.csv_file.expanduser().resolve()
    if not csv_path.exists():
        raise SystemExit(f"Input file does not exist: {csv_path}")

    rows, _ = load_rows(csv_path)

    out_dir = (
        args.out_dir.expanduser().resolve()
        if args.out_dir is not None
        else csv_path.parent / f"{csv_path.stem}_sparse_report"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    p1 = out_dir / "01_graph_health.png"
    p2 = out_dir / "02_sparse_usability.png"
    p3 = out_dir / "03_fallback_reasons.png"

    plot_graph_health(rows, p1)
    _, cand_window, region_window, attach_window = plot_sparse_usability(rows, args.window, p2)
    cand_fb_pct, region_fb_pct = plot_fallback_reasons(rows, p3)

    print_summary(
        rows,
        args.window,
        cand_window,
        region_window,
        attach_window,
        cand_fb_pct,
        region_fb_pct,
    )

    print("Saved figures:")
    print(f"  {p1}")
    print(f"  {p2}")
    print(f"  {p3}")

    should_show = not args.no_show and bool(os.environ.get("DISPLAY"))
    if should_show:
        plt.show()
    elif not args.no_show:
        print("\nNo DISPLAY detected; figures were saved but not opened. Use them from the output directory.")


if __name__ == "__main__":
    main()
