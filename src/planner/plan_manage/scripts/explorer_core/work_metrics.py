"""Measurement-only work timing for the Explorer.

Enabled by the `work_metrics_enable` parameter. Phase rows record wall and
thread CPU; the transaction total additionally records run-queue wait from
/proc/self/task/<tid>/schedstat (two reads per transaction).
"""
import csv
import functools
import threading
import time


def _schedstat_wait_ns():
    try:
        tid = threading.get_native_id()
        with open(f"/proc/self/task/{tid}/schedstat") as fh:
            parts = fh.read().split()
        return int(parts[1]) if len(parts) >= 2 else -1
    except Exception:
        return -1


class WorkMetrics:
    HEADER = ["work_class", "start_steady_ns", "phase", "transaction_id", "tid", "wall_ms",
              "cpu_ms", "runnable_ms", "count", "input_size", "result"]

    def __init__(self, enabled=False, path=""):
        self.enabled = bool(enabled)
        self._fh = None
        self._writer = None
        self._lock = threading.Lock()
        self._local = threading.local()
        self._seq = 0
        self._rows = 0
        if self.enabled:
            self._fh = open(path or "/tmp/explorer_work_metrics.csv", "w", newline="")
            self._writer = csv.writer(self._fh)
            self._writer.writerow(self.HEADER)
            self._fh.flush()

    def begin_transaction(self, work_class, count=0, input_size=0):
        if not self.enabled:
            return None
        self._seq += 1
        tx = {
            "id": self._seq,
            "prev": getattr(self._local, "tx", None),
            "work_class": work_class,
            "tid": threading.get_native_id(),
            "t0": time.monotonic_ns(),
            "c0": time.thread_time_ns(),
            "w0": _schedstat_wait_ns(),
            "count": count,
            "input_size": input_size,
        }
        self._local.tx = tx
        return tx

    def end_transaction(self, tx, result="done"):
        if not self.enabled or tx is None:
            return
        wall_ms = (time.monotonic_ns() - tx["t0"]) / 1e6
        cpu_ms = (time.thread_time_ns() - tx["c0"]) / 1e6
        w1 = _schedstat_wait_ns()
        if w1 >= 0 and tx["w0"] >= 0 and w1 >= tx["w0"]:
            runnable = (w1 - tx["w0"]) / 1e6
        else:
            runnable = float("nan")
        self._row(tx["work_class"], tx["t0"], "total", tx, wall_ms, cpu_ms, runnable,
                  tx["count"], tx["input_size"], result)
        if getattr(self._local, "tx", None) is tx:
            self._local.tx = tx.get("prev")

    def mark(self):
        if not self.enabled:
            return None
        return (time.monotonic_ns(), time.thread_time_ns())

    def record_since(self, phase, mark, count=0, input_size=0, result="done"):
        if not self.enabled or mark is None:
            return
        tx = getattr(self._local, "tx", None)
        if tx is None:
            return
        wall_ms = (time.monotonic_ns() - mark[0]) / 1e6
        cpu_ms = (time.thread_time_ns() - mark[1]) / 1e6
        self._row(tx["work_class"] if tx else "unknown", mark[0], phase, tx, wall_ms,
                  cpu_ms, float("nan"), count, input_size, result)

    def _row(self, work_class, start_ns, phase, tx, wall_ms, cpu_ms, runnable_ms,
             count, input_size, result):
        if not self.enabled:
            return
        with self._lock:
            self._writer.writerow([
                work_class, start_ns, phase, tx["id"] if tx else 0,
                tx["tid"] if tx else threading.get_native_id(),
                f"{wall_ms:.3f}", f"{cpu_ms:.3f}",
                "nan" if runnable_ms != runnable_ms else f"{runnable_ms:.3f}",
                count, input_size, result])
            self._rows += 1
            if self._rows % 50 == 0:
                self._fh.flush()

    def close(self):
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None


def timed(work_class):
    """Decorator: wrap a method in a WorkMetrics transaction (nested-safe)."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(self, *args, **kwargs):
            wm = getattr(self, "work_metrics", None)
            tx = wm.begin_transaction(work_class) if wm is not None else None
            try:
                return fn(self, *args, **kwargs)
            finally:
                if wm is not None:
                    wm.end_transaction(tx)
        return wrapper
    return deco
