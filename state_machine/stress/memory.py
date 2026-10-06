"""Process memory measurement that works on Linux (Jetson), Windows and macOS.

RSS (resident set size) is the RAM the process actually occupies, which is
what counts against the Jetson's 4 GB. psutil is used when installed; on
Linux /proc is read directly so nothing extra is needed on the Jetson.
"""

import os
import sys
import time
import tracemalloc

try:
    import psutil  # optional
    _PROC = psutil.Process()
except ImportError:  # pragma: no cover - depends on the machine
    psutil = None
    _PROC = None

MB = 1024 * 1024


def _proc_status_kb(field: str) -> float | None:
    try:
        with open(f"/proc/{os.getpid()}/status", encoding="ascii") as f:
            for line in f:
                if line.startswith(field + ":"):
                    return float(line.split()[1])
    except OSError:
        return None
    return None


def rss_mb() -> float:
    """Current resident memory of this process, in MB."""
    if _PROC is not None:
        return _PROC.memory_info().rss / MB
    kb = _proc_status_kb("VmRSS")
    if kb is not None:
        return kb / 1024
    import resource  # Unix fallback: only the peak is available
    return _ru_maxrss_mb(resource)


def _ru_maxrss_mb(resource) -> float:
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return peak / MB if sys.platform == "darwin" else peak / 1024


def peak_rss_mb() -> float:
    """Highest resident memory this process has reached, in MB."""
    kb = _proc_status_kb("VmHWM")  # Linux high-water mark
    if kb is not None:
        return kb / 1024
    if _PROC is not None and hasattr(_PROC.memory_info(), "peak_wset"):  # Windows
        return _PROC.memory_info().peak_wset / MB
    try:
        import resource
        return _ru_maxrss_mb(resource)
    except ImportError:
        return rss_mb()


class Probe:
    """Records memory at checkpoints during a workload.

    Workloads call probe.checkpoint(sim_ms) every simulated minute or so, and
    probe.tick(seconds) with the wall time of each loop iteration.
    """

    def __init__(self, trace_heap: bool = False):
        self.trace_heap = trace_heap
        self.checkpoints: list[tuple[float, float, float | None]] = []
        self.tick_count = 0
        self.tick_total_s = 0.0
        self.tick_max_s = 0.0
        self._slow = 0
        self.tick_budget_s = 0.1  # 10 Hz loop
        self.start_rss = None
        self.start_heap = None
        self._t0 = None

    def start(self) -> None:
        import gc
        gc.collect()
        if self.trace_heap:
            tracemalloc.start()
        self.start_rss = rss_mb()
        self.start_heap = self._heap()
        self._t0 = time.perf_counter()

    def _heap(self) -> float | None:
        return tracemalloc.get_traced_memory()[0] / MB if tracemalloc.is_tracing() else None

    def checkpoint(self, sim_ms: int) -> None:
        self.checkpoints.append((sim_ms / 1000.0, rss_mb(), self._heap()))

    def tick(self, seconds: float) -> None:
        self.tick_count += 1
        self.tick_total_s += seconds
        if seconds > self.tick_max_s:
            self.tick_max_s = seconds
        if seconds > self.tick_budget_s:
            self._slow += 1

    def finish(self) -> None:
        """Read end-of-run memory. Call while the workload's data is still alive."""
        self._wall = time.perf_counter() - self._t0
        self._end_rss = rss_mb()
        self._peak_rss = max(peak_rss_mb(), self._end_rss)
        self._end_heap = self._heap()
        self._heap_peak = (tracemalloc.get_traced_memory()[1] / MB
                           if tracemalloc.is_tracing() else None)

    def result(self) -> dict:
        if not hasattr(self, "_end_rss"):
            self.finish()
        wall, end_rss, heap_peak = self._wall, self._end_rss, self._heap_peak
        return {
            "wall_s": wall,
            "start_rss_mb": self.start_rss,
            "end_rss_mb": end_rss,
            "peak_rss_mb": self._peak_rss,
            "growth_mb": end_rss - self.start_rss,
            "heap_start_mb": self.start_heap,
            "heap_end_mb": self._end_heap,
            "heap_peak_mb": heap_peak,
            "ticks": self.tick_count,
            "tick_mean_ms": 1000 * self.tick_total_s / self.tick_count if self.tick_count else None,
            "tick_max_ms": 1000 * self.tick_max_s,
            "ticks_over_budget": self._slow,
            "leak_mb_per_hour": leak_rate_mb_per_hour(self.checkpoints),
            "checkpoints": self.checkpoints,
        }


def leak_rate_mb_per_hour(checkpoints) -> float | None:
    """Slope of RSS over simulated time (least squares on the second half).

    The first half is skipped so one-time warm-up (imports, caches, the
    allocator settling) is not counted as a leak. Returns MB per simulated hour.
    """
    pts = checkpoints[len(checkpoints) // 2:]
    if len(pts) < 3:
        return None
    xs = [p[0] / 3600 for p in pts]
    ys = [p[1] for p in pts]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
