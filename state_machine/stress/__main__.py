"""Run the memory/stress suite and print a report against the RAM budget.

    python3 -m state_machine.stress                         # quick suite
    python3 -m state_machine.stress --suite full            # long runs
    python3 -m state_machine.stress --only pipeline_10min   # one case
    python3 -m state_machine.stress --json report.json --markdown report.md

Each case runs in a fresh Python process (state_machine/stress/workloads.py),
so its peak memory is its own.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]

# (case name, workload, params, what it tells us)
QUICK = [
    ("sm_scenarios_x200", "sm_scenarios", {"repeats": 200},
     "All named demo scenarios, 200x: baseline loop cost"),
    ("sm_patrol_1h", "sm_patrol", {"hours": 1},
     "60 back-to-back missions; flat memory = no leak in the loop"),
    ("sm_fuzz_1h", "sm_fuzz", {"hours": 1, "events_per_s": 5},
     "Random module outputs, 5 events/s"),
    ("sm_flapping_1h", "sm_flapping", {"hours": 1},
     "Bird toggles every tick: transitions list growth"),
    ("sm_payload_64kb", "sm_payload", {"minutes": 10, "kb_per_module": 64},
     "Observation +64 KB for each of 4 future modules (deepcopy cost)"),
    ("sm_history_all_10min", "sm_history", {"minutes": 10, "keep": "all"},
     "Keeping every tick's snapshot in memory"),
    ("sm_history_ring_10min", "sm_history", {"minutes": 10, "keep": "ring", "ring_s": 60},
     "Keeping only the last 60 s of snapshots"),
    ("pipeline_10min", "pipeline", {"minutes": 10},
     "Real CV + Safety + Docking + SM + Motion, 1 camera 640x480 @15 fps"),
]

FULL = [
    ("sm_patrol_8h", "sm_patrol", {"hours": 8},
     "An 8-hour day of missions, state machine only"),
    ("sm_fuzz_8h", "sm_fuzz", {"hours": 8, "events_per_s": 20},
     "8 h of random module outputs at 20 events/s"),
    ("sm_flapping_8h", "sm_flapping", {"hours": 8},
     "8 h of a bird flickering every tick"),
    ("sm_payload_256kb", "sm_payload", {"minutes": 3, "kb_per_module": 256},
     "Observation +256 KB per module (1 MB world deep-copied every tick)"),
    ("sm_history_all_1h", "sm_history", {"minutes": 60, "keep": "all"},
     "1 h of every snapshot kept in memory"),
    ("pipeline_1h", "pipeline", {"minutes": 60},
     "Full pipeline 1 h, motion logs kept in memory (MemoryLogSink)"),
    ("pipeline_1h_nulllog", "pipeline", {"minutes": 60, "motion_log": "null"},
     "Full pipeline 1 h, motion logs serialised then dropped"),
    ("pipeline_2cam_720p_buf2s", "pipeline",
     {"minutes": 20, "cameras": 2, "width": 1280, "height": 720, "frame_buffer_s": 2},
     "Front + downward camera at 1280x720, keeping 2 s of frames"),
    ("pipeline_2cam_1080p_buf5s", "pipeline",
     {"minutes": 10, "cameras": 2, "width": 1920, "height": 1080, "frame_buffer_s": 5},
     "Worst case: 2x 1080p cameras, 5 s frame buffer each"),
    ("pipeline_model_512mb", "pipeline", {"minutes": 10, "reserve_mb": 512},
     "Pipeline plus a 512 MB stand-in for a future detector model"),
]

SUITES = {"quick": QUICK, "full": QUICK + FULL}


def run_case(workload: str, params: dict, trace_heap: bool) -> dict:
    cmd = [sys.executable, "-m", "state_machine.stress.workloads", workload, json.dumps(params)]
    if trace_heap:
        cmd.append("--trace-heap")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(REPO_ROOT), env.get("PYTHONPATH")]))
    proc = subprocess.run(cmd, cwd=REPO_ROOT, env=env, capture_output=True, text=True)
    if proc.returncode != 0:
        return {"error": (proc.stderr or proc.stdout).strip().splitlines()[-1:]}
    return json.loads(proc.stdout.strip().splitlines()[-1])


def project(result: dict, hours: float) -> float | None:
    """RSS after `hours` of simulated running, extrapolating the leak rate."""
    rate = result.get("leak_mb_per_hour")
    sim_h = result.get("sim_s", 0) / 3600
    if rate is None or sim_h <= 0:
        return None
    return result["end_rss_mb"] + max(0.0, rate) * max(0.0, hours - sim_h)


def verdict(mb: float, budget: float) -> str:
    share = mb / budget
    return "OVER" if share > 1 else "WATCH" if share > 0.5 else "ok"


def fmt(value, spec=".1f", none="-") -> str:
    return none if value is None else format(value, spec)


def render(rows, budget: float, total: float, reserve: float, hours: float) -> list[str]:
    lines = [
        f"RAM budget for our code: {budget:.0f} MB "
        f"({total:.0f} MB total - {reserve:.0f} MB reserved for OS/GPU; change with "
        f"--total-mb / --reserve-mb)",
        "",
        f"| case | sim | ticks | ms/tick (mean / max) | start MB | peak MB | growth MB "
        f"| leak MB/h | {hours:g} h proj. MB | % budget | |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name, _, _, _, r in rows:
        if "error" in r:
            lines.append(f"| {name} | ERROR: {' '.join(r['error'])} |" + " |" * 9)
            continue
        proj = project(r, hours)
        worst = max(r["peak_rss_mb"], proj or 0)
        sim = r.get("sim_s", 0)
        sim_txt = f"{sim/3600:.1f} h" if sim >= 3600 else f"{sim/60:.0f} min"
        lines.append(
            f"| {name} | {sim_txt} | {r['ticks']:,} | "
            f"{fmt(r['tick_mean_ms'], '.2f')} / {fmt(r['tick_max_ms'], '.1f')} | "
            f"{r['start_rss_mb']:.0f} | {r['peak_rss_mb']:.0f} | {r['growth_mb']:+.1f} | "
            f"{fmt(r['leak_mb_per_hour'], '+.2f')} | {fmt(proj, '.0f')} | "
            f"{100 * worst / budget:.1f}% | {verdict(worst, budget)} |")
    lines += ["", "Cases:"]
    for name, _, _, desc, r in rows:
        extras = {k: v for k, v in r.items() if k in (
            "transitions", "frames", "frame_mb", "motion_history_len", "motion_log_records",
            "kept_snapshots", "observation_json_kb", "events", "ticks_over_budget")}
        lines.append(f"- **{name}**: {desc}. {extras}")
    return lines


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suite", choices=SUITES, default="quick")
    parser.add_argument("--only", help="Comma-separated case names (from any suite)")
    parser.add_argument("--list", action="store_true", help="List cases and exit")
    parser.add_argument("--total-mb", type=float, default=4096, help="Device RAM (Jetson Nano: 4096)")
    parser.add_argument("--reserve-mb", type=float, default=1024,
                        help="RAM assumed taken by the OS, desktop and GPU (default 1024)")
    parser.add_argument("--project-hours", type=float, default=8,
                        help="Extrapolate memory growth to this many hours (default 8)")
    parser.add_argument("--trace-heap", action="store_true",
                        help="Also measure the Python heap with tracemalloc (slower)")
    parser.add_argument("--json", type=Path, help="Write raw results here")
    parser.add_argument("--markdown", type=Path, help="Write the report table here")
    args = parser.parse_args(argv)

    every = {c[0]: c for c in QUICK + FULL}
    if args.list:
        for name, workload, params, desc in QUICK + FULL:
            suite = "quick" if (name, workload, params, desc) in QUICK else "full"
            print(f"{name:28} [{suite}] {desc}")
        return 0
    cases = [every[n] for n in args.only.split(",")] if args.only else SUITES[args.suite]

    budget = args.total_mb - args.reserve_mb
    rows = []
    for name, workload, params, desc in cases:
        t0 = time.time()
        print(f"running {name} ...", end=" ", flush=True, file=sys.stderr)
        result = run_case(workload, params, args.trace_heap)
        print(f"{time.time() - t0:.1f}s", file=sys.stderr)
        rows.append((name, workload, params, desc, result))

    lines = render(rows, budget, args.total_mb, args.reserve_mb, args.project_hours)
    print("\n".join(lines))
    if args.json:
        args.json.write_text(json.dumps([{"case": n, "workload": w, "params": p, "description": d, **r}
                                         for n, w, p, d, r in rows], indent=2), encoding="utf-8")
    if args.markdown:
        args.markdown.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 1 if any("error" in r for *_, r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
