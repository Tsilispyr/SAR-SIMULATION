"""
SAR Batch Test Runner — V2 (Stress Profile)
Higher enemy density (~3x V1), tighter radii. Outputs batch_results_{strategy}_v2.json/csv.

Tiers:
  easy       - 3 patrol, 2 aggr, 4 targets,  radius  600m  (20%)
  medium     - 5 patrol, 3 aggr, 6 targets,  radius  800m  (30%)
  hard       - 7 patrol, 5 aggr, 8 targets,  radius 1000m  (30%)
  very_hard  - 9 patrol, 6 aggr, 10 targets, radius 1200m  (20%)

Enemy density: ~4.4/km2  vs  ~1.5/km2 in V1.

Usage:
  python batch_test_v2.py                                     # 30 runs, no UAV
  python batch_test_v2.py --runs 30 --uav-strategy threat_informed
  Strategies: none | lawnmower | threat_informed | target_informed | hybrid
"""
import requests
import time
import json
import random
import argparse
import csv
import os
from datetime import datetime

# == Config =================
DEFAULT_HOST    = "http://localhost:8000"
MAX_WALL_SECS   = 720     # wall-clock timeout per mission (4x * 720s = 2880 sim-sec)
POLL_INTERVAL   = 0.15
SETTLE_DELAY    = 1.2
SIM_SPEED       = 4.0     # keep at or below 5x

# == V2 Stress presets (~3x enemy density vs V1) ===================
PRESETS = [
    {"label": "easy",      "n_patrol": 3, "n_aggr": 2, "n_targets": 4,  "radius":  600, "frac": 0.20},
    {"label": "medium",    "n_patrol": 5, "n_aggr": 3, "n_targets": 6,  "radius":  800, "frac": 0.30},
    {"label": "hard",      "n_patrol": 7, "n_aggr": 5, "n_targets": 8,  "radius": 1000, "frac": 0.30},
    {"label": "very_hard", "n_patrol": 9, "n_aggr": 6, "n_targets": 10, "radius": 1200, "frac": 0.20},
]

BASE_LAT, BASE_LON = 37.9838, 23.7275

# == MPI weights (Mission Performance Index) =======================
# Tunable; values chosen to weight success > safety > time efficiency.
MPI_W_OUTCOME    = 0.50   # mission result score
MPI_W_SAFETY     = 0.30   # escape-event penalty
MPI_W_EFFICIENCY = 0.20   # simulation time efficiency
MPI_E_NORM       = 5.0    # escape events considered "worst case"
MPI_T_NORM       = 2880.0 # sim-sec budget (4x * 720s wall)


def compute_mpi(row: dict) -> float:
    outcome_map = {"success": 1.0, "partial": 0.5, "failed": 0.0}
    outcome    = outcome_map.get(row.get("result", "failed"), 0.0)
    safety     = max(0.0, 1.0 - row.get("escape_events", 0) / MPI_E_NORM)
    t          = row.get("sim_elapsed_s") or MPI_T_NORM
    efficiency = max(0.0, 1.0 - t / MPI_T_NORM)
    return round(MPI_W_OUTCOME * outcome + MPI_W_SAFETY * safety + MPI_W_EFFICIENCY * efficiency, 4)


def build_run_queue(n_runs: int) -> list:
    queue = []
    remaining = n_runs
    for i, p in enumerate(PRESETS):
        count = round(p["frac"] * n_runs) if i < len(PRESETS) - 1 else remaining
        queue.extend([p] * count)
        remaining -= count
    random.shuffle(queue)
    return queue


def api(host, method, path, timeout=30, retries=2, **kwargs):
    fn = getattr(requests, method)
    for attempt in range(1, retries + 1):
        try:
            resp = fn(f"{host}{path}", timeout=timeout, **kwargs)
            resp.raise_for_status()
            return resp.json()
        except Exception as e:
            if attempt < retries:
                time.sleep(5)
            else:
                raise


def wait_for_backend(host: str, max_wait: int = 60):
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            requests.get(f"{host}/api/sim/state", timeout=5)
            return True
        except Exception:
            time.sleep(5)
    return False


def load_map_with_retry(host: str, params: dict, retries: int = 3) -> bool:
    for attempt in range(1, retries + 1):
        try:
            api(host, "get", "/api/map/vector", timeout=120, params=params)
            return True
        except Exception as e:
            if attempt < retries:
                wait = attempt * 15
                print(f"    map load attempt {attempt} failed ({e.__class__.__name__}), "
                      f"retrying in {wait}s...")
                time.sleep(wait)
            else:
                print(f"    map load failed after {retries} attempts: {e.__class__.__name__}")
    return False


def run_mission(host: str, run_idx: int, preset: dict, uav_strategy: str = "none") -> dict:
    lat, lon = BASE_LAT, BASE_LON

    try:
        api(host, "get", "/api/sim/clear", timeout=60)
    except Exception:
        print(f"    backend unresponsive, waiting up to 60s...")
        if not wait_for_backend(host, max_wait=60):
            return {"run": run_idx, "result": "map_error", "preset": preset["label"],
                    "uav_strategy": uav_strategy, "preset_profile": "stress"}
        api(host, "get", "/api/sim/clear", timeout=60)
    time.sleep(SETTLE_DELAY)

    params = {
        "lat": lat, "lon": lon,
        "radius": preset["radius"],
        "n_patrol": preset["n_patrol"],
        "n_aggr":   preset["n_aggr"],
        "n_targets": preset["n_targets"],
        "patrol_speed": round(random.uniform(2.0, 3.0), 1),
        "aggr_speed":   round(random.uniform(3.5, 5.0), 1),
        "agent_speed":  round(random.uniform(5.5, 7.0), 1),
        "detection_radius": round(random.uniform(40.0, 60.0), 1),
        "n_agents": 1,
        "mode": "random",
        "uav_strategy": uav_strategy,
    }
    if not load_map_with_retry(host, params):
        print(f"  [x] RUN {run_idx:3d} | map_error | preset={preset['label']}")
        return {"run": run_idx, "result": "map_error", "preset": preset["label"],
                "uav_strategy": uav_strategy, "preset_profile": "stress"}

    time.sleep(0.3)
    api(host, "post", "/api/sim/speed", json={"multiplier": SIM_SPEED})

    try:
        api(host, "post", "/api/sim/start")
    except Exception as e:
        print(f"  [RUN {run_idx:3d}] START FAILED: {e}")
        return {"run": run_idx, "result": "start_error", "preset": preset["label"],
                "preset_profile": "stress"}

    t0 = time.time()
    final_stats = None
    while (time.time() - t0) < MAX_WALL_SECS:
        try:
            stats = api(host, "get", "/api/stats")
            if stats.get("game_over"):
                final_stats = stats
                break
        except Exception:
            pass
        time.sleep(POLL_INTERVAL)

    wall_time = round(time.time() - t0, 1)

    if final_stats is None:
        try:
            final_stats = api(host, "get", "/api/stats")
        except Exception:
            pass
        targets_remaining = (final_stats or {}).get("targets_remaining", preset["n_targets"])
        targets_total     = (final_stats or {}).get("targets_total",     preset["n_targets"])
        targets_collected = targets_total - targets_remaining
        score             = (final_stats or {}).get("score", 0)
        print(f"  [T] RUN {run_idx:3d} | timeout  | score={score:4d} | "
              f"targets {targets_collected}/{targets_total} | {preset['label']:<9} | wall={wall_time:.0f}s")
        row = {
            "run": run_idx, "result": "failed", "preset": preset["label"],
            "uav_strategy": uav_strategy, "preset_profile": "stress",
            "score": score, "sim_elapsed_s": MPI_T_NORM,
            "wall_sec": wall_time,
            "targets_collected": targets_collected, "targets_total": targets_total,
            "escape_events": 0, "n_patrol": preset["n_patrol"],
            "n_aggr": preset["n_aggr"], "radius": preset["radius"],
            "status_msg": "TIMEOUT",
        }
        row["mpi"] = compute_mpi(row)
        return row

    status = final_stats.get("status", "")
    score  = final_stats.get("score", 0)
    elapsed = final_stats.get("elapsed", 0)
    targets_total     = final_stats.get("targets_total", preset["n_targets"])
    targets_remaining = final_stats.get("targets_remaining", 0)
    targets_collected = targets_total - targets_remaining
    escape_events = sum(
        a.get("escape_events", 0) for a in final_stats.get("agents_stats", [])
    )

    if "MISSION SUCCESS" in status and "PARTIAL" not in status:
        result = "success"
    elif "PARTIAL" in status:
        result = "partial"
    elif "FAILED" in status:
        result = "failed"
    else:
        result = "unknown"

    row = {
        "run":               run_idx,
        "result":            result,
        "preset":            preset["label"],
        "uav_strategy":      uav_strategy,
        "preset_profile":    "stress",
        "score":             score,
        "sim_elapsed_s":     elapsed,
        "wall_sec":          wall_time,
        "targets_collected": targets_collected,
        "targets_total":     targets_total,
        "escape_events":     escape_events,
        "n_patrol":          preset["n_patrol"],
        "n_aggr":            preset["n_aggr"],
        "radius":            preset["radius"],
        "status_msg":        status,
    }
    row["mpi"] = compute_mpi(row)

    icon = {"success": "✓", "partial": "~", "failed": "✗"}.get(result, "?")
    print(
        f"  [{icon}] RUN {run_idx:3d} | {result:<8} | score={score:4d} | "
        f"targets {targets_collected}/{targets_total} | {preset['label']:<9} | "
        f"sim={elapsed:5.0f}s | wall={wall_time:4.1f}s | escapes={escape_events} | mpi={row['mpi']:.3f}"
    )
    return row


def summarise(results: list) -> dict:
    valid = [r for r in results if r.get("result") in ("success", "partial", "failed")]
    total = len(valid)
    if not total:
        return {}
    n_succ  = sum(1 for r in valid if r["result"] == "success")
    n_part  = sum(1 for r in valid if r["result"] == "partial")
    n_fail  = sum(1 for r in valid if r["result"] == "failed")
    scores  = [r["score"] for r in valid]
    elaps   = [r["sim_elapsed_s"] for r in valid if r.get("sim_elapsed_s")]
    escapes = [r.get("escape_events", 0) for r in valid]
    mpis    = [r.get("mpi") or compute_mpi(r) for r in valid]

    return {
        "total":          total,
        "success":        n_succ,
        "partial":        n_part,
        "failed":         n_fail,
        "success_rate":   round(100 * n_succ / total, 1),
        "partial_rate":   round(100 * n_part / total, 1),
        "failed_rate":    round(100 * n_fail / total, 1),
        "avg_score":      round(sum(scores) / len(scores), 1),
        "best_score":     max(scores),
        "avg_sim_time_s": round(sum(elaps) / len(elaps), 1) if elaps else 0,
        "avg_escapes":    round(sum(escapes) / len(escapes), 2),
        "avg_mpi":        round(sum(mpis) / len(mpis), 4),
        "by_preset": {
            label: {
                "total":   sum(1 for r in valid if r["preset"] == label),
                "success": sum(1 for r in valid if r["preset"] == label and r["result"] == "success"),
                "partial": sum(1 for r in valid if r["preset"] == label and r["result"] == "partial"),
                "failed":  sum(1 for r in valid if r["preset"] == label and r["result"] == "failed"),
            }
            for label in ["easy", "medium", "hard", "very_hard"]
        },
    }


def print_summary(s: dict):
    if not s:
        print("\nNo valid results to summarise.")
        return
    print("\n" + "=" * 65)
    print("  SAR BATCH TEST V2 (STRESS) RESULTS SUMMARY")
    print("=" * 65)
    print(f"  Total missions  : {s['total']}")
    print(f"  SUCCESS         : {s['success']:3d}  ({s['success_rate']:5.1f}%)")
    print(f"  PARTIAL SUCCESS : {s['partial']:3d}  ({s['partial_rate']:5.1f}%)")
    print(f"  FAILED          : {s['failed']:3d}  ({s['failed_rate']:5.1f}%)")
    print(f"  Avg MPI         : {s['avg_mpi']}")
    print(f"  Avg score       : {s['avg_score']}")
    print(f"  Best score      : {s['best_score']}")
    print(f"  Avg sim time    : {s['avg_sim_time_s']}s")
    print(f"  Avg escape evts : {s['avg_escapes']}")
    print("\n  By difficulty:")
    for label, b in s["by_preset"].items():
        if b["total"]:
            sr = round(100 * b["success"] / b["total"], 1)
            print(f"    {label:<12} total={b['total']:3d}  success={b['success']:3d} ({sr}%)  "
                  f"partial={b['partial']:3d}  failed={b['failed']:3d}")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs",         type=int, default=30)
    parser.add_argument("--host",         default=DEFAULT_HOST)
    parser.add_argument("--out",          default=None,
                        help="Output JSON file (default: batch_results_<strategy>_v2.json)")
    parser.add_argument("--resume",       action="store_true")
    parser.add_argument("--uav-strategy", default="none",
                        choices=["none", "lawnmower", "threat_informed",
                                 "target_informed", "hybrid"])
    args = parser.parse_args()

    if args.out is None:
        args.out = f"batch_results_{args.uav_strategy}_v2.json"

    try:
        requests.get(f"{args.host}/api/sim/state", timeout=5)
    except Exception:
        print(f"ERROR: Server not reachable at {args.host}")
        print("Start the backend first: docker compose up  or  uvicorn main:app")
        return

    queue = build_run_queue(args.runs)

    results = []
    start_from = 0
    if args.resume and os.path.exists(args.out):
        try:
            with open(args.out) as f:
                prev = json.load(f)
            results = prev.get("results", [])
            start_from = len(results)
            print(f"Resuming from run {start_from + 1} ({start_from} already completed)")
        except Exception as e:
            print(f"Could not load previous results: {e}")

    queue = queue[start_from:]

    print(f"\nSAR Batch Test V2 (STRESS) - {args.runs} missions at {SIM_SPEED}x speed")
    print(f"UAV strategy: {args.uav_strategy}")
    print(f"Presets: {[(p['label'], p['n_patrol']+p['n_aggr'], p['radius']) for p in PRESETS]}")
    print(f"Host: {args.host}   Started: {datetime.now().strftime('%H:%M:%S')}\n")

    def save_incremental(results, out_path):
        s = summarise(results)
        out = {"generated": datetime.now().isoformat(), "runs": args.runs,
               "uav_strategy": args.uav_strategy, "preset_profile": "stress",
               "summary": s or {}, "results": results}
        with open(out_path, "w") as f:
            json.dump(out, f, indent=2)

    for i, preset in enumerate(queue, start=start_from + 1):
        row = run_mission(args.host, i, preset, uav_strategy=args.uav_strategy)
        results.append(row)
        save_incremental(results, args.out)
        if i % 10 == 0:
            s = summarise(results)
            if s:
                print(
                    f"\n  -- After {i} runs: "
                    f"success={s['success_rate']}% | mpi={s['avg_mpi']} | "
                    f"escapes={s['avg_escapes']}\n"
                )

    summary = summarise(results)
    print_summary(summary)

    out = {
        "generated":      datetime.now().isoformat(),
        "runs":           args.runs,
        "uav_strategy":   args.uav_strategy,
        "preset_profile": "stress",
        "summary":        summary,
        "results":        results,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n  Raw results -> {args.out}")

    csv_path = args.out.replace(".json", ".csv")
    if results:
        keys = ["run", "result", "preset", "uav_strategy", "preset_profile",
                "score", "mpi", "sim_elapsed_s", "wall_sec",
                "targets_collected", "targets_total", "escape_events",
                "n_patrol", "n_aggr", "radius"]
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(results)
    print(f"  CSV -> {csv_path}\n")


if __name__ == "__main__":
    main()
