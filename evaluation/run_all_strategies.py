"""
SAR Strategy Comparison Runner
Runs all UAV strategies back-to-back with health checks and cooldowns between batches.

Usage:
  python run_all_strategies.py               # 30 runs per strategy, all 5
  python run_all_strategies.py --runs 50     # 50 runs per strategy
  python run_all_strategies.py --skip none lawnmower   # skip already-done strategies
  python run_all_strategies.py --only hybrid           # run just one

Between batches, waits for the backend to idle before starting the next one.
The OSMnx Overpass cache is populated on the first strategy run; subsequent
strategies reuse it and are much faster to load.
"""
import subprocess
import sys
import time
import json
import os
import argparse
import requests
from datetime import datetime, timedelta

STRATEGIES   = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]
DEFAULT_HOST = "http://localhost:8000"
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
BATCH_SCRIPT = os.path.join(SCRIPT_DIR, "batch_test.py")

# Cooldown between strategies: let backend GC settle, avoids OSMnx hammering Overpass
INTER_BATCH_COOLDOWN = 30   # seconds to wait after one batch before starting next
BACKEND_POLL_INTERVAL = 5   # seconds between /api/sim/state health checks
BACKEND_MAX_WAIT      = 120 # seconds to wait for backend to become idle

CYAN  = "\033[96m"
GREEN = "\033[92m"
YELLOW= "\033[93m"
RED   = "\033[91m"
BOLD  = "\033[1m"
END   = "\033[0m"


def print_banner(msg: str, color=CYAN):
    w = 70
    print(f"\n{color}{BOLD}{'═'*w}")
    print(f"  {msg}")
    print(f"{'═'*w}{END}\n")


def backend_idle(host: str) -> bool:
    """Return True when backend responds AND simulation is not running."""
    try:
        r = requests.get(f"{host}/api/sim/state", timeout=8)
        state = r.json()
        return not state.get("running", False)
    except Exception:
        return False


def wait_for_backend_idle(host: str, label: str = ""):
    """Block until backend is reachable and idle, or until BACKEND_MAX_WAIT."""
    t0 = time.time()
    print(f"  Waiting for backend to idle {label}...", end="", flush=True)
    while time.time() - t0 < BACKEND_MAX_WAIT:
        if backend_idle(host):
            elapsed = time.time() - t0
            print(f" ready ({elapsed:.0f}s)")
            return True
        time.sleep(BACKEND_POLL_INTERVAL)
        print(".", end="", flush=True)
    print(f" {RED}TIMEOUT{END}")
    return False


def result_file(strategy: str) -> str:
    return os.path.join(SCRIPT_DIR, f"batch_results_{strategy}.json")


def load_summary(strategy: str) -> dict | None:
    path = result_file(strategy)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        return data.get("summary", {})
    except Exception:
        return None


def run_strategy(strategy: str, runs: int, host: str) -> bool:
    """Launch batch_test.py as subprocess. Returns True on clean exit."""
    out_file = result_file(strategy)
    cmd = [
        sys.executable, BATCH_SCRIPT,
        "--runs",         str(runs),
        "--uav-strategy", strategy,
        "--out",          out_file,
        "--host",         host,
    ]
    print_banner(f"STRATEGY: {strategy.upper()}  ({runs} missions)", CYAN)
    print(f"  Command: {' '.join(cmd)}\n")

    start = datetime.now()
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proc = subprocess.run(cmd, cwd=SCRIPT_DIR, env=env)
    elapsed = datetime.now() - start

    if proc.returncode == 0:
        print(f"\n{GREEN}  [✓] {strategy} completed in {elapsed}{END}")
        return True
    else:
        print(f"\n{RED}  [✗] {strategy} exited with code {proc.returncode}{END}")
        return False


def print_comparison_table(strategies: list[str]):
    """Print a side-by-side summary table of all completed strategies."""
    summaries = {s: load_summary(s) for s in strategies}
    completed = {s: sm for s, sm in summaries.items() if sm}
    if not completed:
        return

    print_banner("RESULTS COMPARISON", GREEN)
    col = 14
    hdr = f"{'Metric':<22}" + "".join(f"{s:>{col}}" for s in completed)
    print(f"  {hdr}")
    print(f"  {'-'*( 22 + col * len(completed))}")

    def row(label, fn):
        vals = "".join(f"{fn(sm):>{col}}" for sm in completed.values())
        print(f"  {label:<22}{vals}")

    row("Total runs",      lambda s: str(s.get("total", "—")))
    row("Success %",       lambda s: f"{s.get('success_rate', '—')}%")
    row("Partial %",       lambda s: f"{s.get('partial_rate', '—')}%")
    row("Failed %",        lambda s: f"{s.get('failed_rate', '—')}%")
    row("Avg score",       lambda s: str(s.get("avg_score", "—")))
    row("Best score",      lambda s: str(s.get("best_score", "—")))
    row("Avg sim time (s)",lambda s: str(s.get("avg_sim_time_s", "—")))
    row("Avg escape evts", lambda s: str(s.get("avg_escapes", "—")))

    print()
    # Highlight winner by success rate
    best = max(completed, key=lambda s: completed[s].get("success_rate", 0))
    print(f"  {GREEN}{BOLD}Best strategy: {best} "
          f"({completed[best].get('success_rate','?')}% success){END}")
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs",  type=int, default=30,
                        help="Runs per strategy (default: 30)")
    parser.add_argument("--host",  default=DEFAULT_HOST)
    parser.add_argument("--skip",  nargs="+", default=[],
                        choices=STRATEGIES, metavar="STRATEGY",
                        help="Strategies to skip (already completed)")
    parser.add_argument("--only",  nargs="+", default=None,
                        choices=STRATEGIES, metavar="STRATEGY",
                        help="Run only these strategies")
    parser.add_argument("--cooldown", type=int, default=INTER_BATCH_COOLDOWN,
                        help=f"Seconds between batches (default: {INTER_BATCH_COOLDOWN})")
    args = parser.parse_args()

    todo = args.only if args.only else STRATEGIES
    todo = [s for s in todo if s not in args.skip]

    # Auto-skip strategies with complete result files
    already_done = []
    for s in list(todo):
        sm = load_summary(s)
        if sm and sm.get("total", 0) >= args.runs:
            already_done.append(s)
    if already_done:
        print(f"{YELLOW}  Auto-skipping (results already exist with >= {args.runs} runs): "
              f"{', '.join(already_done)}{END}")
        todo = [s for s in todo if s not in already_done]

    if not todo:
        print("\nAll strategies already completed. Showing comparison:\n")
        print_comparison_table(STRATEGIES)
        return

    print_banner(
        f"SAR Strategy Comparison — {args.runs} runs × {len(todo)} strategies "
        f"= {args.runs * len(todo)} total missions", BOLD
    )
    print(f"  Strategies: {todo}")
    print(f"  Cooldown between batches: {args.cooldown}s")
    print(f"  Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    # Verify backend is up
    print("  Checking backend...", end="", flush=True)
    if not wait_for_backend_idle(args.host):
        print(f"{RED}Backend not reachable at {args.host}. Start it first.{END}")
        sys.exit(1)

    failed_strategies = []
    for idx, strategy in enumerate(todo):
        ok = run_strategy(strategy, args.runs, args.host)
        if not ok:
            failed_strategies.append(strategy)

        # Cooldown between batches (skip after last one)
        if idx < len(todo) - 1:
            print(f"\n  {YELLOW}Cooldown: waiting {args.cooldown}s before next batch...{END}")
            time.sleep(args.cooldown)
            wait_for_backend_idle(args.host, label=f"(before {todo[idx+1]})")

    # Final comparison table (include all 5 strategies if files exist)
    all_ran = STRATEGIES if args.only is None else args.only
    print_comparison_table(all_ran)

    if failed_strategies:
        print(f"{RED}  Strategies with errors: {failed_strategies}{END}")
        print(f"  Re-run with: --only {' '.join(failed_strategies)}\n")
    else:
        print(f"{GREEN}{BOLD}  All strategies completed successfully.{END}")

    print(f"  Results in: {SCRIPT_DIR}/batch_results_<strategy>.json\n")


if __name__ == "__main__":
    main()
