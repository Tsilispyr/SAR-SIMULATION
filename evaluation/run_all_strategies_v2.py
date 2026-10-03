"""
SAR Strategy Comparison Runner — V2 (Stress Profile)
Runs all UAV strategies using batch_test_v2.py (high enemy density).
Outputs: batch_results_{strategy}_v2.json / .csv

Usage:
  python run_all_strategies_v2.py               # 30 runs per strategy
  python run_all_strategies_v2.py --runs 50
  python run_all_strategies_v2.py --skip none
  python run_all_strategies_v2.py --only hybrid
"""
import subprocess
import sys
import time
import json
import os
import argparse
import requests
from datetime import datetime

STRATEGIES   = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]
DEFAULT_HOST = "http://localhost:8000"
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
BATCH_SCRIPT = os.path.join(SCRIPT_DIR, "batch_test_v2.py")

INTER_BATCH_COOLDOWN  = 30
BACKEND_POLL_INTERVAL = 5
BACKEND_MAX_WAIT      = 120

CYAN  = "\033[96m"
GREEN = "\033[92m"
YELLOW= "\033[93m"
RED   = "\033[91m"
BOLD  = "\033[1m"
END   = "\033[0m"


def print_banner(msg: str, color=CYAN):
    w = 70
    print(f"\n{color}{BOLD}{'='*w}")
    print(f"  {msg}")
    print(f"{'='*w}{END}\n")


def backend_idle(host: str) -> bool:
    try:
        r = requests.get(f"{host}/api/sim/state", timeout=8)
        return not r.json().get("running", False)
    except Exception:
        return False


def wait_for_backend_idle(host: str, label: str = ""):
    t0 = time.time()
    print(f"  Waiting for backend to idle {label}...", end="", flush=True)
    while time.time() - t0 < BACKEND_MAX_WAIT:
        if backend_idle(host):
            print(f" ready ({time.time()-t0:.0f}s)")
            return True
        time.sleep(BACKEND_POLL_INTERVAL)
        print(".", end="", flush=True)
    print(f" {RED}TIMEOUT{END}")
    return False


def result_file(strategy: str) -> str:
    return os.path.join(SCRIPT_DIR, f"batch_results_{strategy}_v2.json")


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
    out_file = result_file(strategy)
    cmd = [
        sys.executable, BATCH_SCRIPT,
        "--runs",         str(runs),
        "--uav-strategy", strategy,
        "--out",          out_file,
        "--host",         host,
    ]
    print_banner(f"V2 STRATEGY: {strategy.upper()}  ({runs} missions)", CYAN)
    print(f"  Command: {' '.join(cmd)}\n")

    start = datetime.now()
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proc = subprocess.run(cmd, cwd=SCRIPT_DIR, env=env)
    elapsed = datetime.now() - start

    if proc.returncode == 0:
        print(f"\n{GREEN}  [+] {strategy} completed in {elapsed}{END}")
        return True
    else:
        print(f"\n{RED}  [x] {strategy} exited with code {proc.returncode}{END}")
        return False


def print_comparison_table(strategies: list[str]):
    summaries = {s: load_summary(s) for s in strategies}
    completed = {s: sm for s, sm in summaries.items() if sm}
    if not completed:
        return

    print_banner("V2 STRESS RESULTS COMPARISON", GREEN)
    col = 14
    hdr = f"{'Metric':<22}" + "".join(f"{s:>{col}}" for s in completed)
    print(f"  {hdr}")
    print(f"  {'-'*(22 + col * len(completed))}")

    def row(label, fn):
        vals = "".join(f"{fn(sm):>{col}}" for sm in completed.values())
        print(f"  {label:<22}{vals}")

    row("Total runs",       lambda s: str(s.get("total", "-")))
    row("Success %",        lambda s: f"{s.get('success_rate', '-')}%")
    row("Partial %",        lambda s: f"{s.get('partial_rate', '-')}%")
    row("Failed %",         lambda s: f"{s.get('failed_rate', '-')}%")
    row("Avg MPI",          lambda s: str(s.get("avg_mpi", "-")))
    row("Avg score",        lambda s: str(s.get("avg_score", "-")))
    row("Best score",       lambda s: str(s.get("best_score", "-")))
    row("Avg sim time (s)", lambda s: str(s.get("avg_sim_time_s", "-")))
    row("Avg escape evts",  lambda s: str(s.get("avg_escapes", "-")))

    print()
    best_sr  = max(completed, key=lambda s: completed[s].get("success_rate", 0))
    best_mpi = max(completed, key=lambda s: completed[s].get("avg_mpi", 0))
    print(f"  {GREEN}{BOLD}Best success rate : {best_sr} ({completed[best_sr].get('success_rate','?')}%){END}")
    print(f"  {GREEN}{BOLD}Best MPI          : {best_mpi} ({completed[best_mpi].get('avg_mpi','?')}){END}")
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs",     type=int, default=30)
    parser.add_argument("--host",     default=DEFAULT_HOST)
    parser.add_argument("--skip",     nargs="+", default=[], choices=STRATEGIES, metavar="STRATEGY")
    parser.add_argument("--only",     nargs="+", default=None,  choices=STRATEGIES, metavar="STRATEGY")
    parser.add_argument("--cooldown", type=int, default=INTER_BATCH_COOLDOWN)
    args = parser.parse_args()

    todo = args.only if args.only else STRATEGIES
    todo = [s for s in todo if s not in args.skip]

    already_done = [s for s in list(todo)
                    if (sm := load_summary(s)) and sm.get("total", 0) >= args.runs]
    if already_done:
        print(f"{YELLOW}  Auto-skipping (v2 results exist with >= {args.runs} runs): "
              f"{', '.join(already_done)}{END}")
        todo = [s for s in todo if s not in already_done]

    if not todo:
        print("\nAll V2 strategies already completed. Showing comparison:\n")
        print_comparison_table(STRATEGIES)
        return

    print_banner(
        f"SAR Strategy Comparison V2 (STRESS) — {args.runs} runs x {len(todo)} strategies "
        f"= {args.runs * len(todo)} total missions", BOLD
    )
    print(f"  Strategies : {todo}")
    print(f"  Cooldown   : {args.cooldown}s between batches")
    print(f"  Started    : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    print("  Checking backend...", end="", flush=True)
    if not wait_for_backend_idle(args.host):
        print(f"{RED}Backend not reachable at {args.host}. Start it first.{END}")
        sys.exit(1)

    failed = []
    for idx, strategy in enumerate(todo):
        ok = run_strategy(strategy, args.runs, args.host)
        if not ok:
            failed.append(strategy)
        if idx < len(todo) - 1:
            print(f"\n  {YELLOW}Cooldown: waiting {args.cooldown}s...{END}")
            time.sleep(args.cooldown)
            wait_for_backend_idle(args.host, label=f"(before {todo[idx+1]})")

    all_ran = STRATEGIES if args.only is None else args.only
    print_comparison_table(all_ran)

    if failed:
        print(f"{RED}  Failed: {failed}{END}")
        print(f"  Retry with: --only {' '.join(failed)}\n")
    else:
        print(f"{GREEN}{BOLD}  All V2 strategies completed.{END}")
    print(f"  Results: {SCRIPT_DIR}/batch_results_<strategy>_v2.json\n")


if __name__ == "__main__":
    main()
