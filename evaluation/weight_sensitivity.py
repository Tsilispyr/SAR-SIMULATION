"""
SAR UAV Strategy Comparison -- MPI Weight Sensitivity Analysis

The Mission Performance Index weights (0.50 outcome / 0.30 safety / 0.20
efficiency) are author-defined. This script checks how sensitive the V1
(standard density) strategy ranking is to that choice, directly answering:
"If a different SAR operator wanted to prioritize safety over time efficiency
even more, would the threat-informed strategy still come out on top?"

Recomputes MPI for all 5 strategies in V1 under the current weights and two
alternative, more safety-weighted vectors, and reports the resulting ranking
under each.

Usage:
  python weight_sensitivity.py
"""
import csv
import os
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STRATEGIES = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]

MPI_E_NORM = 5.0
MPI_T_NORM = 2880.0

WEIGHT_VECTORS = {
    "current (0.50/0.30/0.20)": (0.50, 0.30, 0.20),
    "safety-emphasized (0.40/0.45/0.15)": (0.40, 0.45, 0.15),
    "safety-dominant (0.35/0.50/0.15)": (0.35, 0.50, 0.15),
}


def load_rows(strategy, suffix=""):
    path = os.path.join(SCRIPT_DIR, f"batch_results_{strategy}{suffix}.csv")
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def mpi_components(row):
    outcome = {"success": 1.0, "partial": 0.5, "failed": 0.0}.get(row["result"], 0.0)
    escape_events = float(row["escape_events"])
    safety = max(0.0, 1.0 - escape_events / MPI_E_NORM)
    t = float(row["sim_elapsed_s"]) if row["sim_elapsed_s"] else MPI_T_NORM
    efficiency = max(0.0, 1.0 - t / MPI_T_NORM)
    return outcome, safety, efficiency


if __name__ == "__main__":
    print("Loading V1 (standard density) rows for all 5 strategies...")
    components = {}
    for s in STRATEGIES:
        rows = load_rows(s, "")
        components[s] = np.array([mpi_components(r) for r in rows])  # (N, 3)

    print("\n" + "=" * 78)
    print("V1 mean MPI and rank under each weight vector")
    print("=" * 78)
    rankings = {}
    for label, (w_out, w_saf, w_eff) in WEIGHT_VECTORS.items():
        means = {}
        for s in STRATEGIES:
            comp = components[s]
            mpi = w_out * comp[:, 0] + w_saf * comp[:, 1] + w_eff * comp[:, 2]
            means[s] = mpi.mean()
        ranked = sorted(means.items(), key=lambda kv: kv[1], reverse=True)
        rankings[label] = ranked
        print(f"\n  Weights: {label}")
        for rank, (s, m) in enumerate(ranked, start=1):
            marker = "  <-- rank 1" if rank == 1 else ""
            print(f"    {rank}. {s:16s} mean MPI={m:.4f}{marker}")

    print("\n" + "=" * 78)
    print("Answer to reviewer question: does threat-informed stay rank #1?")
    print("=" * 78)
    for label, ranked in rankings.items():
        top = ranked[0][0]
        print(f"  {label}: top strategy = {top} "
              f"({'threat_informed retains rank 1' if top == 'threat_informed' else 'ranking CHANGES -- top is now ' + top})")
