"""
SAR UAV Strategy Comparison -- Per-Tier Density Breakdown

The paper's central density-threshold claim rests on comparing two density
regimes (V1 ~1.5/km^2, V2 ~4.4/km^2) rather than a continuous sweep. This
script stratifies the EXISTING per-run data by difficulty tier (easy/medium/
hard/very_hard -- already recorded in the 'preset' column, no new simulations)
to check whether the V1-to-V2 success-rate degradation is uniform across
tiers or concentrated at specific ones, for the baseline and threat-informed
strategies. This gives the density-threshold discussion a finer-grained,
data-grounded basis without expanding the experimental scope.

Usage:
  python tier_breakdown.py
"""
import csv
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TIERS = ["easy", "medium", "hard", "very_hard"]
STRATEGIES = ["none", "threat_informed"]


def load_rows(strategy, suffix=""):
    path = os.path.join(SCRIPT_DIR, f"batch_results_{strategy}{suffix}.csv")
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def success_rate_by_tier(rows):
    out = {}
    for tier in TIERS:
        tier_rows = [r for r in rows if r["preset"] == tier]
        if not tier_rows:
            out[tier] = None
            continue
        successes = sum(1 for r in tier_rows if r["result"] == "success")
        out[tier] = (successes, len(tier_rows), successes / len(tier_rows))
    return out


if __name__ == "__main__":
    print("Per-tier success rate, V1 (standard) vs. V2 (stress), by strategy")
    print("=" * 78)
    for s in STRATEGIES:
        v1_rows = load_rows(s, "")
        v2_rows = load_rows(s, "_v2")
        v1_by_tier = success_rate_by_tier(v1_rows)
        v2_by_tier = success_rate_by_tier(v2_rows)
        print(f"\n--- {s} ---")
        print(f"  {'Tier':10s} {'V1 (n succ/N, %)':22s} {'V2 (n succ/N, %)':22s} {'Delta (pp)':10s}")
        for tier in TIERS:
            v1 = v1_by_tier[tier]
            v2 = v2_by_tier[tier]
            v1_str = f"{v1[0]}/{v1[1]} ({v1[2]*100:.1f}%)" if v1 else "n/a"
            v2_str = f"{v2[0]}/{v2[1]} ({v2[2]*100:.1f}%)" if v2 else "n/a"
            delta = (v2[2] - v1[2]) * 100 if v1 and v2 else None
            delta_str = f"{delta:+.1f}" if delta is not None else "n/a"
            print(f"  {tier:10s} {v1_str:22s} {v2_str:22s} {delta_str:10s}")

    print("\n" + "=" * 78)
    print("Interpretation check: is the degradation monotonic across tiers?")
    print("=" * 78)
    for s in STRATEGIES:
        v1_rows = load_rows(s, "")
        v2_rows = load_rows(s, "_v2")
        v1_by_tier = success_rate_by_tier(v1_rows)
        v2_by_tier = success_rate_by_tier(v2_rows)
        deltas = []
        for tier in TIERS:
            v1 = v1_by_tier[tier]
            v2 = v2_by_tier[tier]
            if v1 and v2:
                deltas.append((tier, (v2[2] - v1[2]) * 100))
        print(f"  {s}: deltas by tier (easy->very_hard) = {[round(d, 1) for _, d in deltas]}")
