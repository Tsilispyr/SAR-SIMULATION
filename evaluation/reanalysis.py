"""
SAR UAV Strategy Comparison -- Re-analysis (October 2026 review)

Re-derives every headline number of the 2x5 strategy study directly from the raw
batch_results_<strategy>[_v2].csv files and adds the checks the original analysis
scripts do not cover:

  - success-rate tests on the binary outcome itself (Fisher's exact per strategy,
    chi-square omnibus across the 5 conditions, pooled UAV vs. baseline),
    with Wilson 95% intervals;
  - Kruskal-Wallis omnibus on per-run MPI;
  - MPI by outcome class (does the composite ever rank a failure above a partial?);
  - escape-event distribution vs. the E_max = 5 normalisation constant;
  - timeouts (censored runs recorded as failures);
  - effective simulation speed per cell (sim_elapsed_s / wall_sec), which exposes
    cells that were generated at a different SIM_SPEED setting;
  - runs per arm needed to detect a given success-rate gap.

Usage:
  python reanalysis.py
"""
import csv
import math
import os
import numpy as np
from scipy import stats

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STRATEGIES = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]
PROFILES = [("V1", ""), ("V2", "_v2")]

# MPI formula (matches batch_test_v2.py exactly)
MPI_E_NORM = 5.0
MPI_T_NORM = 2880.0


def compute_mpi(row):
    outcome = {"success": 1.0, "partial": 0.5, "failed": 0.0}.get(row["result"], 0.0)
    safety = max(0.0, 1.0 - float(row["escape_events"]) / MPI_E_NORM)
    t = float(row["sim_elapsed_s"]) if row["sim_elapsed_s"] else MPI_T_NORM
    efficiency = max(0.0, 1.0 - t / MPI_T_NORM)
    return 0.50 * outcome + 0.30 * safety + 0.20 * efficiency


def load_rows(strategy, suffix=""):
    path = os.path.join(SCRIPT_DIR, f"batch_results_{strategy}{suffix}.csv")
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def wilson(k, n, z=1.96):
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return centre - half, centre + half


def runs_per_arm(p1, p2, z_alpha=1.959964, z_beta=0.841621):
    """Two-sided two-proportion test, alpha = 0.05, power = 0.80."""
    p_bar = (p1 + p2) / 2
    num = (z_alpha * math.sqrt(2 * p_bar * (1 - p_bar))
           + z_beta * math.sqrt(p1 * (1 - p1) + p2 * (1 - p2))) ** 2
    return math.ceil(num / (p1 - p2) ** 2)


if __name__ == "__main__":
    cells = {}
    print("=" * 104)
    print("Per-cell summary (recomputed from CSV; timeouts = rows with sim_elapsed_s >= 2880)")
    print("=" * 104)
    print(f"{'cell':22s} {'N':>3s} {'succ':>4s} {'part':>4s} {'fail':>4s} {'tmo':>3s} "
          f"{'SR%':>6s} {'Wilson 95%':>14s} {'MPI':>6s} {'esc':>5s} {'speed':>6s}")
    for profile, suffix in PROFILES:
        for s in STRATEGIES:
            rows = load_rows(s, suffix)
            n = len(rows)
            k = sum(r["result"] == "success" for r in rows)
            part = sum(r["result"] == "partial" for r in rows)
            fail = sum(r["result"] == "failed" for r in rows)
            timeouts = sum(float(r["sim_elapsed_s"] or 0) >= MPI_T_NORM for r in rows)
            mpi = np.array([compute_mpi(r) for r in rows])
            esc = np.mean([float(r["escape_events"]) for r in rows])
            done = [r for r in rows if float(r["sim_elapsed_s"] or 0) < MPI_T_NORM]
            speed = np.mean([float(r["sim_elapsed_s"]) / float(r["wall_sec"]) for r in done])
            lo, hi = wilson(k, n)
            cells[(profile, s)] = {"rows": rows, "n": n, "k": k, "mpi": mpi}
            print(f"{profile + ' ' + s:22s} {n:3d} {k:4d} {part:4d} {fail:4d} {timeouts:3d} "
                  f"{100 * k / n:6.1f} [{100 * lo:5.1f},{100 * hi:5.1f}] {mpi.mean():6.3f} "
                  f"{esc:5.2f} {speed:5.2f}x")
    print("  speed = mean sim_elapsed_s / wall_sec over non-timeout runs; at SIM_SPEED = 4 it")
    print("  cannot exceed 4.0x, so a cell well above 4x was generated at a higher SIM_SPEED.")

    print("\n" + "=" * 104)
    print("Strategy vs. baseline: Fisher's exact on success rate; Welch t and Mann-Whitney U on MPI")
    print("=" * 104)
    for profile, _ in PROFILES:
        base = cells[(profile, "none")]
        for s in STRATEGIES[1:]:
            c = cells[(profile, s)]
            _, p_fisher = stats.fisher_exact([[c["k"], c["n"] - c["k"]],
                                              [base["k"], base["n"] - base["k"]]])
            _, p_welch = stats.ttest_ind(c["mpi"], base["mpi"], equal_var=False)
            _, p_mwu = stats.mannwhitneyu(c["mpi"], base["mpi"], alternative="two-sided")
            print(f"  {profile} {s:16s} SR {100 * c['k'] / c['n']:5.1f}% vs {100 * base['k'] / base['n']:5.1f}%"
                  f"  Fisher p={p_fisher:.3f} | dMPI={c['mpi'].mean() - base['mpi'].mean():+.4f}"
                  f"  Welch p={p_welch:.3f}  MWU p={p_mwu:.3f}")

    print("\nOmnibus across all 5 conditions:")
    for profile, _ in PROFILES:
        table = [[cells[(profile, s)]["k"], cells[(profile, s)]["n"] - cells[(profile, s)]["k"]]
                 for s in STRATEGIES]
        chi2, p_chi, dof, _ = stats.chi2_contingency(table)
        kw = stats.kruskal(*[cells[(profile, s)]["mpi"] for s in STRATEGIES])
        print(f"  {profile}: chi-square (success) = {chi2:.2f}, dof = {dof}, p = {p_chi:.3f} | "
              f"Kruskal-Wallis (MPI) H = {kw.statistic:.2f}, p = {kw.pvalue:.3f}")

    print("\nPooled UAV conditions vs. baseline (success rate):")
    for profile, _ in PROFILES:
        k = sum(cells[(profile, s)]["k"] for s in STRATEGIES[1:])
        n = sum(cells[(profile, s)]["n"] for s in STRATEGIES[1:])
        base = cells[(profile, "none")]
        _, p = stats.fisher_exact([[k, n - k], [base["k"], base["n"] - base["k"]]])
        print(f"  {profile}: UAV {k}/{n} = {100 * k / n:.1f}% vs baseline "
              f"{base['k']}/{base['n']} = {100 * base['k'] / base['n']:.1f}%  (Fisher p = {p:.3f})")

    print("\n" + "=" * 104)
    print("MPI by outcome class")
    print("=" * 104)
    for profile, _ in PROFILES:
        rows = [r for s in STRATEGIES for r in cells[(profile, s)]["rows"]]
        for result in ["success", "partial", "failed"]:
            vals = [compute_mpi(r) for r in rows if r["result"] == result]
            print(f"  {profile} {result:8s} n={len(vals):3d}  MPI mean={np.mean(vals):.3f}  "
                  f"min={min(vals):.3f}  max={max(vals):.3f}")
        best_fail = max((r for r in rows if r["result"] == "failed"), key=compute_mpi)
        print(f"  {profile} highest-MPI failure: MPI={compute_mpi(best_fail):.3f} at "
              f"t={best_fail['sim_elapsed_s']} s with {best_fail['targets_collected']}/"
              f"{best_fail['targets_total']} targets")

    print("\nEscape events vs. E_max = 5:")
    for profile, _ in PROFILES:
        esc = [int(float(r["escape_events"])) for s in STRATEGIES for r in cells[(profile, s)]["rows"]]
        print(f"  {profile}: runs with 0 escapes = {sum(e == 0 for e in esc)}/{len(esc)}, "
              f"runs with > 5 escapes = {sum(e > 5 for e in esc)}, max = {max(esc)}")

    print("\n" + "=" * 104)
    print("Runs per arm for 80% power (alpha = 0.05, two-sided) to detect a success-rate gap")
    print("=" * 104)
    for p1, p2 in [(0.867, 0.767), (0.933, 0.833), (0.70, 0.80), (0.80, 0.90)]:
        print(f"  {p1:.3f} vs {p2:.3f}: {runs_per_arm(p1, p2)} runs per arm")
