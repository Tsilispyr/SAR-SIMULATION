"""
SAR UAV Strategy Comparison -- Statistical Significance Analysis

Loads all 10 batch_results_<strategy>[_v2].csv files (5 strategies x 2 density
profiles, N=30 unseeded stochastic replicates per cell -- no seed column exists
in the raw data, so these are independent stochastic replicates, not fixed
random seeds), recomputes MPI from the primitive columns (result, escape_events,
sim_elapsed_s) using the same formula as batch_test_v2.py, and runs real
significance tests on the strategy comparisons the paper's Discussion section
claims:
  - V1 (standard density): threat_informed vs. none (baseline)
  - V2 (stress density): each of the 4 UAV-equipped strategies vs. none

For each comparison: Welch's t-test and Mann-Whitney U on per-run MPI, plus a
normal-approximation 95% CI on the success-rate difference, plus the raw MPI
IQR per cell (checking the paper's current "IQR < 0.05" claim empirically).

Usage:
  python statistical_analysis.py
"""
import csv
import math
import os
import numpy as np
from scipy import stats

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STRATEGIES = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]

# MPI formula (matches batch_test_v2.py exactly)
MPI_W_OUTCOME = 0.50
MPI_W_SAFETY = 0.30
MPI_W_EFFICIENCY = 0.20
MPI_E_NORM = 5.0
MPI_T_NORM = 2880.0


def compute_mpi(row):
    outcome = {"success": 1.0, "partial": 0.5, "failed": 0.0}.get(row["result"], 0.0)
    escape_events = float(row["escape_events"])
    safety = max(0.0, 1.0 - escape_events / MPI_E_NORM)
    t = float(row["sim_elapsed_s"]) if row["sim_elapsed_s"] else MPI_T_NORM
    efficiency = max(0.0, 1.0 - t / MPI_T_NORM)
    return MPI_W_OUTCOME * outcome + MPI_W_SAFETY * safety + MPI_W_EFFICIENCY * efficiency


def load_cell(strategy, suffix=""):
    """Returns (mpi_array, success_rate, n) for one strategy/profile cell."""
    path = os.path.join(SCRIPT_DIR, f"batch_results_{strategy}{suffix}.csv")
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    mpi = np.array([compute_mpi(r) for r in rows])
    successes = sum(1 for r in rows if r["result"] == "success")
    return mpi, successes / len(rows), len(rows)


def success_rate_ci95(successes, n):
    """Normal-approximation 95% CI half-width on a binomial proportion."""
    p = successes / n
    se = math.sqrt(p * (1 - p) / n)
    return 1.96 * se


def cohens_d(a, b):
    """Cohen's d with pooled standard deviation (unequal N supported)."""
    n_a, n_b = len(a), len(b)
    pooled_var = ((n_a - 1) * a.var(ddof=1) + (n_b - 1) * b.var(ddof=1)) / (n_a + n_b - 2)
    pooled_std = math.sqrt(pooled_var)
    return (a.mean() - b.mean()) / pooled_std


def compare(name_a, mpi_a, sr_a, n_a, name_b, mpi_b, sr_b, n_b):
    t_stat, t_p = stats.ttest_ind(mpi_a, mpi_b, equal_var=False)
    u_stat, u_p = stats.mannwhitneyu(mpi_a, mpi_b, alternative="two-sided")
    d = cohens_d(mpi_a, mpi_b)
    ci_a = success_rate_ci95(int(round(sr_a * n_a)), n_a)
    ci_b = success_rate_ci95(int(round(sr_b * n_b)), n_b)
    iqr_a = np.percentile(mpi_a, 75) - np.percentile(mpi_a, 25)
    iqr_b = np.percentile(mpi_b, 75) - np.percentile(mpi_b, 25)
    print(f"\n--- {name_a} vs {name_b} ---")
    print(f"  {name_a}: mean MPI={mpi_a.mean():.4f} (IQR={iqr_a:.4f}), "
          f"success rate={sr_a*100:.1f}% (+/-{ci_a*100:.1f}pp, 95% CI)")
    print(f"  {name_b}: mean MPI={mpi_b.mean():.4f} (IQR={iqr_b:.4f}), "
          f"success rate={sr_b*100:.1f}% (+/-{ci_b*100:.1f}pp, 95% CI)")
    print(f"  Welch's t-test on MPI: t={t_stat:.3f}, p={t_p:.4f} "
          f"{'(significant at 0.05)' if t_p < 0.05 else '(NOT significant at 0.05)'}")
    print(f"  Mann-Whitney U on MPI: U={u_stat:.1f}, p={u_p:.4f} "
          f"{'(significant at 0.05)' if u_p < 0.05 else '(NOT significant at 0.05)'}")
    print(f"  Cohen's d = {d:.3f} ({'negligible' if abs(d) < 0.2 else 'small' if abs(d) < 0.5 else 'medium' if abs(d) < 0.8 else 'large'} effect)")
    print(f"  TABLE ROW: {name_a} vs. {name_b} | dMPI={mpi_a.mean()-mpi_b.mean():+.4f} | "
          f"t={t_stat:.2f} | p={t_p:.3f} | p_MW={u_p:.3f} | d={d:.2f}")


if __name__ == "__main__":
    print("Loading all 10 result files and recomputing MPI from primitives...")
    v1 = {s: load_cell(s, "") for s in STRATEGIES}
    v2 = {s: load_cell(s, "_v2") for s in STRATEGIES}

    print("\n" + "=" * 70)
    print("V1 (standard density, ~1.5/km^2): threat-informed vs. baseline")
    print("=" * 70)
    mpi_a, sr_a, n_a = v1["threat_informed"]
    mpi_b, sr_b, n_b = v1["none"]
    compare("threat_informed", mpi_a, sr_a, n_a, "none (baseline)", mpi_b, sr_b, n_b)

    print("\n" + "=" * 70)
    print("V2 (stress density, ~4.4/km^2): each UAV strategy vs. baseline")
    print("=" * 70)
    base_mpi, base_sr, base_n = v2["none"]
    for s in ["lawnmower", "threat_informed", "target_informed", "hybrid"]:
        mpi_s, sr_s, n_s = v2[s]
        compare(s, mpi_s, sr_s, n_s, "none (baseline)", base_mpi, base_sr, base_n)

    print("\n" + "=" * 70)
    print("All strategy-density cells: MPI IQR summary (checks 'IQR < 0.05' claim)")
    print("=" * 70)
    for profile_name, cells in [("V1", v1), ("V2", v2)]:
        for s in STRATEGIES:
            mpi, sr, n = cells[s]
            iqr = np.percentile(mpi, 75) - np.percentile(mpi, 25)
            print(f"  {profile_name} {s:16s}: N={n}, mean MPI={mpi.mean():.4f}, IQR={iqr:.4f}")
