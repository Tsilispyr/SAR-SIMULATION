"""
Generate poster-quality charts from batch_results.json
Usage: python plot_results.py [batch_results.json]
Outputs: results_pie.png, results_by_difficulty.png, results_score_dist.png
"""
import json
import sys
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

INPUT = sys.argv[1] if len(sys.argv) > 1 else "batch_results.json"
OUT_DIR = os.path.dirname(os.path.abspath(INPUT))

with open(INPUT) as f:
    data = json.load(f)

results = data["results"]
summary = data["summary"]
valid   = [r for r in results if r.get("result") in ("success", "partial", "failed")]

COLORS = {
    "success": "#2ecc71",
    "partial": "#f39c12",
    "failed":  "#e74c3c",
}
BG = "#1a1a2e"
FG = "#e0e0e0"

def style_ax(ax):
    ax.set_facecolor("#16213e")
    ax.tick_params(colors=FG)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.title.set_color(FG)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")

# == 1. Outcome pie =========
fig, ax = plt.subplots(figsize=(5, 5), facecolor=BG)
labels  = ["Success", "Partial\nSuccess", "Failed"]
vals    = [summary["success"], summary["partial"], summary["failed"]]
colors  = [COLORS["success"], COLORS["partial"], COLORS["failed"]]
wedges, texts, autotexts = ax.pie(
    vals, labels=labels, colors=colors, autopct="%1.1f%%",
    startangle=90, pctdistance=0.75,
    wedgeprops={"edgecolor": BG, "linewidth": 2},
)
for t in texts:
    t.set_color(FG); t.set_fontsize(11)
for at in autotexts:
    at.set_color("white"); at.set_fontweight("bold"); at.set_fontsize(10)
ax.set_title(f"Mission Outcomes  (n={summary['total']})", color=FG, fontsize=13, pad=14)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "results_pie.png"), dpi=150, bbox_inches="tight", facecolor=BG)
plt.close()
print("Saved results_pie.png")

# == 2. Grouped bar by difficulty ======================
presets = ["easy", "medium", "hard", "very_hard"]
labels2 = ["Easy", "Medium", "Hard", "Very Hard"]
bp      = summary.get("by_preset", {})

succ_vals  = [bp.get(p, {}).get("success", 0) for p in presets]
part_vals  = [bp.get(p, {}).get("partial", 0) for p in presets]
fail_vals  = [bp.get(p, {}).get("failed",  0) for p in presets]

x  = np.arange(len(presets))
w  = 0.25

fig, ax = plt.subplots(figsize=(7, 4), facecolor=BG)
style_ax(ax)
ax.bar(x - w,   succ_vals, w, label="Success",        color=COLORS["success"], zorder=3)
ax.bar(x,       part_vals, w, label="Partial Success", color=COLORS["partial"], zorder=3)
ax.bar(x + w,   fail_vals, w, label="Failed",          color=COLORS["failed"],  zorder=3)
ax.set_xticks(x); ax.set_xticklabels(labels2, color=FG)
ax.set_ylabel("Count", color=FG); ax.set_xlabel("Difficulty", color=FG)
ax.set_title("Outcomes by Difficulty", color=FG, fontsize=13)
ax.legend(facecolor="#16213e", edgecolor="#444", labelcolor=FG, fontsize=9)
ax.grid(axis="y", alpha=0.2, zorder=0)
ax.set_facecolor("#16213e")
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "results_by_difficulty.png"), dpi=150, bbox_inches="tight", facecolor=BG)
plt.close()
print("Saved results_by_difficulty.png")

# == 3. Score distribution histogram ================
scores = [r["score"] for r in valid]
fig, ax = plt.subplots(figsize=(6, 4), facecolor=BG)
style_ax(ax)
n_bins = 20
ax.hist(scores, bins=n_bins, color="#3498db", edgecolor=BG, zorder=3)
ax.axvline(np.mean(scores), color="#e74c3c", linestyle="--", lw=1.5,
           label=f"Mean = {np.mean(scores):.0f}")
ax.axvline(np.median(scores), color=COLORS["partial"], linestyle=":", lw=1.5,
           label=f"Median = {np.median(scores):.0f}")
ax.set_xlabel("Score", color=FG); ax.set_ylabel("Frequency", color=FG)
ax.set_title("Score Distribution", color=FG, fontsize=13)
ax.legend(facecolor="#16213e", edgecolor="#444", labelcolor=FG, fontsize=9)
ax.grid(alpha=0.15, zorder=0)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "results_score_dist.png"), dpi=150, bbox_inches="tight", facecolor=BG)
plt.close()
print("Saved results_score_dist.png")

# == 4. Success rate vs difficulty (line chart) ===========================
totals = [bp.get(p, {}).get("total", 0) for p in presets]
sr     = [100 * bp.get(p, {}).get("success", 0) / max(t, 1) for p, t in zip(presets, totals)]
pr     = [100 * bp.get(p, {}).get("partial", 0) / max(t, 1) for p, t in zip(presets, totals)]
fr     = [100 * bp.get(p, {}).get("failed",  0) / max(t, 1) for p, t in zip(presets, totals)]

fig, ax = plt.subplots(figsize=(6, 4), facecolor=BG)
style_ax(ax)
ax.plot(labels2, sr, "o-", color=COLORS["success"], lw=2, ms=7, label="Success")
ax.plot(labels2, pr, "s-", color=COLORS["partial"], lw=2, ms=7, label="Partial")
ax.plot(labels2, fr, "^-", color=COLORS["failed"],  lw=2, ms=7, label="Failed")
ax.set_ylim(0, 105)
ax.set_xlabel("Difficulty", color=FG); ax.set_ylabel("Rate (%)", color=FG)
ax.set_title("Success Rate vs Difficulty", color=FG, fontsize=13)
ax.legend(facecolor="#16213e", edgecolor="#444", labelcolor=FG, fontsize=9)
ax.grid(alpha=0.15, zorder=0)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "results_rate_vs_difficulty.png"), dpi=150, bbox_inches="tight", facecolor=BG)
plt.close()
print("Saved results_rate_vs_difficulty.png")

# == Console summary =============================
print(f"\n{'='*50}")
print(f"  Total: {summary['total']}  |  Success: {summary['success_rate']}%"
      f"  |  Partial: {summary['partial_rate']}%  |  Failed: {summary['failed_rate']}%")
print(f"  Avg score: {summary['avg_score']}  |  Best: {summary['best_score']}")
print(f"  Avg sim time: {summary['avg_sim_time_s']}s  |  Avg escapes: {summary['avg_escapes']}")
print(f"{'='*50}\n")
