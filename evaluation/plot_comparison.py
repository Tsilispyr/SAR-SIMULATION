"""
SAR UAV Strategy Comparison Charts
Backfills MPI into V1 result files, then generates comparison charts for V1 vs V2.

Output files:
  comparison_bars.png        -- Success rate, MPI, escape events (V1 vs V2)
  comparison_heatmap.png     -- Success rate by difficulty tier (V1 and V2 side by side)

Usage:
  python plot_comparison.py
"""
import json
import glob
import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ── MPI formula (matches batch_test_v2.py exactly) ───────────────────────────
MPI_E_NORM = 5.0
MPI_T_NORM = 2880.0

def compute_mpi(row: dict) -> float:
    outcome = {"success": 1.0, "partial": 0.5, "failed": 0.0}.get(
        row.get("result", "failed"), 0.0)
    safety     = max(0.0, 1.0 - row.get("escape_events", 0) / MPI_E_NORM)
    t          = row.get("sim_elapsed_s") or MPI_T_NORM
    efficiency = max(0.0, 1.0 - t / MPI_T_NORM)
    return round(0.50 * outcome + 0.30 * safety + 0.20 * efficiency, 4)

# ── Backfill MPI into V1 JSON files ──────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STRATEGIES = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]

print("Backfilling MPI into V1 files...")
for s in STRATEGIES:
    path = os.path.join(SCRIPT_DIR, f"batch_results_{s}.json")
    if not os.path.exists(path):
        continue
    with open(path) as f:
        d = json.load(f)
    results = d.get("results", [])
    for r in results:
        if "mpi" not in r:
            r["mpi"] = compute_mpi(r)
    valid = [r for r in results if r.get("result") in ("success", "partial", "failed")]
    if valid:
        d["summary"]["avg_mpi"] = round(
            sum(r.get("mpi", compute_mpi(r)) for r in valid) / len(valid), 4)
    with open(path, "w") as f:
        json.dump(d, f, indent=2)
    print(f"  {s}: avg_mpi = {d['summary'].get('avg_mpi')}")

# ── Load all data ─────────────────────────────────────────────────────────────
def load(strategy, suffix=""):
    path = os.path.join(SCRIPT_DIR, f"batch_results_{strategy}{suffix}.json")
    if not os.path.exists(path):
        return {}, []
    with open(path) as f:
        d = json.load(f)
    return d.get("summary", {}), d.get("results", [])

v1 = {s: load(s, "")   for s in STRATEGIES}
v2 = {s: load(s, "_v2") for s in STRATEGIES}

# ── Design tokens (from skill reference palette) ──────────────────────────────
BG      = "#fcfcfb"
C_V1    = "#2a78d6"   # blue  – slot 1
C_V2    = "#eb6834"   # orange – slot 6
C_GRID  = "#e1e0d9"
C_AXIS  = "#c3c2b7"
C_TEXT  = "#0b0b0b"
C_SEC   = "#52514e"
C_MUTED = "#898781"

STRATEGY_LABELS = [
    "None\n(Baseline)", "Lawnmower", "Threat-\nInformed",
    "Target-\nInformed", "Hybrid",
]
# Shorter single-line labels for the print-narrow comparison figures (Figures 2-3 in the
# paper, embedded at only 3.0 in wide) -- the two-line wrapped labels above collide with
# each other once the per-panel font size is large enough to be legible at that width.
STRATEGY_LABELS_COMPACT = ["None", "Lawn.", "Threat-Inf.", "Target-Inf.", "Hybrid"]
TIER_LABELS = ["Easy", "Medium", "Hard", "Very Hard"]
TIERS       = ["easy", "medium", "hard", "very_hard"]

matplotlib.rcParams.update({
    "font.family":       "sans-serif",
    "font.size":         10,
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "axes.edgecolor":    C_AXIS,
    "axes.grid":         True,
    "grid.color":        C_GRID,
    "grid.linewidth":    0.7,
    "axes.axisbelow":    True,
    "figure.facecolor":  BG,
    "axes.facecolor":    BG,
    "text.color":        C_TEXT,
    "axes.labelcolor":   C_SEC,
    "xtick.color":       C_MUTED,
    "ytick.color":       C_MUTED,
    "xtick.labelsize":   9,
    "ytick.labelsize":   9,
    "legend.frameon":    False,
    "legend.fontsize":   9,
})

# Figures 2 and 3 in the paper are embedded at only 3.0 in wide (matching the IEEE column
# width). A figure's *effective* printed font size is native_pt * (embed_width /
# native_fig_width_in), so the fix is a narrower native canvas with much larger native
# fonts -- not just bumping font.size against the old 16 in / 13 in wide canvases, which
# printed at roughly 2-4 pt. These constants target >=7 pt effective at 3.0 in.
PRINT_EMBED_W = 3.0
PRINT_FONT = {
    "panel_title": 14, "axis_label": 12.5,
    "tick": 12, "legend": 12, "cell_val": 11,
}

# ── Helpers ───────────────────────────────────────────────────────────────────
# Design note (2026-09-02 redesign): the previous version labeled every single bar/cell
# with its exact value. At the 3.0in embed width this reliably collided -- adjacent bars'
# value text overlapped ("0.87" and "0.82" merging into "0.870.82"), because there simply
# isn't enough horizontal room for a text label per bar at a legible font size. Per
# standard practice (label the *few* points that need it, let the axis carry the rest),
# per-bar/per-cell value labels are dropped entirely for the bar charts; the y-axis now
# carries multiple ticks so the reader can read values off the gridlines instead. The
# heatmap keeps its cell labels (color alone is too imprecise on a 5x4 grid) but drops the
# now-redundant suptitle and shortens tick labels to stop them colliding.
def sm(d, key, default=0):
    return d[0].get(key, default)

def grouped_bar(ax, vals_v1, vals_v2, ylabel, title, ylim, yticks):
    x = np.arange(len(STRATEGIES))
    w = 0.34
    ax.bar(x - w/2, vals_v1, w, color=C_V1, label="V1 Standard", linewidth=0, zorder=3)
    ax.bar(x + w/2, vals_v2, w, color=C_V2, label="V2 Stress",   linewidth=0, zorder=3)
    ax.set_title(title, fontsize=PRINT_FONT["panel_title"], fontweight="bold", color=C_TEXT, pad=8)
    ax.set_ylabel(ylabel, fontsize=PRINT_FONT["axis_label"])
    ax.set_xticks(x)
    ax.set_xticklabels(STRATEGY_LABELS_COMPACT, fontsize=PRINT_FONT["tick"] - 1, rotation=32, ha="right")
    ax.set_yticks(yticks)
    ax.tick_params(axis="y", labelsize=PRINT_FONT["tick"])
    ax.set_ylim(ylim)
    ax.xaxis.grid(False)
    ax.spines["left"].set_color(C_AXIS)
    ax.spines["bottom"].set_color(C_AXIS)


def draw_heatmap(ax, matrix, title, cmap="Blues"):
    data = np.array(matrix, dtype=float)
    im = ax.imshow(data, aspect="auto", cmap=cmap, vmin=0, vmax=100,
                   interpolation="nearest")
    # Thin white separators between cells so adjacent percentage labels never visually merge
    for k in range(len(TIERS) + 1):
        ax.axvline(k - 0.5, color=BG, linewidth=2, zorder=2)
    for k in range(len(STRATEGIES) + 1):
        ax.axhline(k - 0.5, color=BG, linewidth=2, zorder=2)
    for i in range(len(STRATEGIES)):
        for j in range(len(TIERS)):
            val = data[i, j]
            if not math.isnan(val):
                ink = "white" if val > 60 else C_TEXT
                ax.text(j, i, f"{val:.0f}", ha="center", va="center",
                        fontsize=PRINT_FONT["cell_val"], color=ink, fontweight="bold", zorder=4)
    ax.set_xticks(range(len(TIERS)))
    ax.set_xticklabels(["Easy", "Med.", "Hard", "V.Hard"], fontsize=PRINT_FONT["tick"] - 1)
    ax.set_yticks(range(len(STRATEGIES)))
    ax.set_yticklabels(
        ["None", "Lawnmower", "Threat-Inf.", "Target-Inf.", "Hybrid"],
        fontsize=PRINT_FONT["tick"] - 1,
    )
    ax.set_title(title, fontsize=PRINT_FONT["panel_title"], fontweight="bold", color=C_TEXT, pad=8)
    for spine in ax.spines.values():
        spine.set_color(C_AXIS)
    return im

# ── Extract values ────────────────────────────────────────────────────────────
success_v1 = [sm(v1[s], "success_rate") for s in STRATEGIES]
success_v2 = [sm(v2[s], "success_rate") for s in STRATEGIES]
mpi_v1     = [sm(v1[s], "avg_mpi")      for s in STRATEGIES]
mpi_v2     = [sm(v2[s], "avg_mpi")      for s in STRATEGIES]
esc_v1     = [sm(v1[s], "avg_escapes")  for s in STRATEGIES]
esc_v2     = [sm(v2[s], "avg_escapes")  for s in STRATEGIES]

def tier_matrix(data_dict):
    matrix = []
    for s in STRATEGIES:
        row = []
        for tier in TIERS:
            b = data_dict[s][0].get("by_preset", {}).get(tier, {})
            total = b.get("total", 0)
            succ  = b.get("success", 0)
            row.append(round(100 * succ / total, 1) if total else float("nan"))
        matrix.append(row)
    return matrix

heat_v1 = tier_matrix(v1)
heat_v2 = tier_matrix(v2)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 1: Grouped bar charts (success rate, MPI, escape events)
# ═══════════════════════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(1, 3, figsize=(7.4, 2.55), facecolor=BG)
fig1.subplots_adjust(wspace=0.65, left=0.09, right=0.99, top=0.78, bottom=0.36)

esc_max = max(max(esc_v1), max(esc_v2))
esc_top = math.ceil(esc_max * 1.3)

grouped_bar(axes[0], success_v1, success_v2, "Success (%)", "Success Rate",
            ylim=(0, 115), yticks=[0, 25, 50, 75, 100])

grouped_bar(axes[1], mpi_v1, mpi_v2, "MPI", "MPI Score",
            ylim=(0, 1.12), yticks=[0, 0.25, 0.5, 0.75, 1.0])

grouped_bar(axes[2], esc_v1, esc_v2, "Escapes", "Escape Events",
            ylim=(0, esc_top), yticks=list(range(0, esc_top + 1, max(1, esc_top // 5))))

# No suptitle -- the Word caption below the figure already states what it is; removing it
# (and the per-bar value labels above) recovers both height and horizontal breathing room.
handles, labels = axes[0].get_legend_handles_labels()
fig1.legend(handles, labels, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0),
            fontsize=PRINT_FONT["legend"], frameon=False)

out1 = os.path.join(SCRIPT_DIR, "comparison_bars.png")
fig1.savefig(out1, dpi=300, facecolor=BG)  # fixed canvas (no bbox_inches="tight")
print(f"\nSaved: {out1}")
plt.close(fig1)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 2: Difficulty heatmaps (V1 left, V2 right)
# ═══════════════════════════════════════════════════════════════════════════════
fig2, (ax_h1, ax_h2) = plt.subplots(1, 2, figsize=(7.6, 3.05), facecolor=BG)
fig2.subplots_adjust(wspace=0.75, left=0.2, right=0.85, top=0.86, bottom=0.16)

im1 = draw_heatmap(ax_h1, heat_v1, "V1 Standard")
im2 = draw_heatmap(ax_h2, heat_v2, "V2 Stress")

# Shared colorbar on the right
cax = fig2.add_axes([0.89, 0.16, 0.025, 0.7])
cb  = fig2.colorbar(im2, cax=cax)
cb.set_label("Success %", fontsize=PRINT_FONT["axis_label"], color=C_SEC)
cb.ax.tick_params(labelsize=PRINT_FONT["tick"] - 2, colors=C_MUTED)

# No suptitle -- the Word caption below the figure already names it.

out2 = os.path.join(SCRIPT_DIR, "comparison_heatmap.png")
fig2.savefig(out2, dpi=300, facecolor=BG)  # fixed canvas (no bbox_inches="tight")
print(f"Saved: {out2}")
plt.close(fig2)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 3 & 4: Per-version overview (all metrics for V1, then V2)
# Each figure: 3 bars (success, MPI, escapes) with strategies on x-axis
# ═══════════════════════════════════════════════════════════════════════════════
def single_version_bars(data_dict, version_label, color, out_name):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), facecolor=BG)
    fig.subplots_adjust(wspace=0.38, left=0.06, right=0.97, top=0.88, bottom=0.15)

    succ = [data_dict[s][0].get("success_rate", 0) for s in STRATEGIES]
    mpi  = [data_dict[s][0].get("avg_mpi", 0)      for s in STRATEGIES]
    esc  = [data_dict[s][0].get("avg_escapes", 0)  for s in STRATEGIES]
    x    = np.arange(len(STRATEGIES))
    w    = 0.55

    for ax, vals, ylabel, title, ylim, fmt in [
        (axes[0], succ, "Success Rate (%)",    "Mission Success Rate",        (0, 115),  ".1f"),
        (axes[1], mpi,  "MPI Score [0 - 1]",   "Mission Performance Index",   (0, 1.12), ".3f"),
        (axes[2], esc,  "Avg. Escape Events",   "Average Enemy Escape Events", (0, max(esc)*1.4 if max(esc) > 0 else 1), ".2f"),
    ]:
        bars = ax.bar(x, vals, w, color=color, linewidth=0, zorder=3)
        pad  = (ylim[1] - ylim[0]) * 0.025
        for b in bars:
            h = b.get_height()
            ax.text(b.get_x() + b.get_width()/2, h + pad, f"{h:{fmt}}",
                    ha="center", va="bottom", fontsize=8.5, color=C_TEXT)
        ax.set_title(title, fontsize=11, fontweight="bold", color=C_TEXT, pad=8)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels(STRATEGY_LABELS, fontsize=8.5)
        ax.set_ylim(ylim)
        ax.xaxis.grid(False)
        ax.spines["left"].set_color(C_AXIS)
        ax.spines["bottom"].set_color(C_AXIS)

    fig.suptitle(f"UAV Strategy Results - {version_label}",
                 fontsize=13, fontweight="bold", color=C_TEXT, y=0.97)
    path = os.path.join(SCRIPT_DIR, out_name)
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"Saved: {path}")
    plt.close(fig)

single_version_bars(v1, "V1 Standard Profile", C_V1, "v1_overview.png")
single_version_bars(v2, "V2 Stress Profile",   C_V2, "v2_overview.png")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURES 5-9: Per-strategy deep-dive (one figure per strategy, V1 vs V2)
# Layout: 2 rows x 2 cols
#   top-left:  success/partial/failed stacked bar (V1 vs V2)
#   top-right: MPI bar (V1 vs V2)
#   bottom-left: escape events bar (V1 vs V2)
#   bottom-right: by-difficulty line chart (V1 solid, V2 dashed)
# ═══════════════════════════════════════════════════════════════════════════════
TIER_X = np.arange(len(TIERS))
C_SUCCESS = "#008300"   # green  slot 2
C_PARTIAL = "#eda100"   # yellow slot 4
C_FAILED  = "#e34948"   # red    slot 8

for strat, label in zip(STRATEGIES, STRATEGY_LABELS):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), facecolor=BG)
    fig.subplots_adjust(hspace=0.45, wspace=0.38,
                        left=0.09, right=0.97, top=0.88, bottom=0.1)

    sm1, res1 = v1[strat]
    sm2, res2 = v2[strat]

    # -- Top-left: stacked outcome bars --
    ax = axes[0, 0]
    categories = ["V1 Standard", "V2 Stress"]
    succ_vals = [sm1.get("success_rate", 0), sm2.get("success_rate", 0)]
    part_vals = [sm1.get("partial_rate", 0), sm2.get("partial_rate", 0)]
    fail_vals = [sm1.get("failed_rate",  0), sm2.get("failed_rate",  0)]
    xi = np.arange(2)
    w  = 0.5
    b1 = ax.bar(xi, succ_vals, w, label="Success", color=C_SUCCESS, linewidth=0, zorder=3)
    b2 = ax.bar(xi, part_vals, w, bottom=succ_vals, label="Partial", color=C_PARTIAL, linewidth=0, zorder=3)
    b3 = ax.bar(xi, fail_vals, w,
                bottom=[s+p for s,p in zip(succ_vals, part_vals)],
                label="Failed", color=C_FAILED, linewidth=0, zorder=3)
    for i, (s, p, f) in enumerate(zip(succ_vals, part_vals, fail_vals)):
        if s > 4: ax.text(i, s/2,       f"{s:.0f}%", ha="center", va="center", fontsize=9, color="white", fontweight="bold")
        if p > 4: ax.text(i, s + p/2,   f"{p:.0f}%", ha="center", va="center", fontsize=9, color="white", fontweight="bold")
        if f > 4: ax.text(i, s + p + f/2, f"{f:.0f}%", ha="center", va="center", fontsize=9, color="white", fontweight="bold")
    ax.set_xticks(xi); ax.set_xticklabels(categories, fontsize=9)
    ax.set_ylabel("Mission Outcome (%)", fontsize=9)
    ax.set_ylim(0, 115); ax.xaxis.grid(False)
    ax.set_title("Outcome Breakdown", fontsize=10, fontweight="bold", color=C_TEXT, pad=6)
    ax.legend(fontsize=8, loc="upper right")
    ax.spines["left"].set_color(C_AXIS); ax.spines["bottom"].set_color(C_AXIS)

    # -- Top-right: MPI bar --
    ax = axes[0, 1]
    mpi_vals = [sm1.get("avg_mpi", 0), sm2.get("avg_mpi", 0)]
    bars = ax.bar(xi, mpi_vals, w, color=[C_V1, C_V2], linewidth=0, zorder=3)
    for b in bars:
        h = b.get_height()
        ax.text(b.get_x()+b.get_width()/2, h+0.015, f"{h:.3f}",
                ha="center", va="bottom", fontsize=9, color=C_TEXT)
    ax.set_xticks(xi); ax.set_xticklabels(categories, fontsize=9)
    ax.set_ylabel("MPI Score [0 - 1]", fontsize=9)
    ax.set_ylim(0, 1.12); ax.xaxis.grid(False)
    ax.set_title("Mission Performance Index", fontsize=10, fontweight="bold", color=C_TEXT, pad=6)
    ax.spines["left"].set_color(C_AXIS); ax.spines["bottom"].set_color(C_AXIS)

    # -- Bottom-left: escape events bar --
    ax = axes[1, 0]
    esc_vals = [sm1.get("avg_escapes", 0), sm2.get("avg_escapes", 0)]
    bars = ax.bar(xi, esc_vals, w, color=[C_V1, C_V2], linewidth=0, zorder=3)
    for b in bars:
        h = b.get_height()
        ax.text(b.get_x()+b.get_width()/2, h+0.05, f"{h:.2f}",
                ha="center", va="bottom", fontsize=9, color=C_TEXT)
    ax.set_xticks(xi); ax.set_xticklabels(categories, fontsize=9)
    ax.set_ylabel("Avg. Escape Events", fontsize=9)
    top = max(esc_vals)*1.4 if max(esc_vals) > 0 else 1
    ax.set_ylim(0, top); ax.xaxis.grid(False)
    ax.set_title("Enemy Escape Events", fontsize=10, fontweight="bold", color=C_TEXT, pad=6)
    ax.spines["left"].set_color(C_AXIS); ax.spines["bottom"].set_color(C_AXIS)

    # -- Bottom-right: success by difficulty tier --
    ax = axes[1, 1]
    def tier_success(sm):
        bp = sm.get("by_preset", {})
        vals = []
        for t in TIERS:
            b = bp.get(t, {})
            tot = b.get("total", 0)
            vals.append(round(100*b.get("success",0)/tot, 1) if tot else 0)
        return vals

    tv1 = tier_success(sm1)
    tv2 = tier_success(sm2)
    ax.plot(TIER_X, tv1, "o-", color=C_V1, linewidth=2, markersize=6,
            label="V1 Standard", zorder=3)
    ax.plot(TIER_X, tv2, "s--", color=C_V2, linewidth=2, markersize=6,
            label="V2 Stress", zorder=3)
    for xi_, v in zip(TIER_X, tv1):
        ax.text(xi_, v+2.5, f"{v:.0f}%", ha="center", fontsize=8, color=C_V1)
    for xi_, v in zip(TIER_X, tv2):
        ax.text(xi_, v-5, f"{v:.0f}%", ha="center", fontsize=8, color=C_V2)
    ax.set_xticks(TIER_X); ax.set_xticklabels(TIER_LABELS, fontsize=9)
    ax.set_ylabel("Success Rate (%)", fontsize=9)
    ax.set_ylim(0, 115); ax.set_title("Success by Difficulty Tier", fontsize=10, fontweight="bold", color=C_TEXT, pad=6)
    ax.legend(fontsize=8)
    ax.spines["left"].set_color(C_AXIS); ax.spines["bottom"].set_color(C_AXIS)

    FULL_NAMES = {
        "none": "None (Baseline)", "lawnmower": "Lawnmower",
        "threat_informed": "Threat-Informed", "target_informed": "Target-Informed",
        "hybrid": "Hybrid",
    }
    fig.suptitle(f"Strategy: {FULL_NAMES[strat]}", fontsize=13, fontweight="bold", color=C_TEXT, y=0.97)
    path = os.path.join(SCRIPT_DIR, f"strategy_{strat}.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"Saved: {path}")
    plt.close(fig)

print("\nDone.")

