"""
SAR UAV-UGV Two-Tier Architecture Diagram (paper Figure 1).

Rebuilt from scratch (no earlier generator script existed in this repo) to fix a real layout
bug: the previous hand-made version routed the UAV Agent -> Intelligence Store "detections"
arrow as a diagonal curve that swept directly across the "detections" text label and crossed
the Intelligence Store -> Temporal Decay Model arrow. This version routes both cross-tier
connectors as orthogonal (elbow) paths confined entirely to the neutral gap strip between tier
bands, so they never cross any in-tier arrow or label.

Usage:
  python plot_architecture.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.path import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---- palette (matches the original diagram's tier colors) ----
BG = "#fcfcfb"
UAV_BAND, UAV_BOX = "#dbe9f7", "#8fb8dd"
INT_BAND, INT_BOX = "#fdf3d9", "#f0c26a"
ENEMY_BOX = "#fbe2cf"
ENEMY_EDGE = "#d98b52"
GND_BAND, GND_BOX = "#e0f2e6", "#8fcaa4"
UGV_EDGE = "#2f7a4f"
EDGE_GRAY = "#4a4a4a"
TEXT_DARK = "#1a1a1a"
LABEL_GRAY = "#5a5a5a"

fig, ax = plt.subplots(figsize=(7.4, 3.34), facecolor=BG)
ax.set_xlim(0, 20)
ax.set_ylim(0, 9.1)
ax.axis("off")
fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)


def band(y0, y1, color, label):
    ax.add_patch(FancyBboxPatch((0.15, y0), 19.7, y1 - y0, boxstyle="round,pad=0,rounding_size=0.12",
                                 linewidth=1.1, edgecolor="#9a9a9a", facecolor=color, zorder=1))
    ax.text(0.45, y1 - 0.06, label, fontsize=10.5, fontweight="bold", color=LABEL_GRAY,
            va="top", ha="left", zorder=3)


def box(cx, cy, w, h, title, subtitle, face, edge, title_size=12.5, sub_size=10):
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                 boxstyle="round,pad=0,rounding_size=0.08",
                                 linewidth=1.6, edgecolor=edge, facecolor=face, zorder=4))
    ax.text(cx, cy + h * 0.22, title, fontsize=title_size, fontweight="bold", ha="center",
            va="center", color=TEXT_DARK, zorder=5)
    if subtitle:
        ax.text(cx, cy - h * 0.26, subtitle, fontsize=sub_size, style="italic", ha="center",
                va="center", color="#333333", zorder=5)


def math_box(cx, cy, w, h, title, formula, face, edge, formula_size=12):
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                 boxstyle="round,pad=0,rounding_size=0.08",
                                 linewidth=1.6, edgecolor=edge, facecolor=face, zorder=4))
    ax.text(cx, cy + h * 0.26, title, fontsize=12.5, fontweight="bold", ha="center",
            va="center", color=TEXT_DARK, zorder=5)
    ax.text(cx, cy - h * 0.24, formula, fontsize=formula_size, ha="center", va="center",
            color=TEXT_DARK, zorder=5)


def straight_arrow(p0, p1, color=EDGE_GRAY, lw=1.8, mutation=14):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=mutation,
                                  linewidth=lw, color=color, zorder=3, shrinkA=0, shrinkB=0))


def elbow_arrow(pts, color, lw=2.0, mutation=13):
    path = Path(pts, [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1))
    ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=mutation,
                                  linewidth=lw, color=color, zorder=3,
                                  shrinkA=0, shrinkB=0, joinstyle="round"))


def edge_label(x, y, text):
    ax.text(x, y, text, fontsize=9.5, ha="center", va="center", color=LABEL_GRAY,
            zorder=6, bbox=dict(boxstyle="round,pad=0.2", facecolor=BG, edgecolor="none"))


# ================= tier bands (each 2.7 tall, 0.4 gap between) =================
# Ground:        y in [0.0, 2.7]
# gap            y in [2.7, 3.1]
# Intelligence:  y in [3.1, 5.8]
# gap            y in [5.8, 6.2]
# UAV:           y in [6.2, 8.9]
band(6.2, 8.9, UAV_BAND, "UAV TIER")
band(3.1, 5.8, INT_BAND, "INTELLIGENCE PIPELINE")
band(0.0, 2.7, GND_BAND, "GROUND TIER")

# ================= UAV tier boxes (label reserves top 0.5 of the band) =================
box(3.5, 7.15, 5.0, 1.5, "Coverage Strategy", "(5 variants)", UAV_BOX, "#3a3a3a")
box(15.5, 7.15, 5.0, 1.5, "UAV Agent", "(aerial, 180 m radius)", UAV_BOX, "#3a3a3a")
straight_arrow((6.0, 7.15), (13.0, 7.15))

# ================= Intelligence tier boxes =================
box(5.5, 4.6, 7.0, 1.0, "Intelligence Store", "(positions + timestamps)", INT_BOX, "#3a3a3a")
box(5.5, 3.45, 6.4, 0.85, "Enemy Agents", "(patrol + aggr.)", ENEMY_BOX, ENEMY_EDGE,
    title_size=11.5, sub_size=9.5)
math_box(15.5, 4.6, 7.0, 1.0, "Temporal Decay Model",
         r"$W(d,\Delta t) = W_{base}\cdot e^{-\Delta t/\tau}$", INT_BOX, "#3a3a3a",
         formula_size=11.5)

straight_arrow((9.0, 4.6), (12.0, 4.6))
straight_arrow((5.5, 3.875), (5.5, 4.1), color=ENEMY_EDGE, lw=1.8, mutation=9)

# ================= Ground tier boxes =================
box(3.5, 1.35, 6.0, 1.5, "OSM Road Graph", "(Athens, 1500 m radius)", GND_BOX, "#3a3a3a")
math_box(11.25, 1.35, 6.5, 1.5, "Penalized A* Planner",
         r"$C(u,v) = L(u,v) + \Sigma\,W(d,\Delta t)$", GND_BOX, "#3a3a3a", formula_size=11)
box(17.75, 1.35, 3.2, 1.5, "UGV", "", GND_BOX, UGV_EDGE, title_size=12.5)

straight_arrow((6.5, 1.35), (8.0, 1.35))
straight_arrow((14.5, 1.35), (16.15, 1.35))

# ================= Cross-tier orthogonal connectors (confined to the gap strips) =================
GAP1_Y = 6.0   # between UAV tier (bottom 6.2) and Intelligence tier (top 5.8)
GAP2_Y = 2.9   # between Intelligence tier (bottom 3.1) and Ground tier (top 2.7)

# UAV Agent -> Intelligence Store ("detections"): drop, go left in the gap, drop into the box top
elbow_arrow([(15.5, 6.4), (15.5, GAP1_Y), (5.5, GAP1_Y), (5.5, 5.1)],
            "#2f6f9e", lw=2.0, mutation=13)
edge_label(10.5, GAP1_Y + 0.24, "detections")

# Temporal Decay Model -> Penalized A* Planner ("penalized costs")
elbow_arrow([(15.5, 4.1), (15.5, GAP2_Y), (11.25, GAP2_Y), (11.25, 2.125)],
            "#2f7a4f", lw=2.0, mutation=13)
edge_label(13.6, GAP2_Y + 0.24, "penalized costs")

out_path = os.path.join(SCRIPT_DIR, "system_architecture.png")
fig.savefig(out_path, dpi=300, facecolor=BG)
print("Saved:", out_path)
