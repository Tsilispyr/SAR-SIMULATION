"""
SAR UAV Strategy Comparison -- Paired Headless Probe

Imports the backend module directly (no Docker, no HTTP) and runs one mission per
UAV strategy from an IDENTICAL spawn and RNG stream for each seed, using the same
tick loop as simulation_loop(). This is the paired ("common random numbers")
design the batch harness lacks, at a small scale, plus instrumentation of what
the UAV actually contributes:

  - graph-direction side effects of ox.project_graph (directed reachability,
    nodes with no successors, terrain-penalty lookups that miss);
  - edges long enough to escape the midpoint-only proximity / UAV queries;
  - per run: outcome, escape decisions, MANEUVER attempts rejected for lack of a
    directed path, UAV distance flown, intel store size/age, and how many
    enemies the UAV ever detected.

The OSMnx cache and any mission stats are written to a temp directory, never to
Backend/data. Requires the backend's Python dependencies:
  pip install -r Backend/requirements.txt

Usage:
  python headless_probe.py                      # seeds 11 23, R = 1000 m, V2-medium counts
  python headless_probe.py --seeds 1 2 3 --radius 800
"""
import argparse
import contextlib
import io
import math
import os
import random
import sys
import tempfile
import time

import networkx as nx
import numpy as np
import osmnx as ox

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import Backend.src.main as M  # noqa: E402  (namespace-package import keeps the relative imports working)

WORK_DIR = os.path.join(tempfile.gettempdir(), "sar_headless_probe")
ox.settings.cache_folder = os.path.join(WORK_DIR, "osmnx_cache")
M.STATS_DIR = os.path.join(WORK_DIR, "stats")
M.SCENARIOS_DIR = os.path.join(WORK_DIR, "scenarios")

BASE_LAT, BASE_LON = 37.9838, 23.7275            # same centre as the batch harness
STRATEGIES = ["none", "lawnmower", "threat_informed", "target_informed", "hybrid"]
SIM_SPEED = 4.0                                   # same multiplier as the batch harness
DT = M.TICK_RATE * SIM_SPEED
MAX_TICKS = int(2880 / DT)                        # 48-minute simulated budget


def quiet():
    return contextlib.redirect_stdout(io.StringIO())


def graph_report(G):
    U = G.to_undirected()
    rng = random.Random(7)
    nodes = list(G.nodes())
    pairs = [(rng.choice(nodes), rng.choice(nodes)) for _ in range(400)]
    print(f"[direction] G_proj directed={G.is_directed()}; random node pairs reachable: "
          f"directed {sum(nx.has_path(G, a, b) for a, b in pairs)}/400, "
          f"undirected {sum(nx.has_path(U, a, b) for a, b in pairs)}/400")
    print(f"[direction] nodes with zero successors: {sum(G.out_degree(n) == 0 for n in nodes)}/{len(nodes)}")
    seps = np.array([math.hypot(G.nodes[u]['x'] - G.nodes[v]['x'], G.nodes[u]['y'] - G.nodes[v]['y'])
                     for u, v in U.edges()])
    print(f"[midpoint] edges with endpoint separation > 160 m (enemy on an endpoint can be missed "
          f"at a 50 m detection radius): {(seps > 160).sum()}/{len(seps)}; "
          f"> 120 m (fresh UAV sighting, r_min = 60 m): {(seps > 120).sum()}")
    return U


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, nargs="+", default=[11, 23])
    parser.add_argument("--radius", type=int, default=1000)
    parser.add_argument("--n-patrol", type=int, default=5)
    parser.add_argument("--n-aggr", type=int, default=3)
    parser.add_argument("--n-targets", type=int, default=6)
    args = parser.parse_args()

    speeds = dict(patrol_speed=2.5, aggr_speed=4.5, agent_speed=6.0, detection_radius=50.0)
    t0 = time.time()
    with quiet():
        M.fetch_vector_map(lat=BASE_LAT, lon=BASE_LON, radius=args.radius, n_agents=1, mode="random",
                           n_patrol=args.n_patrol, n_aggr=args.n_aggr, n_targets=args.n_targets,
                           uav_strategy="none", **speeds)
    G = M.current_map_data["G_proj"]
    cx, cy = M.current_map_data["center_x"], M.current_map_data["center_y"]
    print(f"map loaded in {time.time() - t0:.0f}s: nodes={G.number_of_nodes()} edges={G.number_of_edges()} "
          f"buildings={len(M.current_map_data.get('buildings_proj', []))}")
    U = graph_report(G)

    brave = {"calls": 0, "rejected": 0, "no_directed_path": 0}
    original_brave = M.SARGameState.find_brave_path

    def brave_probe(self, Gp, ag, target, cx_, cy_):
        brave["calls"] += 1
        path = original_brave(self, Gp, ag, target, cx_, cy_)
        if not path:
            brave["rejected"] += 1
            if not nx.has_path(G, ag.robot_node, target["node"]):
                brave["no_directed_path"] += 1
        return path

    M.SARGameState.find_brave_path = brave_probe
    gl = M.game_logic

    for seed in args.seeds:
        print(f"\n=== seed {seed}: identical spawn and RNG stream for every strategy ===")
        for strategy in STRATEGIES:
            for k in brave:
                brave[k] = 0
            random.seed(seed)
            np.random.seed(seed)
            with quiet():
                gl.spawn_objects(G, cx, cy, args.n_patrol, args.n_aggr, args.n_targets,
                                 speeds["patrol_speed"], speeds["aggr_speed"], 1, speeds["agent_speed"],
                                 speeds["detection_radius"], None, None, None,
                                 uav_strategy=strategy, mission_radius=args.radius)
            random.seed(seed + 1000)
            gl.sim_running = True
            hops = terrain_missed = hover = 0
            uav_m = 0.0
            prev_uav = None
            sizes, ages, seen = [], [], set()
            while gl.current_tick < MAX_TICKS and not gl.game_over:
                with quiet():
                    for ag in gl.agents:            # same body as simulation_loop()
                        if not ag.active:
                            continue
                        before = ag.robot_node
                        gl.advance_agent(ag, DT)
                        ag.robot_node = ox.distance.nearest_nodes(G, ag.robot_x + cx, ag.robot_y + cy)
                        if ag.robot_node != before:
                            hops += 1
                            if G.get_edge_data(before, ag.robot_node) is None and U.has_edge(before, ag.robot_node):
                                terrain_missed += 1
                    gl.update(G, DT, cx, cy)
                uav = gl.uav_agent
                if uav is not None:
                    if prev_uav is not None:
                        step = math.hypot(uav.x - prev_uav[0], uav.y - prev_uav[1])
                        uav_m += step
                        hover += step < 0.5
                    prev_uav = (uav.x, uav.y)
                    sizes.append(len(gl.uav_intelligence))
                    if gl.uav_intelligence:
                        ages.append(np.mean([gl.current_tick - v[2] for v in gl.uav_intelligence.values()]))
                    seen |= set(gl.uav_intelligence)
            ag = gl.agents[0]
            row = (f"  {strategy:16s} {gl.status_message:32s} t={gl.mission_elapsed:6.0f}s "
                   f"targets {gl.targets_total - len(gl.targets)}/{gl.targets_total} escapes={ag.escape_events:2d} "
                   f"maneuver rejected {brave['rejected']}/{brave['calls']} "
                   f"(no directed path: {brave['no_directed_path']}) "
                   f"terrain missed {terrain_missed}/{hops}")
            if uav is not None:
                row += (f" | UAV {uav_m / 1000:.1f} km, hover {100 * hover / max(1, len(sizes)):.1f}%, "
                        f"intel mean {np.mean(sizes):.2f} (age {np.mean(ages) if ages else 0:.0f} ticks), "
                        f"detected {len(seen)}/{len(gl.enemies)} enemies")
            print(row, flush=True)


if __name__ == "__main__":
    main()
