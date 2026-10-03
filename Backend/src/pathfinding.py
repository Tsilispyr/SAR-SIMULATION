import math
import numpy as np
from shapely.geometry import Point

def compute_dead_ends(G):
    """Return set of node ids with degree <= 1 (dead-end cul-de-sacs)."""
    return {n for n in G.nodes() if G.degree(n) <= 1}

def densify_path(path_coords, step_meters=2.0):
    if len(path_coords) < 2:
        return path_coords
    new_path = []
    for i in range(len(path_coords) - 1):
        p1, p2 = path_coords[i], path_coords[i + 1]
        new_path.append(p1)
        dist = ((p2["x"] - p1["x"])**2 + (p2["y"] - p1["y"])**2)**0.5
        if dist > step_meters:
            num_segments = int(dist / step_meters)
            for j in range(1, num_segments + 1):
                t = j / (num_segments + 1)
                new_path.append({
                    "x": round(p1["x"] + (p2["x"] - p1["x"]) * t, 2),
                    "y": round(p1["y"] + (p2["y"] - p1["y"]) * t, 2)
                })
    new_path.append(path_coords[-1])
    return new_path

def build_penalized_graph(G, enemies, geofence=None, cx=0.0, cy=0.0,
                          uav_threats=None, current_tick=0, tau=30.0):
    """Return a copy of G with tiered enemy risk penalties and geofence penalties applied.
    Uses cKDTree spatial indexing to avoid brute-force O(E x K) distance checks.

    uav_threats: dict {enemy_id: (ex_centered, ey_centered, tick_seen)} from UAV detections.
    Applies a temporal-decay penalty for UAV-spotted threats:
        W(d, t) = W_base * exp(-t / tau) * max(0, 1 - d / r_influence)
    where t = ticks since detection, tau = decay constant (ticks), r_influence grows with t
    as the enemy may have drifted from its last known position.
    """
    from scipy.spatial import cKDTree

    U = G.to_undirected() if G.is_directed() else G.copy()

    edge_list = list(U.edges(keys=True, data=True))
    if not edge_list:
        return U

    edge_mids     = np.empty((len(edge_list), 2), dtype=np.float64)
    edge_u_coords = np.empty((len(edge_list), 2), dtype=np.float64)
    edge_v_coords = np.empty((len(edge_list), 2), dtype=np.float64)
    for idx, (u, v, k, data) in enumerate(edge_list):
        ux, uy = U.nodes[u]['x'], U.nodes[u]['y']
        vx, vy = U.nodes[v]['x'], U.nodes[v]['y']
        edge_mids[idx]     = ((ux + vx) / 2.0, (uy + vy) / 2.0)
        edge_u_coords[idx] = (ux, uy)
        edge_v_coords[idx] = (vx, vy)

    mid_tree = cKDTree(edge_mids) if (enemies or uav_threats) and len(edge_list) > 0 else None

    # Geofence: massive penalty outside polygon
    if geofence is not None:
        for idx, (u, v, k, data) in enumerate(edge_list):
            mid_x, mid_y = edge_mids[idx]
            if not geofence.contains(Point(mid_x, mid_y)):
                data['length'] = data.get('length', 1.0) + 6000.0

    # Direct-sighting enemy proximity penalties (tiered rings)
    if enemies and mid_tree is not None:
        for enemy in enemies:
            ex, ey     = enemy.x + cx, enemy.y + cy
            radius     = enemy.detection_radius
            ext_radius = radius + 30.0
            aggr_mult  = 1.5 if enemy.type == "aggressive" else 1.0

            nearby_indices = mid_tree.query_ball_point([ex, ey], ext_radius)
            for idx in nearby_indices:
                u, v, k, data = edge_list[idx]
                ux, uy = edge_u_coords[idx]
                vx, vy = edge_v_coords[idx]
                mid_x, mid_y = edge_mids[idx]
                closest_dist = min(
                    math.hypot(mid_x - ex, mid_y - ey),
                    math.hypot(ux    - ex, uy    - ey),
                    math.hypot(vx    - ex, vy    - ey),
                )
                if   closest_dist == 0:          penalty = 8000.0
                elif closest_dist <= 10.0:       penalty = 3000.0
                elif closest_dist <= 30.0:       penalty = 1500.0
                elif closest_dist <= radius:     penalty = 800.0
                elif closest_dist <= ext_radius: penalty = 300.0
                else:                            penalty = 0.0
                if penalty > 0:
                    data['length'] = data.get('length', 1.0) + penalty * aggr_mult

    # UAV temporal-decay threat penalties
    # Peak penalty W_BASE decays as exp(-t/tau). Influence radius grows as enemy may drift
    # at ~3 m/tick from its last known position (conservative upper bound on enemy speed).
    # This models uncertainty: a fresh sighting yields high, sharp penalty;
    # a stale sighting yields lower, spatially spread-out penalty.
    W_BASE         = 1200.0   # peak penalty for a just-detected UAV threat
    DRIFT_RATE     = 3.0      # m/tick - max expected enemy movement
    MIN_DECAY      = 0.05     # skip entries whose strength fell below 5%

    if uav_threats and mid_tree is not None:
        for enemy_id, (uex, uey, tick_seen) in uav_threats.items():
            t = max(0, current_tick - tick_seen)
            decay = math.exp(-t / max(tau, 1.0))
            if decay < MIN_DECAY:
                continue
            r_influence = max(60.0, DRIFT_RATE * t)
            w_peak      = W_BASE * decay
            ex_abs      = uex + cx
            ey_abs      = uey + cy

            nearby = mid_tree.query_ball_point([ex_abs, ey_abs], r_influence)
            for idx in nearby:
                _, _, _, data = edge_list[idx]
                d   = math.hypot(edge_mids[idx][0] - ex_abs, edge_mids[idx][1] - ey_abs)
                pen = w_peak * max(0.0, 1.0 - d / r_influence)
                if pen > 1.0:
                    data['length'] = data.get('length', 1.0) + pen

    return U
