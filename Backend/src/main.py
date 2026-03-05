# V12.0 — Backend self-drive loop, browser mission view, enemy pathfinding fix
from __future__ import annotations
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import osmnx as ox
import networkx as nx
from pyproj import Transformer
import asyncio
import threading
import traceback
import logging
import random
import math
import json
import os
import time

# --- ANSI DEBUG COLORS ---
class C:
    OK    = '\033[92m'  # Green
    WARN  = '\033[93m'  # Yellow
    ERR   = '\033[91m'  # Red
    OSM   = '\033[96m'  # Cyan
    AGENT = '\033[94m'  # Blue
    ENEMY = '\033[95m'  # Magenta
    END   = '\033[0m'   # Reset

# --- 1. LOGGING SETUP ---
class EndpointFilter(logging.Filter):
    MUTED = ("/api/robot/position", "/api/robot/telemetry", "/api/stats")
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return not any(ep in msg for ep in self.MUTED)

logging.getLogger("uvicorn.access").addFilter(EndpointFilter())

# --- 2. GLOBAL SETTINGS ---
ox.settings.use_cache = True
ox.settings.cache_folder = '/app/data/osmnx_cache'

STATS_DIR   = '/app/data/stats'
ROBOT_SPEED = 6.0   # m/s autonomous movement speed
TICK_RATE   = 0.2   # seconds per simulation tick (5 Hz)
sim_speed_multiplier: float = 1.0  # adjustable via /api/sim/speed

_sim_lock = threading.Lock()  # protects game_logic from concurrent writes

# app is created without lifespan here; lifespan is wired in after its definition below.
app = FastAPI(title="SAR UGV AI Backend", version="12.0 (Self-Drive + Browser View)")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True,
                   allow_methods=["*"], allow_headers=["*"])

# --- 3. DATA MODELS ---
class RobotPosition(BaseModel):
    x: float
    y: float
    delta: float = 0.5

class VectorPathRequest(BaseModel):
    start_x: float; start_y: float; end_x: float; end_y: float

# --- 4. GAME LOGIC CLASSES ---

class Enemy:
    def __init__(self, _id, enemy_type, start_node, G, cx, cy, patrol_speed=2.5, aggr_speed=4.5):
        self.id = _id
        self.type = "patrol"         # current state
        self.base_type = enemy_type  # 'patrol' or 'aggressive'
        self.G = G.to_undirected() if G.is_directed() else G
        self.cx = cx
        self.cy = cy
        self.detection_radius = 50.0
        self.patrol_speed = patrol_speed
        self.aggr_speed   = aggr_speed
        self.speed        = patrol_speed
        self.current_path  = []
        self.current_node  = start_node
        self.previous_node = None
        n = G.nodes[start_node]
        self.x = round(n['x'] - cx, 2)
        self.y = round(n['y'] - cy, 2)

    def calculate_path_coords(self, path_nodes):
        coords = []
        for i in range(len(path_nodes) - 1):
            u, v = path_nodes[i], path_nodes[i + 1]
            edge_data_dict = self.G.get_edge_data(u, v) or self.G.get_edge_data(v, u)
            data = min(edge_data_dict.values(), key=lambda x: x.get('length', 999999)) if edge_data_dict else {}
            if 'geometry' in data:
                geom_coords = list(data['geometry'].coords)
                u_x, u_y = self.G.nodes[u]['x'], self.G.nodes[u]['y']
                if ((geom_coords[0][0] - u_x)**2 + (geom_coords[0][1] - u_y)**2 >
                        (geom_coords[-1][0] - u_x)**2 + (geom_coords[-1][1] - u_y)**2):
                    geom_coords.reverse()
                for c in geom_coords:
                    pt = {"x": round(c[0] - self.cx, 2), "y": round(c[1] - self.cy, 2)}
                    if not coords or coords[-1] != pt:
                        coords.append(pt)
            else:
                pt_u = {"x": round(self.G.nodes[u]['x'] - self.cx, 2), "y": round(self.G.nodes[u]['y'] - self.cy, 2)}
                pt_v = {"x": round(self.G.nodes[v]['x'] - self.cx, 2), "y": round(self.G.nodes[v]['y'] - self.cy, 2)}
                if not coords or coords[-1] != pt_u:
                    coords.append(pt_u)
                coords.append(pt_v)
        return coords

    def get_new_destination(self, robot_node):
        if self.type == "patrol":
            neighbors = list(self.G.neighbors(self.current_node))
            if self.previous_node in neighbors and len(neighbors) > 1:
                neighbors.remove(self.previous_node)
            if neighbors:
                next_node = random.choice(neighbors)
                return [self.current_node, next_node]
        elif self.type == "aggressive":
            try:
                # Guard against disconnected nodes to prevent spammy failures
                if not nx.has_path(self.G, self.current_node, robot_node):
                    return []
                path = nx.shortest_path(self.G, self.current_node, robot_node, weight='length')
                if len(path) > 1:
                    return [path[0], path[1]]
                elif len(path) == 1:
                    return path
            except:
                pass
        return []

    def move(self, robot_node, robot_x, robot_y, delta_t):
        dist_to_robot = math.sqrt((self.x - robot_x)**2 + (self.y - robot_y)**2)
        if self.base_type == "aggressive":
            if dist_to_robot < self.detection_radius:
                if self.type != "aggressive":
                    self.type = "aggressive"
                    self.speed = self.aggr_speed
                    print(f"{C.ENEMY}[ENEMY-{self.id}]  PATROL → CHASE! Dist={dist_to_robot:.1f}m{C.END}")
            else:
                if self.type != "patrol":
                    self.type = "patrol"
                    self.speed = self.patrol_speed
                    self.current_path.clear()
                    print(f"{C.ENEMY}[ENEMY-{self.id}]  CHASE → PATROL  Dist={dist_to_robot:.1f}m{C.END}")

        if len(self.current_path) == 0:
            node_path = self.get_new_destination(robot_node)
            if len(node_path) > 1:
                self.previous_node = self.current_node
                self.current_path  = self.calculate_path_coords(node_path)
                self.current_node  = node_path[-1]
                print(f"{C.ENEMY}[ENEMY-{self.id}] {self.type.upper()} Mode: New Path | Node:{self.current_node} | Pts:{len(self.current_path)}{C.END}")
            else:
                print(f"{C.ERR}[ENEMY-{self.id}] FAILED to find path!{C.END}")

        move_dist_remaining = self.speed * delta_t
        while len(self.current_path) > 0 and move_dist_remaining > 0:
            target = self.current_path[0]
            dist_to_target = math.sqrt((target['x'] - self.x)**2 + (target['y'] - self.y)**2)
            if dist_to_target < 0.01:
                self.x, self.y = target['x'], target['y']
                self.current_path.pop(0)
                continue
            if move_dist_remaining >= dist_to_target:
                self.x, self.y = target['x'], target['y']
                self.current_path.pop(0)
                move_dist_remaining -= dist_to_target
            else:
                dir_x = (target['x'] - self.x) / dist_to_target
                dir_y = (target['y'] - self.y) / dist_to_target
                self.x += dir_x * move_dist_remaining
                self.y += dir_y * move_dist_remaining
                move_dist_remaining = 0


# ---------------------------------------------------------------------------
#  TERRAIN COST CONFIG (mirrors SARGV-FIN-CC.py TILE_TYPE_DIFFICULT/HAZARD)
# ---------------------------------------------------------------------------
HIGHWAY_SCORE_PENALTY = {
    # Per-step score penalty for traversing this road type
    "motorway":      0,
    "motorway_link": 0,
    "trunk":         0,
    "trunk_link":    0,
    "primary":       0,
    "primary_link":  0,
    "secondary":     0,
    "secondary_link":0,
    "tertiary":      1,   # DIFFICULT
    "tertiary_link": 1,
    "unclassified":  1,
    "residential":   1,
    "living_street": 1,
    "service":       1,
    "track":         3,   # HAZARD
    "path":          3,
    "footway":       3,
    "cycleway":      3,
    "steps":         3,
}

HIGHWAY_MULTIPLIER = {
    "motorway":      0.8,
    "motorway_link": 0.8,
    "trunk":         0.85,
    "trunk_link":    0.85,
    "primary":       1.0,
    "primary_link":  1.0,
    "secondary":     1.0,
    "secondary_link":1.0,
    "tertiary":      1.3,
    "tertiary_link": 1.3,
    "unclassified":  1.4,
    "residential":   1.5,
    "living_street": 2.0,
    "service":       2.0,
    "track":         2.5,
    "path":          3.5,
    "footway":       3.5,
    "cycleway":      3.0,
    "steps":         4.0,
}


def build_penalized_graph(G, enemies, cx=0.0, cy=0.0):
    """Return a copy of G with tiered enemy risk penalties applied to edge lengths.
    cx, cy: map centre in projected coords. Enemy positions are centred-relative,
    so we add cx/cy to convert them to absolute projected coords for distance checks.
    """
    U = G.to_undirected() if G.is_directed() else G.copy()
    for enemy in enemies:
        # Enemy stores centred coords; add cx/cy for projected comparison with node coords
        ex, ey     = enemy.x + cx, enemy.y + cy
        radius     = enemy.detection_radius
        ext_radius = radius + 30.0
        aggr_mult  = 1.5 if enemy.type == "aggressive" else 1.0
        for u, v, k, data in U.edges(keys=True, data=True):
            ux, uy = U.nodes[u]['x'], U.nodes[u]['y']
            vx, vy = U.nodes[v]['x'], U.nodes[v]['y']
            mid_x, mid_y = (ux + vx) / 2.0, (uy + vy) / 2.0
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
    return U


def compute_dead_ends(G):
    """Return set of node ids with degree ≤ 1 (dead-end cul-de-sacs)."""
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


def save_stats_to_disk(stats_history, score, status):
    """Persist mission stats to /app/data/stats/<timestamp>.json"""
    try:
        os.makedirs(STATS_DIR, exist_ok=True)
        filename = os.path.join(STATS_DIR, f"mission_{int(time.time())}.json")
        with open(filename, 'w') as f:
            json.dump({
                "timestamp": time.time(),
                "final_score": score,
                "final_status": status,
                "history": stats_history
            }, f)
        print(f"{C.OK}[STATS] Saved to {filename}{C.END}")
    except Exception as e:
        print(f"{C.ERR}[STATS] Failed to save: {e}{C.END}")


# ---------------------------------------------------------------------------
#  GAME STATE
# ---------------------------------------------------------------------------

class SARGameState:
    def __init__(self):
        self.enemies            = []
        self.targets            = []
        self.targets_total      = 0
        self.score              = 0
        self.game_over          = False
        self.status_message     = "Active"
        self._proximity_cooldowns = {}
        self.stats_history      = []
        self.mission_elapsed    = 0.0
        self.stats_saved        = False

        # Autonomous robot state
        self.robot_x            = 0.0
        self.robot_y            = 0.0
        self.robot_node         = None
        self.robot_auto_path    = []
        self.current_target_id  = None
        self.escape_mode        = False
        self.replan_timer       = 0.0
        self.last_decision      = "Waiting for mission start..."
        self.dead_ends          = set()

        # Terrain penalty tracking
        self._last_robot_node   = None

        # Simulation control
        self.last_telemetry_time = 0.0  # set by Godot telemetry; 0 → self-driven
        self.sim_running        = False  # mission doesn't start until /api/sim/start
        self.sim_paused         = False
        self.robot_speed_ms     = 6.0   # m/s; configurable via /api/map/vector agent_speed
        self._spawn_snapshot    = None  # set by spawn_objects for restart

    # ------------------------------------------------------------------
    def spawn_objects(self, G, cx, cy, n_patrol=3, n_aggressive=2, n_targets=5,
                      patrol_speed=2.5, aggr_speed=4.5):
        self.enemies.clear()
        self.targets.clear()
        self.score              = 0
        self.game_over          = False
        self.status_message     = "Mission Start"
        self._proximity_cooldowns = {}
        self.stats_history      = []
        self.mission_elapsed    = 0.0
        self.stats_saved        = False
        self.robot_auto_path    = []
        self.current_target_id  = None
        self.escape_mode        = False
        self.replan_timer       = 0.0
        self.last_decision      = "Selecting first target..."
        self._last_robot_node   = None
        self.dead_ends          = compute_dead_ends(G)
        self.last_telemetry_time = 0.0  # reset so self-drive loop takes over

        all_nodes = list(G.nodes())
        if len(all_nodes) > n_targets:
            target_nodes = random.sample(all_nodes, n_targets)
            for i, node in enumerate(target_nodes):
                n = G.nodes[node]
                self.targets.append({
                    "id": i,
                    "x": round(n['x'] - cx, 2),
                    "y": round(n['y'] - cy, 2),
                    "node": node
                })
        else:
            target_nodes = []

        self.targets_total = len(self.targets)

        available_nodes = [n for n in all_nodes if n not in target_nodes]
        total_enemies   = n_patrol + n_aggressive
        if len(available_nodes) > total_enemies:
            enemy_nodes = random.sample(available_nodes, total_enemies)
            for i in range(n_patrol):
                self.enemies.append(Enemy(i, "patrol", enemy_nodes[i], G, cx, cy, patrol_speed, aggr_speed))
                print(f"{C.ENEMY}[SPAWN] Enemy-{i} PATROL | speed={patrol_speed}{C.END}")
            for i in range(n_aggressive):
                idx = n_patrol + i
                self.enemies.append(Enemy(idx, "aggressive", enemy_nodes[idx], G, cx, cy, patrol_speed, aggr_speed))
                print(f"{C.ENEMY}[SPAWN] Enemy-{idx} AGGRESSIVE | patrol={patrol_speed}/aggr={aggr_speed}{C.END}")

        # Save spawn snapshot for restart
        self._spawn_snapshot = {
            "enemies":  [{"id": e.id, "base_type": e.base_type,
                          "node": e.current_node,
                          "patrol_speed": patrol_speed, "aggr_speed": aggr_speed}
                         for e in self.enemies],
            "targets":  [{"id": t["id"], "x": t["x"], "y": t["y"],
                          "node": t["node"]} for t in self.targets],
            "robot":    {"x": 0.0, "y": 0.0},
        }
        self.sim_running = False   # must call /api/sim/start to begin
        self.sim_paused  = False

    # ------------------------------------------------------------------
    # ADVANCE ROBOT (called by background simulation loop)
    # ------------------------------------------------------------------
    def advance_robot(self, delta_t: float):
        """Move robot_x/robot_y along robot_auto_path at self.robot_speed_ms m/s."""
        if not self.robot_auto_path:
            return
        dist_to_move = self.robot_speed_ms * delta_t
        while self.robot_auto_path and dist_to_move > 0:
            wp = self.robot_auto_path[0]
            dx = wp['x'] - self.robot_x
            dy = wp['y'] - self.robot_y
            d  = math.hypot(dx, dy)
            if d < 0.3:                          # snap & pop
                self.robot_x, self.robot_y = wp['x'], wp['y']
                self.robot_auto_path.pop(0)
            elif dist_to_move >= d:              # consume waypoint
                self.robot_x, self.robot_y = wp['x'], wp['y']
                self.robot_auto_path.pop(0)
                dist_to_move -= d
            else:                                # partial step
                self.robot_x += (dx / d) * dist_to_move
                self.robot_y += (dy / d) * dist_to_move
                dist_to_move = 0

    # ------------------------------------------------------------------
    # ESCAPE PATH — flee to spot maximising dist(enemy→spot) − path_len(robot→spot)
    # ------------------------------------------------------------------
    def find_escape_path(self, G, robot_node, penalized_G, cx, cy):
        """Return centred path coords leading away from threats, or [] if no threat."""
        threatening = [e for e in self.enemies
                       if e.base_type == "aggressive" and e.type == "aggressive"]
        if not threatening:
            return []

        # Candidate escape nodes: BFS ring 2–8 hops from robot
        candidates = set()
        frontier = {robot_node}
        visited  = {robot_node}
        for hop in range(8):
            next_frontier = set()
            for n in frontier:
                for nb in G.neighbors(n):
                    if nb not in visited:
                        next_frontier.add(nb)
                        visited.add(nb)
            if hop >= 2:
                candidates |= next_frontier
            frontier = next_frontier
            if not frontier:
                break

        if not candidates:
            return []

        best_score = -999999.0
        best_node  = None
        for cand in candidates:
            # Node coords are projected (absolute); enemy coords are centred-relative
            cx_n = G.nodes[cand]['x']
            cy_n = G.nodes[cand]['y']
            enemy_turns = min(
                (math.hypot((cx_n - cx) - e.x, (cy_n - cy) - e.y) / max(e.speed, 0.1))
                for e in threatening
            )
            try:
                path_len = nx.shortest_path_length(penalized_G, robot_node, cand, weight='length')
            except:
                continue
            score = enemy_turns - (path_len / 4.0)
            if score > best_score:
                best_score = score
                best_node  = cand

        if best_node is None:
            return []
        try:
            node_path = nx.shortest_path(penalized_G, robot_node, best_node, weight='length')
            coords = _extract_path_coords_centred(G, node_path, cx, cy)
            return densify_path(coords, 2.0)
        except:
            return []

    # ------------------------------------------------------------------
    # BRAVE PATH — proceed to nearest target if we'll arrive before the enemy
    # ------------------------------------------------------------------
    def find_brave_path(self, G, robot_node, target, cx, cy):
        """Return centred path to target only if robot arrives SAFELY before aggressive enemies.

        'Safely' means robot_steps + safety_margin < enemy_steps (margin = 1 step).
        Routes on the penalized G (passed as G) so the path naturally avoids enemies.
        Only aggressive enemies in active-chase mode are considered a disqualifying threat.
        Patrol enemies affect path cost via the penalized graph but don't veto the route.
        """
        threatening = [e for e in self.enemies
                       if e.base_type == "aggressive" and e.type == "aggressive"]
        target_node = target['node']

        # Dead-end guard — skip targets that are dead-ends
        try:
            if G.degree(target_node) <= 1:
                return []  # dead-end target — too risky
        except:
            pass

        try:
            # Route via penalized graph (avoids enemy zones)
            node_path   = nx.shortest_path(G, robot_node, target_node, weight='length')
            robot_steps = nx.shortest_path_length(G, robot_node, target_node, weight='length')
        except:
            return []

        SAFETY_MARGIN = 1  # robot must arrive at least 1 step ahead
        for e in threatening:
            try:
                e_node = ox.distance.nearest_nodes(
                    current_map_data["G_proj"], e.x + cx, e.y + cy)
                enemy_steps = nx.shortest_path_length(
                    current_map_data["G_proj"], e_node, target_node, weight='length')
            except:
                enemy_steps = 999999

            if robot_steps + SAFETY_MARGIN >= enemy_steps:
                return []  # enemy arrives too close behind us — abort

        coords = _extract_path_coords_centred(current_map_data["G_proj"], node_path, cx, cy)
        return densify_path(coords, 2.0)

    # ------------------------------------------------------------------
    # AUTONOMOUS RE-ROUTING — called every tick; replans when needed
    # ------------------------------------------------------------------
    REPLAN_INTERVAL = 5  # ticks between full replans

    def autonomous_replan(self, G, robot_node, cx, cy):
        """Choose best target, compute escape/brave/normal path, update robot_auto_path."""
        self.replan_timer += 1
        if self.replan_timer < self.REPLAN_INTERVAL and self.robot_auto_path:
            return  # still following existing path, not time to replan yet

        self.replan_timer = 0

        if not self.targets:
            return

        penalized_G = build_penalized_graph(G, self.enemies, cx, cy)
        # Store centre on penalized graph (may still be needed for debug reads)
        penalized_G.graph['center_x_raw'] = cx
        penalized_G.graph['center_y_raw'] = cy

        # --- Check escape condition ---
        threatening = [e for e in self.enemies
                       if e.base_type == "aggressive" and e.type == "aggressive"]
        if threatening:
            nearest_threat_dist = min(
                math.hypot(e.x - self.robot_x, e.y - self.robot_y)
                for e in threatening
            )
            if nearest_threat_dist < 1.2 * self.enemies[0].detection_radius:
                # Try brave first
                best_t = self._pick_best_target(penalized_G, robot_node)
                if best_t:
                    brave = self.find_brave_path(penalized_G, robot_node, best_t, cx, cy)
                    if brave:
                        self.robot_auto_path = brave
                        self.escape_mode     = False
                        self.current_target_id = best_t['id']
                        self.last_decision = (
                            f"BRAVE: heading to T{best_t['id']} despite threat "
                            f"({nearest_threat_dist:.0f}m)"
                        )
                        print(f"{C.AGENT}[AUTO] {self.last_decision}{C.END}")
                        return

                # Escape
                escape = self.find_escape_path(G, robot_node, penalized_G, cx, cy)
                if escape:
                    self.robot_auto_path   = escape
                    self.escape_mode       = True
                    self.current_target_id = None
                    self.last_decision     = (
                        f"ESCAPE: fleeing threat "
                        f"({nearest_threat_dist:.0f}m to nearest aggressive)"
                    )
                    print(f"{C.AGENT}[AUTO] {self.last_decision}{C.END}")
                    return

        # --- Normal: pick lowest cost target ---
        best_t = self._pick_best_target(penalized_G, robot_node)
        if not best_t:
            return

        if best_t['id'] == self.current_target_id and self.robot_auto_path:
            return  # already on the right path

        try:
            node_path = nx.shortest_path(penalized_G, robot_node, best_t['node'], weight='length')
            coords    = _extract_path_coords_centred(G, node_path, cx, cy)
            self.robot_auto_path = densify_path(coords, 2.0)
            self.escape_mode     = False
            self.current_target_id = best_t['id']
            self.last_decision   = (
                f"Targeting T{best_t['id']} at ({best_t['x']:.0f},{best_t['y']:.0f}) "
                f"| path pts: {len(self.robot_auto_path)} | escape_mode: OFF"
            )
            print(f"{C.AGENT}[AUTO] {self.last_decision}{C.END}")
        except Exception as e:
            print(f"{C.ERR}[AUTO] pathfinding failed: {e}{C.END}")

    def _pick_best_target(self, penalized_G, robot_node):
        """Return target dict with lowest A* cost from robot_node."""
        best_t    = None
        best_cost = float('inf')
        # Apply dead-end penalty to penalized_G temporarily
        for t in self.targets:
            try:
                cost = nx.shortest_path_length(penalized_G, robot_node, t['node'], weight='length')
                # Dead-end penalty: +200 if target node is a dead-end
                if t['node'] in self.dead_ends:
                    cost += 200
                if cost < best_cost:
                    best_cost = cost
                    best_t    = t
            except:
                pass
        return best_t

    # ------------------------------------------------------------------
    # MAIN UPDATE LOOP
    # ------------------------------------------------------------------
    def update(self, robot_x, robot_y, robot_node, G, delta_t):
        if self.game_over:
            return

        self.robot_x    = robot_x
        self.robot_y    = robot_y
        self.robot_node = robot_node
        self.mission_elapsed += delta_t

        # Record time-series snapshot
        self.stats_history.append({
            "t":       round(self.mission_elapsed, 1),
            "score":   self.score,
            "targets": len(self.targets)
        })

        # --- Terrain penalty for current edge ---
        if self._last_robot_node is not None and robot_node != self._last_robot_node:
            edge_data = G.get_edge_data(self._last_robot_node, robot_node)
            if edge_data:
                data = min(edge_data.values(), key=lambda x: x.get('length', 999999))
                hw   = data.get('highway', '')
                if isinstance(hw, list):
                    hw = hw[0]
                penalty = HIGHWAY_SCORE_PENALTY.get(hw, 0)
                if penalty:
                    self.score -= penalty
                    print(f"{C.WARN}[TERRAIN] Node transition on '{hw}' road → score −{penalty} | Score: {self.score}{C.END}")
        self._last_robot_node = robot_node

        # --- Target collection ---
        for t in self.targets[:]:
            dist = math.sqrt((t['x'] - robot_x)**2 + (t['y'] - robot_y)**2)
            if dist < 8.0:
                self.targets.remove(t)
                self.score += 50
                if self.current_target_id == t['id']:
                    self.current_target_id = None
                    self.robot_auto_path   = []  # trigger replan
                print(f"{C.OK}[SCORE] Target #{t['id']} Collected! Score:{self.score} Remaining:{len(self.targets)}{C.END}")

        # --- Mission success ---
        if not self.targets:
            self.status_message = "MISSION SUCCESS!"
            self.game_over      = True
            print(f"{C.OK}[GAME] MISSION SUCCESS! Final Score: {self.score}{C.END}")
            if not self.stats_saved:
                save_stats_to_disk(self.stats_history, self.score, self.status_message)
                self.stats_saved = True
            return

        # --- Enemy movement & contact ---
        for enemy in self.enemies:
            enemy.move(robot_node, robot_x, robot_y, delta_t)

            dist = math.sqrt((enemy.x - robot_x)**2 + (enemy.y - robot_y)**2)

            # Proximity warning
            warn_key = f"prox_{enemy.id}"
            if dist < enemy.detection_radius * 2.0:
                if self._proximity_cooldowns.get(warn_key, 0) <= 0:
                    print(f"{C.WARN}[THREAT] ⚠ ENEMY-{enemy.id} ({enemy.type}) {dist:.0f}m away{C.END}")
                    self._proximity_cooldowns[warn_key] = 60
                else:
                    self._proximity_cooldowns[warn_key] -= 1

            # Contact check
            if dist < 5.0:
                # Per SARGV-FIN-CC: agent is incapacitated (not instant full failure for multi-agent)
                targets_collected = self.targets_total - len(self.targets)
                frac = targets_collected / max(self.targets_total, 1)
                if frac > 0.70:
                    self.status_message = "MISSION PARTIAL SUCCESS (>70% targets)"
                else:
                    self.status_message = "MISSION FAILED: ENEMY CONTACT"
                self.score    -= 100
                self.game_over = True
                print(f"{C.ERR}[GAME] ENEMY CONTACT → {self.status_message} | Score:{self.score}{C.END}")
                if not self.stats_saved:
                    save_stats_to_disk(self.stats_history, self.score, self.status_message)
                    self.stats_saved = True
                return

        # --- Advance auto_path (pop consumed waypoints) ---
        while self.robot_auto_path:
            wp = self.robot_auto_path[0]
            d  = math.hypot(wp['x'] - robot_x, wp['y'] - robot_y)
            if d < 5.0:
                self.robot_auto_path.pop(0)
            else:
                break

        # --- Autonomous re-routing ---
        cx = G.graph.get('center_x_raw', 0)
        cy = G.graph.get('center_y_raw', 0)
        self.autonomous_replan(G, robot_node, cx, cy)


# ---------------------------------------------------------------------------
# HELPER: extract path coords from node list
# ---------------------------------------------------------------------------
def _extract_path_coords(G, node_path, _cx, _cy):
    """Convert a NetworkX node list to [{x, y}] dicts (proj coords, not centred)."""
    path_coords = []
    if len(node_path) > 1:
        for i in range(len(node_path) - 1):
            u, v = node_path[i], node_path[i + 1]
            edge_data_dict = G.get_edge_data(u, v) or G.get_edge_data(v, u)
            data = min(edge_data_dict.values(), key=lambda x: x.get('length', 999999)) if edge_data_dict else {}
            if 'geometry' in data:
                coords = list(data['geometry'].coords)
                u_x, u_y = G.nodes[u]['x'], G.nodes[u]['y']
                if ((coords[0][0] - u_x)**2 + (coords[0][1] - u_y)**2 >
                        (coords[-1][0] - u_x)**2 + (coords[-1][1] - u_y)**2):
                    coords.reverse()
                for c in coords:
                    pt = {"x": round(c[0], 2), "y": round(c[1], 2)}
                    if not path_coords or path_coords[-1] != pt:
                        path_coords.append(pt)
            else:
                pt_u = {"x": round(G.nodes[u]['x'], 2), "y": round(G.nodes[u]['y'], 2)}
                pt_v = {"x": round(G.nodes[v]['x'], 2), "y": round(G.nodes[v]['y'], 2)}
                if not path_coords or path_coords[-1] != pt_u:
                    path_coords.append(pt_u)
                path_coords.append(pt_v)
    else:
        n = node_path[0]
        path_coords.append({"x": round(G.nodes[n]['x'], 2), "y": round(G.nodes[n]['y'], 2)})
    return path_coords


def _extract_path_coords_centred(G, node_path, cx, cy):
    """Like _extract_path_coords but subtracts (cx, cy) for centred coords."""
    raw = _extract_path_coords(G, node_path, 0, 0)
    return [{"x": round(p['x'] - cx, 2), "y": round(p['y'] - cy, 2)} for p in raw]


# ---------------------------------------------------------------------------
# BACKGROUND SIMULATION LOOP (self-drives robot when Godot is not connected)
# ---------------------------------------------------------------------------
async def simulation_loop():
    """5 Hz tick: advances robot, runs game update, updates lat/lon."""
    while True:
        await asyncio.sleep(TICK_RATE)
        with _sim_lock:
            G = current_map_data.get("G_proj")
            if G is None or game_logic.game_over:
                continue
            # Wait for Start button signal
            if not game_logic.sim_running or game_logic.sim_paused:
                continue
            # Yield to Godot telemetry if it posted recently (within 2s)
            if time.time() - game_logic.last_telemetry_time < 2.0:
                continue
            try:
                cx, cy = current_map_data["center_x"], current_map_data["center_y"]
                G.graph['center_x_raw'] = cx
                G.graph['center_y_raw'] = cy

                effective_dt = TICK_RATE * sim_speed_multiplier
                game_logic.advance_robot(effective_dt)
                rx, ry = game_logic.robot_x, game_logic.robot_y
                robot_node = ox.distance.nearest_nodes(G, rx + cx, ry + cy)
                game_logic.update(rx, ry, robot_node, G, effective_dt)

                transformer = current_map_data.get("transformer")
                if transformer and 'crs' in G.graph:
                    lon, lat = transformer.transform(rx + cx, ry + cy)
                    robot_state["lat"] = lat
                    robot_state["lon"] = lon
            except Exception as exc:
                print(f"{C.ERR}[SIM LOOP] {exc}{C.END}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(simulation_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

# Wire lifespan into the app now that it's defined
app.router.lifespan_context = lifespan


# ---------------------------------------------------------------------------
# GLOBAL STATE
# ---------------------------------------------------------------------------
current_map_data = {
    "G_proj":        None,
    "center_x":      0.0,
    "center_y":      0.0,
    "last_json_response": None,
    "transformer":   None,
}
game_logic  = SARGameState()
robot_state = {"lat": None, "lon": None}


# ---------------------------------------------------------------------------
# API ENDPOINTS
# ---------------------------------------------------------------------------

@app.post("/api/robot/telemetry")
def update_robot_position(pos: RobotPosition):
    """Accept telemetry from Godot. If active, Godot overrides the self-drive loop."""
    G = current_map_data["G_proj"]
    if G is None:
        return {"status": "failed", "message": "No map loaded"}

    cx, cy = current_map_data["center_x"], current_map_data["center_y"]
    G.graph['center_x_raw'] = cx
    G.graph['center_y_raw'] = cy

    robot_node = ox.distance.nearest_nodes(G, pos.x + cx, pos.y + cy)

    with _sim_lock:
        game_logic.last_telemetry_time = time.time()  # marks Godot as active
        game_logic.robot_x = pos.x
        game_logic.robot_y = pos.y
        game_logic.update(pos.x, pos.y, robot_node, G, pos.delta)

    transformer = current_map_data.get("transformer")
    if transformer and 'crs' in G.graph:
        lon, lat = transformer.transform(pos.x + cx, pos.y + cy)
        robot_state["lat"], robot_state["lon"] = lat, lon

    return {
        "status": "success",
        "game_state": {
            "score":     game_logic.score,
            "message":   game_logic.status_message,
            "game_over": game_logic.game_over,
            "targets":   game_logic.targets,
            "enemies":   [{"id": e.id, "type": e.type, "x": e.x, "y": e.y}
                          for e in game_logic.enemies],
        },
        "auto_path": game_logic.robot_auto_path[:60],
    }


@app.get("/api/robot/position")
def get_robot_position():
    G = current_map_data["G_proj"]
    transformer = current_map_data.get("transformer")
    if G is None or transformer is None or 'crs' not in G.graph:
        return {**robot_state, "enemies": [], "targets": [], "auto_path": [],
                "score": 0, "status": "No map", "escape_mode": False, "game_over": False}

    cx, cy = current_map_data["center_x"], current_map_data["center_y"]

    enemies_latlon = []
    for e in game_logic.enemies:
        lon, lat = transformer.transform(e.x + cx, e.y + cy)
        enemies_latlon.append({
            "id": e.id, "type": e.type, "lat": lat, "lon": lon,
            "detection_radius_m": e.detection_radius
        })

    targets_latlon = []
    for t in game_logic.targets:
        lon, lat = transformer.transform(t['x'] + cx, t['y'] + cy)
        targets_latlon.append({"id": t['id'], "lat": lat, "lon": lon})

    # Convert auto_path waypoints to lat/lon for browser visualization
    auto_path_latlon = []
    for p in game_logic.robot_auto_path[:50]:
        try:
            p_lon, p_lat = transformer.transform(p['x'] + cx, p['y'] + cy)
            auto_path_latlon.append([p_lat, p_lon])
        except:
            pass

    return {
        "lat":      robot_state["lat"],
        "lon":      robot_state["lon"],
        "enemies":  enemies_latlon,
        "targets":  targets_latlon,
        "auto_path": auto_path_latlon,
        "score":    game_logic.score,
        "status":   game_logic.status_message,
        "escape_mode": game_logic.escape_mode,
        "game_over":   game_logic.game_over,
    }


@app.get("/api/map/vector")
def fetch_vector_map(lat: float = 38.900, lon: float = 22.433, radius: int = 1500,
                     n_patrol: int = 3, n_aggr: int = 2, n_targets: int = 5,
                     patrol_speed: float = 2.5, aggr_speed: float = 4.5,
                     agent_speed: float = 6.0):
    try:
        print(f"\n{C.OSM}[OSM] Map Request: lat={lat}, lon={lon}, radius={radius}{C.END}")
        print(f"{C.OSM}[OSM] Spawn: Patrol={n_patrol}({patrol_speed}m/s) Aggr={n_aggr}({aggr_speed}m/s) Targets={n_targets}{C.END}")

        G_directed = ox.graph_from_point((lat, lon), dist=radius, network_type='all')
        G          = G_directed.to_undirected()
        largest_cc = max(nx.connected_components(G), key=len)
        G          = G.subgraph(largest_cc).copy()
        G_proj     = ox.project_graph(G)

        # Apply terrain cost to edge lengths
        edges_modified = 0
        for u, v, k, data in G_proj.edges(keys=True, data=True):
            hw   = data.get('highway', '')
            if isinstance(hw, list): hw = hw[0]
            mult = HIGHWAY_MULTIPLIER.get(hw, 1.2)
            if mult != 1.0:
                data['length'] = data.get('length', 1.0) * mult
                edges_modified += 1
        print(f"{C.OSM}[TERRAIN] {edges_modified}/{G_proj.number_of_edges()} edges weighted.{C.END}")

        nodes_gdf = ox.graph_to_gdfs(G_proj, edges=False)
        min_x, min_y, max_x, max_y = nodes_gdf.total_bounds
        bbox_cx = (min_x + max_x) / 2.0
        bbox_cy = (min_y + max_y) / 2.0
        center_node = ox.distance.nearest_nodes(G_proj, bbox_cx, bbox_cy)
        center_x    = G_proj.nodes[center_node]['x']
        center_y    = G_proj.nodes[center_node]['y']
        print(f"{C.WARN}[OSM] Center: ({center_x:.2f}, {center_y:.2f}){C.END}")

        # Cache transformer once
        transformer = None
        if 'crs' in G_proj.graph:
            transformer = Transformer.from_crs(G_proj.graph['crs'], "epsg:4326", always_xy=True)

        current_map_data.update({
            "G_proj":      G_proj,
            "center_x":    center_x,
            "center_y":    center_y,
            "transformer": transformer,
            "n_patrol":    n_patrol,
            "n_aggr":      n_aggr,
            "n_targets":   n_targets,
            "patrol_speed":patrol_speed,
            "aggr_speed":  aggr_speed,
            "agent_speed": agent_speed,
        })

        game_logic.robot_speed_ms = agent_speed
        game_logic.spawn_objects(G_proj, center_x, center_y, n_patrol, n_aggr, n_targets,
                                  patrol_speed, aggr_speed)

        roads_data = []
        for u, v, data in G_proj.edges(data=True):
            coords = (list(data['geometry'].coords) if 'geometry' in data
                      else [(G_proj.nodes[u]['x'], G_proj.nodes[u]['y']),
                            (G_proj.nodes[v]['x'], G_proj.nodes[v]['y'])])
            roads_data.append([{"x": round(c[0] - center_x, 2), "y": round(c[1] - center_y, 2)}
                                for c in coords])

        response = {
            "status":   "success",
            "metadata": {"lat": lat, "lon": lon, "radius": radius},
            "roads":    roads_data
        }
        current_map_data["last_json_response"] = response
        return response

    except Exception as e:
        print(traceback.format_exc())
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/map/current")
def get_current_map():
    return current_map_data["last_json_response"] or HTTPException(404, "No map")


@app.post("/api/path/vector")
def get_vector_path(req: VectorPathRequest):
    """Manual pathfinding endpoint (right-click override from Godot)."""
    G = current_map_data["G_proj"]
    if G is None:
        return {"status": "failed"}

    cx, cy = current_map_data["center_x"], current_map_data["center_y"]
    U      = G.to_undirected() if G.is_directed() else G.copy()

    # Line-of-sight shortcut
    try:
        start_edge = ox.distance.nearest_edges(U, req.start_x + cx, req.start_y + cy)
        end_edge   = ox.distance.nearest_edges(U, req.end_x   + cx, req.end_y   + cy)
        if isinstance(start_edge, tuple) and isinstance(end_edge, tuple):
            se = (start_edge[0], start_edge[1])
            ee = (end_edge[0],   end_edge[1])
            if se == ee or se == (ee[1], ee[0]):
                print(f"{C.AGENT}[AGENT] Line-of-Sight shortcut.{C.END}")
                return {"status": "success",
                        "path": densify_path([{"x": req.start_x, "y": req.start_y},
                                               {"x": req.end_x,   "y": req.end_y}], 2.0)}
    except Exception as e:
        print(f"{C.ERR}[ERROR] LoS check: {e}{C.END}")

    # Apply risk penalties
    dead_ends = game_logic.dead_ends
    for enemy in game_logic.enemies:
        ex, ey     = enemy.x + cx, enemy.y + cy
        radius     = enemy.detection_radius
        ext_radius = radius + 30.0
        aggr_mult  = 1.5 if enemy.type == "aggressive" else 1.0
        for u, v, k, data in U.edges(keys=True, data=True):
            ux, uy = U.nodes[u]['x'], U.nodes[u]['y']
            vx, vy = U.nodes[v]['x'], U.nodes[v]['y']
            mid_x, mid_y = (ux + vx) / 2.0, (uy + vy) / 2.0
            closest_dist = min(math.hypot(mid_x - ex, mid_y - ey),
                               math.hypot(ux    - ex, uy    - ey),
                               math.hypot(vx    - ex, vy    - ey))
            if   closest_dist == 0:          penalty = 8000.0
            elif closest_dist <= 10.0:       penalty = 3000.0
            elif closest_dist <= 30.0:       penalty = 1500.0
            elif closest_dist <= radius:     penalty = 800.0
            elif closest_dist <= ext_radius: penalty = 300.0
            else:                            penalty = 0.0
            if penalty > 0:
                data['length'] = data.get('length', 1.0) + penalty * aggr_mult

    # Dead-end penalty
    for u, v, k, data in U.edges(keys=True, data=True):
        if u in dead_ends or v in dead_ends:
            data['length'] = data.get('length', 1.0) + 200.0

    start_node = ox.distance.nearest_nodes(U, req.start_x + cx, req.start_y + cy)
    end_node   = ox.distance.nearest_nodes(U, req.end_x   + cx, req.end_y   + cy)

    try:
        node_path   = nx.shortest_path(U, start_node, end_node, weight='length')
        path_coords = _extract_path_coords_centred(G, node_path, cx, cy)

        # Anti-backtrack
        end_pt_x, end_pt_y = round(req.end_x, 2), round(req.end_y, 2)
        if len(path_coords) >= 2:
            second_to_last = path_coords[-2]
            last           = path_coords[-1]
            dist_to_click  = math.hypot(end_pt_x - second_to_last["x"], end_pt_y - second_to_last["y"])
            dist_to_last   = math.hypot(last["x"] - second_to_last["x"], last["y"] - second_to_last["y"])
            if dist_to_click <= dist_to_last:
                path_coords.pop()
                print(f"{C.WARN}[AGENT] Anti-Backtrack: popped overshoot.{C.END}")

        path_coords.append({"x": end_pt_x, "y": end_pt_y})
        dense_path = densify_path(path_coords, step_meters=2.0)
        print(f"{C.OK}[AGENT] Manual path: {len(dense_path)} pts{C.END}")
        return {"status": "success", "path": dense_path}

    except Exception as e:
        print(f"{C.ERR}[ERROR] Pathfinding: {e}{C.END}")
        return {"status": "success",
                "path": densify_path([{"x": req.start_x, "y": req.start_y},
                                       {"x": req.end_x,   "y": req.end_y}], 2.0)}


@app.get("/api/sim/reset")
def reset_game():
    G = current_map_data["G_proj"]
    if G is not None:
        cx, cy        = current_map_data["center_x"], current_map_data["center_y"]
        n_patrol      = current_map_data.get("n_patrol",      3)
        n_aggr        = current_map_data.get("n_aggr",        2)
        n_targets     = current_map_data.get("n_targets",     5)
        patrol_speed  = current_map_data.get("patrol_speed",  2.5)
        aggr_speed    = current_map_data.get("aggr_speed",    4.5)
        game_logic.spawn_objects(G, cx, cy, n_patrol, n_aggr, n_targets, patrol_speed, aggr_speed)
        print(f"{C.OK}[RESET] Simulation Reset{C.END}")
        return {"status": "success", "message": "Simulation Reset"}
    return {"status": "failed", "message": "No map loaded"}


@app.get("/api/sim/clear")
def clear_game_state():
    """Instantly wipes enemies and targets (pre-map-load flush)."""
    game_logic.enemies.clear()
    game_logic.targets.clear()
    game_logic._proximity_cooldowns.clear()
    game_logic.stats_history.clear()
    game_logic.mission_elapsed  = 0.0
    game_logic.score            = 0
    game_logic.game_over        = False
    game_logic.robot_auto_path  = []
    game_logic.status_message   = "Clearing..."
    game_logic.sim_running      = False
    game_logic.sim_paused       = False
    print(f"{C.WARN}[SIM] Game state cleared{C.END}")
    return {"status": "success"}


# ---------------------------------------------------------------------------
# MISSION CONTROL ENDPOINTS
# ---------------------------------------------------------------------------

@app.post("/api/sim/start")
def sim_start():
    """Start (or resume) the simulation. Robot begins moving."""
    with _sim_lock:
        if game_logic.game_over:
            return {"status": "game_over", "message": "Mission ended. Use /restart."}
        game_logic.sim_running = True
        game_logic.sim_paused  = False
    print(f"{C.OK}[SIM] Mission STARTED{C.END}")
    return {"status": "running"}


@app.post("/api/sim/pause")
def sim_pause():
    """Toggle pause. Returns new paused state."""
    with _sim_lock:
        if not game_logic.sim_running:
            return {"status": "not_started"}
        game_logic.sim_paused = not game_logic.sim_paused
        state = "paused" if game_logic.sim_paused else "running"
    print(f"{C.WARN}[SIM] Mission {state.upper()}{C.END}")
    return {"status": state}


@app.post("/api/sim/stop")
def sim_stop():
    """Completely stop the mission (triggers game_over)."""
    with _sim_lock:
        game_logic.sim_running    = False
        game_logic.sim_paused     = False
        game_logic.game_over      = True
        game_logic.status_message = "MISSION STOPPED"
    print(f"{C.WARN}[SIM] Mission STOPPED by user{C.END}")
    return {"status": "stopped"}


@app.post("/api/sim/restart")
def sim_restart():
    """Restore enemies and targets to original spawn positions and reset state."""
    G = current_map_data.get("G_proj")
    if G is None:
        return {"status": "failed", "message": "No map loaded"}
    snap = game_logic._spawn_snapshot
    if snap is None:
        # Fall back to full re-spawn
        cx = current_map_data["center_x"]
        cy = current_map_data["center_y"]
        n_patrol     = current_map_data.get("n_patrol",     3)
        n_aggr       = current_map_data.get("n_aggr",       2)
        n_targets    = current_map_data.get("n_targets",    5)
        patrol_speed = current_map_data.get("patrol_speed", 2.5)
        aggr_speed   = current_map_data.get("aggr_speed",   4.5)
        game_logic.spawn_objects(G, cx, cy, n_patrol, n_aggr, n_targets, patrol_speed, aggr_speed)
        return {"status": "reset", "note": "No snapshot; used random re-spawn"}

    with _sim_lock:
        cx, cy = current_map_data["center_x"], current_map_data["center_y"]

        # Restore enemies
        game_logic.enemies.clear()
        for i, es in enumerate(snap["enemies"]):
            e = Enemy(es["id"], es["base_type"], es["node"], G, cx, cy,
                      es["patrol_speed"], es["aggr_speed"])
            game_logic.enemies.append(e)

        # Restore targets
        game_logic.targets = [dict(t) for t in snap["targets"]]
        game_logic.targets_total = len(game_logic.targets)

        # Reset robot
        game_logic.robot_x          = snap["robot"]["x"]
        game_logic.robot_y          = snap["robot"]["y"]
        game_logic.robot_auto_path  = []
        game_logic.current_target_id = None
        game_logic.replan_timer     = 0.0
        game_logic.escape_mode      = False

        # Reset mission state
        game_logic.score              = 0
        game_logic.game_over          = False
        game_logic.status_message     = "Mission Restart"
        game_logic.mission_elapsed    = 0.0
        game_logic.stats_history.clear()
        game_logic.stats_saved        = False
        game_logic._proximity_cooldowns.clear()
        game_logic._last_robot_node   = None
        game_logic.last_decision      = "Mission restarted — press Start"
        game_logic.sim_running        = False
        game_logic.sim_paused         = False
        game_logic.last_telemetry_time = 0.0

    robot_state["lat"] = None
    robot_state["lon"] = None
    print(f"{C.OK}[SIM] Mission RESTARTED from snapshot{C.END}")
    return {"status": "restarted"}


@app.get("/api/sim/state")
def sim_state():
    """Return current mission control state for UI sync."""
    return {
        "running":  game_logic.sim_running,
        "paused":   game_logic.sim_paused,
        "game_over": game_logic.game_over,
        "elapsed":  round(game_logic.mission_elapsed, 1),
        "map_loaded": current_map_data.get("G_proj") is not None,
    }



class SpeedRequest(BaseModel):
    multiplier: float = 1.0


@app.post("/api/sim/speed")
def set_sim_speed(req: SpeedRequest):
    """Set simulation speed multiplier (0.25 – 50×). Affects robot speed, elapsed time, and enemy movement."""
    global sim_speed_multiplier
    sim_speed_multiplier = max(0.25, min(50.0, req.multiplier))
    print(f"{C.WARN}[SIM] Speed set to {sim_speed_multiplier}×{C.END}")
    return {"status": "success", "multiplier": sim_speed_multiplier}


@app.get("/api/sim/speed")
def get_sim_speed():
    return {"multiplier": sim_speed_multiplier}



@app.get("/api/stats")
def get_stats():
    """Returns mission time-series + decision log for charts and stats.html."""
    return {
        "elapsed":       round(game_logic.mission_elapsed, 1),
        "score":         game_logic.score,
        "targets_remaining": len(game_logic.targets),
        "targets_total": game_logic.targets_total,
        "enemies_count": len(game_logic.enemies),
        "status":        game_logic.status_message,
        "game_over":     game_logic.game_over,
        "escape_mode":   game_logic.escape_mode,
        "last_decision": game_logic.last_decision,
        "history":       game_logic.stats_history,
    }


@app.get("/api/stats/history")
def list_stats_history():
    """List saved JSON mission files from disk."""
    try:
        os.makedirs(STATS_DIR, exist_ok=True)
        files = sorted(os.listdir(STATS_DIR), reverse=True)
        return {"files": files[:20]}  # last 20 missions
    except Exception as e:
        return {"files": [], "error": str(e)}


@app.get("/api/stats/summary")
def get_stats_summary():
    """Aggregate all saved mission files into success/partial/failed counts + score stats."""
    try:
        os.makedirs(STATS_DIR, exist_ok=True)
        files = sorted(os.listdir(STATS_DIR))
        success = partial = failed = 0
        scores = []
        recent = []  # last 10 missions for timeline
        for fname in files:
            if not fname.endswith(".json"):
                continue
            try:
                with open(os.path.join(STATS_DIR, fname)) as f:
                    m = json.load(f)
                status = m.get("final_status", "").upper()
                score  = m.get("final_score", 0)
                ts     = m.get("timestamp", 0)
                if "SUCCESS" in status and "PARTIAL" not in status:
                    result = "success"; success += 1
                elif "PARTIAL" in status:
                    result = "partial"; partial += 1
                else:
                    result = "failed";  failed += 1
                scores.append(score)
                recent.append({"ts": ts, "result": result, "score": score})
            except Exception:
                continue
        total = success + partial + failed
        return {
            "total":   total,
            "success": success,
            "partial": partial,
            "failed":  failed,
            "best_score":  max(scores) if scores else 0,
            "avg_score":   round(sum(scores) / len(scores), 1) if scores else 0,
            "recent":  sorted(recent, key=lambda x: x["ts"])[-20:],
        }
    except Exception as e:
        return {"total": 0, "success": 0, "partial": 0, "failed": 0,
                "best_score": 0, "avg_score": 0, "recent": [], "error": str(e)}

