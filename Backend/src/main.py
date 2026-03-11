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

STATS_DIR     = '/app/data/stats'
SCENARIOS_DIR = '/app/data/scenarios'
ROBOT_SPEED   = 6.0
TICK_RATE     = 0.2
sim_speed_multiplier: float = 1.0

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
    def __init__(self, _id, enemy_type, start_node, G, cx, cy,
                 patrol_speed=2.5, aggr_speed=4.5, detection_radius=50.0):
        self.id = _id
        self.type = "patrol"
        self.base_type = enemy_type
        self.G = G.to_undirected() if G.is_directed() else G
        self.cx = cx
        self.cy = cy
        self.detection_radius = detection_radius
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
#  PER-AGENT STATE
# ---------------------------------------------------------------------------
AGENT_COLORS = ['#3d78ff', '#22c55e', '#f97316', '#e879f9']

class AgentState:
    """Encapsulates one autonomous agent's position, path, and score."""
    def __init__(self, agent_id: int, start_node, G, cx: float, cy: float,
                 speed_ms: float = 6.0):
        self.id              = agent_id
        self.color           = AGENT_COLORS[agent_id % 4]
        n                    = G.nodes[start_node]
        self.robot_x         = round(n['x'] - cx, 2)
        self.robot_y         = round(n['y'] - cy, 2)
        self.robot_node      = start_node
        self.robot_speed_ms  = speed_ms
        self.robot_auto_path = []
        self.current_target_id: int | None = None
        self.escape_mode     = False
        self.replan_timer    = 0
        self.last_decision   = "Initialising..."
        self._last_robot_node = None
        self.score           = 0
        self.escape_events   = 0
        self.terrain_penalty = 0
        self.targets_collected = 0
        self.dead_ends       = set()
        self.active          = True   # False when incapacitated by contact
        self.assigned_target_id = None  # greedy pre-assignment hint


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


def save_stats_to_disk(stats_history, score, status, n_agents=1):
    """Persist mission stats to /app/data/stats/<timestamp>.json"""
    try:
        os.makedirs(STATS_DIR, exist_ok=True)
        filename = os.path.join(STATS_DIR, f"mission_{int(time.time())}.json")
        with open(filename, 'w') as f:
            json.dump({
                "timestamp": time.time(),
                "final_score": score,
                "final_status": status,
                "n_agents": n_agents,
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
        self.enemies              = []
        self.targets              = []
        self.targets_total        = 0
        self.game_over            = False
        self.status_message       = "Active"
        self._proximity_cooldowns = {}
        self.stats_history        = []
        self.mission_elapsed      = 0.0
        self.stats_saved          = False
        self.agents: list         = []   # list[AgentState]
        self.dead_ends            = set()
        # Simulation control
        self.last_telemetry_time  = 0.0
        self.sim_running          = False
        self.sim_paused           = False
        self.robot_speed_ms       = 6.0
        self._spawn_snapshot      = None

    # ------------------------------------------------------------------
    @property
    def score(self):
        """Total score = sum of all agent scores."""
        return sum(a.score for a in self.agents) if self.agents else 0

    @property
    def escape_mode(self):
        return any(a.escape_mode for a in self.agents)

    @property
    def last_decision(self):
        if not self.agents:
            return "Waiting for mission start..."
        return self.agents[0].last_decision

    # ------------------------------------------------------------------
    def spawn_objects(self, G, cx, cy, n_patrol=3, n_aggressive=2, n_targets=5,
                      patrol_speed=2.5, aggr_speed=4.5, n_agents=1,
                      agent_speed=6.0, detection_radius=50.0,
                      custom_placements=None):
        """Spawn all entities. custom_placements overrides random when mode=custom."""
        self.enemies.clear()
        self.targets.clear()
        self.agents.clear()
        self.game_over            = False
        self.status_message       = "Mission Start"
        self._proximity_cooldowns = {}
        self.stats_history        = []
        self.mission_elapsed      = 0.0
        self.stats_saved          = False
        self.dead_ends            = compute_dead_ends(G)
        self.last_telemetry_time  = 0.0
        self.robot_speed_ms       = agent_speed

        all_nodes = list(G.nodes())

        if custom_placements:
            # ── Custom editor placements ──
            for p in custom_placements:
                nnode = ox.distance.nearest_nodes(G, p['lon_proj'], p['lat_proj'])
                if p['type'] == 'target':
                    nn = G.nodes.get(nnode, {})
                    self.targets.append({
                        'id': len(self.targets),
                        'x': round(nn['x'] - cx, 2),
                        'y': round(nn['y'] - cy, 2),
                        'node': nnode
                    })
                elif p['type'] in ('patrol', 'aggressive'):
                    self.enemies.append(
                        Enemy(len(self.enemies), p['type'], nnode, G, cx, cy,
                              patrol_speed, aggr_speed, detection_radius))
            # Spawn agents at specified positions (or random if none given)
            agent_placements = [p for p in custom_placements if p['type'] == 'agent']
            all_agent_nodes = [ox.distance.nearest_nodes(
                G, p['lon_proj'], p['lat_proj']) for p in agent_placements]
            while len(all_agent_nodes) < n_agents:
                all_agent_nodes.append(random.choice(all_nodes))
            for aid in range(n_agents):
                ag = AgentState(aid, all_agent_nodes[aid], G, cx, cy, agent_speed)
                ag.dead_ends = self.dead_ends
                self.agents.append(ag)
        else:
            # ── Random placement ──
            if len(all_nodes) > n_targets:
                target_nodes = random.sample(all_nodes, n_targets)
                for i, node in enumerate(target_nodes):
                    n = G.nodes[node]
                    self.targets.append({
                        'id': i,
                        'x': round(n['x'] - cx, 2),
                        'y': round(n['y'] - cy, 2),
                        'node': node
                    })
            else:
                target_nodes = []

            available = [n for n in all_nodes if n not in target_nodes]
            total_enemies = n_patrol + n_aggressive
            if len(available) > total_enemies + n_agents:
                enemy_nodes = random.sample(available, total_enemies)
                for i in range(n_patrol):
                    self.enemies.append(Enemy(i, 'patrol', enemy_nodes[i], G, cx, cy,
                                              patrol_speed, aggr_speed, detection_radius))
                for i in range(n_aggressive):
                    idx = n_patrol + i
                    self.enemies.append(Enemy(idx, 'aggressive', enemy_nodes[idx], G, cx, cy,
                                              patrol_speed, aggr_speed, detection_radius))
                remaining = [n for n in available if n not in enemy_nodes]
                agent_starts = random.sample(remaining, min(n_agents, len(remaining)))
                while len(agent_starts) < n_agents:
                    agent_starts.append(random.choice(all_nodes))
            else:
                agent_starts = random.sample(all_nodes, min(n_agents, len(all_nodes)))

            for aid in range(n_agents):
                ag = AgentState(aid, agent_starts[aid], G, cx, cy, agent_speed)
                ag.dead_ends = self.dead_ends
                self.agents.append(ag)

        self.targets_total = len(self.targets)
        # Greedy distinct target pre-assignment
        self._assign_targets_to_agents(G)

        self._spawn_snapshot = {
            'enemies': [{'id': e.id, 'base_type': e.base_type, 'node': e.current_node,
                         'patrol_speed': patrol_speed, 'aggr_speed': aggr_speed,
                         'detection_radius': detection_radius}
                        for e in self.enemies],
            'targets': [dict(t) for t in self.targets],
            'agents':  [{'id': a.id, 'node': a.robot_node, 'speed': a.robot_speed_ms}
                        for a in self.agents],
        }
        self.sim_running = False
        self.sim_paused  = False
        print(f"{C.OK}[SPAWN] {len(self.agents)} agent(s), {len(self.targets)} targets, "
              f"{len(self.enemies)} enemies{C.END}")

    def _assign_targets_to_agents(self, G):
        """Greedily pre-assign distinct targets to agents (closest unassigned first)."""
        assigned = set()
        # Sort agents by id for determinism
        for ag in self.agents:
            best_t, best_cost = None, float('inf')
            for t in self.targets:
                if t['id'] in assigned:
                    continue
                dx = t['x'] - ag.robot_x
                dy = t['y'] - ag.robot_y
                cost = math.hypot(dx, dy)
                if cost < best_cost:
                    best_cost = cost
                    best_t = t
            if best_t is None and self.targets:
                best_t = self.targets[0]  # all assigned, share
            if best_t:
                ag.assigned_target_id = best_t['id']
                assigned.add(best_t['id'])

    # ------------------------------------------------------------------
    # ADVANCE AGENT (called by background simulation loop per agent)
    # ------------------------------------------------------------------
    def advance_agent(self, ag: AgentState, delta_t: float):
        """Move agent along its path at agent.robot_speed_ms m/s."""
        if not ag.robot_auto_path:
            return
        dist_to_move = ag.robot_speed_ms * delta_t
        while ag.robot_auto_path and dist_to_move > 0:
            wp = ag.robot_auto_path[0]
            dx = wp['x'] - ag.robot_x
            dy = wp['y'] - ag.robot_y
            d  = math.hypot(dx, dy)
            if d < 0.3:
                ag.robot_x, ag.robot_y = wp['x'], wp['y']
                ag.robot_auto_path.pop(0)
            elif dist_to_move >= d:
                ag.robot_x, ag.robot_y = wp['x'], wp['y']
                ag.robot_auto_path.pop(0)
                dist_to_move -= d
            else:
                ag.robot_x += (dx / d) * dist_to_move
                ag.robot_y += (dy / d) * dist_to_move
                dist_to_move = 0

    # ------------------------------------------------------------------
    # ESCAPE PATH (per-agent)
    # ------------------------------------------------------------------
    def find_escape_path(self, G, ag: AgentState, penalized_G, cx, cy):
        threatening = [e for e in self.enemies
                       if e.base_type == 'aggressive' and e.type == 'aggressive']
        if not threatening:
            return []
        robot_node = ag.robot_node
        candidates = set()
        frontier = {robot_node}; visited = {robot_node}
        for hop in range(8):
            nxt = set()
            for n in frontier:
                for nb in G.neighbors(n):
                    if nb not in visited:
                        nxt.add(nb); visited.add(nb)
            if hop >= 2:
                candidates |= nxt
            frontier = nxt
            if not frontier:
                break
        if not candidates:
            return []
        best_score, best_node = -999999.0, None
        for cand in candidates:
            cx_n, cy_n = G.nodes[cand]['x'], G.nodes[cand]['y']
            enemy_turns = min(
                math.hypot((cx_n - cx) - e.x, (cy_n - cy) - e.y) / max(e.speed, 0.1)
                for e in threatening)
            try:
                path_len = nx.shortest_path_length(penalized_G, robot_node, cand, weight='length')
            except:
                continue
            sc = enemy_turns - path_len / 4.0
            if sc > best_score:
                best_score = sc; best_node = cand
        if best_node is None:
            return []
        try:
            node_path = nx.shortest_path(penalized_G, robot_node, best_node, weight='length')
            return densify_path(_extract_path_coords_centred(G, node_path, cx, cy), 2.0)
        except:
            return []

    # ------------------------------------------------------------------
    # BRAVE PATH (per-agent)
    # ------------------------------------------------------------------
    def find_brave_path(self, G, ag: AgentState, target, cx, cy):
        threatening = [e for e in self.enemies
                       if e.base_type == 'aggressive' and e.type == 'aggressive']
        target_node = target['node']
        robot_node  = ag.robot_node
        try:
            if G.degree(target_node) <= 1:
                return []
        except:
            pass
        try:
            node_path   = nx.shortest_path(G, robot_node, target_node, weight='length')
            robot_steps = nx.shortest_path_length(G, robot_node, target_node, weight='length')
        except:
            return []
        for e in threatening:
            try:
                e_node = ox.distance.nearest_nodes(current_map_data['G_proj'], e.x + cx, e.y + cy)
                enemy_steps = nx.shortest_path_length(
                    current_map_data['G_proj'], e_node, target_node, weight='length')
            except:
                enemy_steps = 999999
            if robot_steps + 1 >= enemy_steps:
                return []
        coords = _extract_path_coords_centred(current_map_data['G_proj'], node_path, cx, cy)
        return densify_path(coords, 2.0)

    # ------------------------------------------------------------------
    # AUTONOMOUS RE-ROUTING (per-agent)
    # ------------------------------------------------------------------
    REPLAN_INTERVAL = 5

    def autonomous_replan(self, G, ag: AgentState, cx, cy):
        ag.replan_timer += 1
        if ag.replan_timer < self.REPLAN_INTERVAL and ag.robot_auto_path:
            return
        ag.replan_timer = 0
        if not self.targets:
            return

        penalized_G = build_penalized_graph(G, self.enemies, cx, cy)
        threatening  = [e for e in self.enemies
                        if e.base_type == 'aggressive' and e.type == 'aggressive']
        if threatening:
            nearest_dist = min(
                math.hypot(e.x - ag.robot_x, e.y - ag.robot_y) for e in threatening)
            det_r = self.enemies[0].detection_radius if self.enemies else 50.0
            if nearest_dist < 1.2 * det_r:
                best_t = self._pick_best_target(penalized_G, ag)
                if best_t:
                    brave = self.find_brave_path(penalized_G, ag, best_t, cx, cy)
                    if brave:
                        ag.robot_auto_path   = brave
                        ag.escape_mode       = False
                        ag.current_target_id = best_t['id']
                        ag.last_decision     = (f"BRAVE: T{best_t['id']} ({nearest_dist:.0f}m threat)")
                        return
                esc = self.find_escape_path(G, ag, penalized_G, cx, cy)
                if esc:
                    ag.robot_auto_path   = esc
                    ag.escape_mode       = True
                    ag.current_target_id = None
                    ag.escape_events     += 1
                    ag.last_decision     = (f"ESCAPE: fleeing ({nearest_dist:.0f}m)")
                    return

        best_t = self._pick_best_target(penalized_G, ag)
        if not best_t:
            return
        if best_t['id'] == ag.current_target_id and ag.robot_auto_path:
            return
        try:
            node_path = nx.shortest_path(penalized_G, ag.robot_node, best_t['node'], weight='length')
            ag.robot_auto_path   = densify_path(_extract_path_coords_centred(G, node_path, cx, cy), 2.0)
            ag.escape_mode       = False
            ag.current_target_id = best_t['id']
            ag.last_decision     = (f"Targeting T{best_t['id']} | pts:{len(ag.robot_auto_path)}")
            print(f"{C.AGENT}[A{ag.id}] {ag.last_decision}{C.END}")
        except Exception as e:
            print(f"{C.ERR}[A{ag.id}] replan failed: {e}{C.END}")

    def _pick_best_target(self, penalized_G, ag: AgentState):
        """Pick lowest-cost target; prefer assigned target to avoid collision with other agents."""
        best_t, best_cost = None, float('inf')
        for t in self.targets:
            try:
                cost = nx.shortest_path_length(penalized_G, ag.robot_node, t['node'], weight='length')
                if t['node'] in ag.dead_ends:
                    cost += 200
                # Bonus for pre-assigned target to guide agents apart
                if t['id'] == ag.assigned_target_id:
                    cost -= cost * 0.15  # 15% preference discount
                if cost < best_cost:
                    best_cost = cost; best_t = t
            except:
                pass
        return best_t

    # ------------------------------------------------------------------
    # MAIN UPDATE LOOP
    # ------------------------------------------------------------------
    def update(self, G, delta_t, cx, cy):
        """Tick all agents, move enemies, check collisions and mission state."""
        if self.game_over:
            return

        self.mission_elapsed += delta_t

        # Per-agent terrain, target collection, path pruning, replan
        for ag in self.agents:
            if not ag.active:
                continue

            # Terrain penalty
            if ag._last_robot_node is not None and ag.robot_node != ag._last_robot_node:
                edge_data = G.get_edge_data(ag._last_robot_node, ag.robot_node)
                if edge_data:
                    data = min(edge_data.values(), key=lambda x: x.get('length', 999999))
                    hw = data.get('highway', '')
                    if isinstance(hw, list): hw = hw[0]
                    penalty = HIGHWAY_SCORE_PENALTY.get(hw, 0)
                    if penalty:
                        ag.score -= penalty
                        ag.terrain_penalty += penalty
            ag._last_robot_node = ag.robot_node

            # Target collection (any agent collects = removed for all)
            for t in self.targets[:]:
                dist = math.hypot(t['x'] - ag.robot_x, t['y'] - ag.robot_y)
                if dist < 8.0:
                    self.targets.remove(t)
                    ag.score += 50
                    ag.targets_collected += 1
                    if ag.current_target_id == t['id']:
                        ag.current_target_id  = None
                        ag.robot_auto_path    = []
                    # Reset assigned hint for ALL agents to force re-assignment
                    for other in self.agents:
                        if other.assigned_target_id == t['id']:
                            other.assigned_target_id = None
                    self._assign_targets_to_agents(G)
                    print(f"{C.OK}[A{ag.id}] Target #{t['id']} collected! Remaining:{len(self.targets)}{C.END}")

            # Path pruning
            while ag.robot_auto_path:
                wp = ag.robot_auto_path[0]
                if math.hypot(wp['x'] - ag.robot_x, wp['y'] - ag.robot_y) < 5.0:
                    ag.robot_auto_path.pop(0)
                else:
                    break

            # Replan
            self.autonomous_replan(G, ag, cx, cy)

        # Record time-series snapshot (per-agent score keys for chart lookup)
        snap = {
            't':       round(self.mission_elapsed, 1),
            'score':   self.score,
            'targets': len(self.targets),
            'agents':  [{'id': a.id, 'score': a.score, 'escape': a.escape_mode}
                        for a in self.agents]
        }
        for a in self.agents:
            snap[f'score_{a.id}'] = a.score
        self.stats_history.append(snap)

        # Mission success
        if not self.targets:
            self.status_message = 'MISSION SUCCESS!'
            self.game_over = True
            if not self.stats_saved:
                save_stats_to_disk(self.stats_history, self.score,
                                   self.status_message, len(self.agents))
                self.stats_saved = True
            return

        # Enemy movement & contact
        active_agents = [a for a in self.agents if a.active]
        all_incapacitated = True
        for ag in active_agents:
            for enemy in self.enemies:
                enemy.move(ag.robot_node, ag.robot_x, ag.robot_y, delta_t)
                dist = math.hypot(enemy.x - ag.robot_x, enemy.y - ag.robot_y)
                warn_key = f'prox_{enemy.id}_{ag.id}'
                if dist < enemy.detection_radius * 2.0:
                    if self._proximity_cooldowns.get(warn_key, 0) <= 0:
                        self._proximity_cooldowns[warn_key] = 60
                    else:
                        self._proximity_cooldowns[warn_key] -= 1
                if dist < 5.0:
                    ag.active = False
                    ag.score -= 100
                    print(f"{C.ERR}[A{ag.id}] CONTACT with Enemy-{enemy.id}!{C.END}")
            if ag.active:
                all_incapacitated = False

        # Game over when all agents incapacitated
        if all_incapacitated and self.agents:
            targets_collected = self.targets_total - len(self.targets)
            frac = targets_collected / max(self.targets_total, 1)
            self.status_message = ('MISSION PARTIAL SUCCESS (>70% targets)'
                                   if frac > 0.70 else 'MISSION FAILED: ENEMY CONTACT')
            self.game_over = True
            if not self.stats_saved:
                save_stats_to_disk(self.stats_history, self.score,
                                   self.status_message, len(self.agents))
                self.stats_saved = True


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
    """5 Hz tick: advances all agents, runs game update, updates lat/lon."""
    while True:
        await asyncio.sleep(TICK_RATE)
        with _sim_lock:
            G = current_map_data.get("G_proj")
            if G is None or game_logic.game_over:
                continue
            if not game_logic.sim_running or game_logic.sim_paused:
                continue
            if time.time() - game_logic.last_telemetry_time < 2.0:
                continue
            try:
                cx = current_map_data["center_x"]
                cy = current_map_data["center_y"]
                G.graph['center_x_raw'] = cx
                G.graph['center_y_raw'] = cy
                effective_dt = TICK_RATE * sim_speed_multiplier
                transformer  = current_map_data.get("transformer")

                for ag in game_logic.agents:
                    if not ag.active:
                        continue
                    game_logic.advance_agent(ag, effective_dt)
                    ag.robot_node = ox.distance.nearest_nodes(G, ag.robot_x + cx, ag.robot_y + cy)
                    # Update lat/lon for this agent
                    if transformer and 'crs' in G.graph:
                        lon, lat = transformer.transform(ag.robot_x + cx, ag.robot_y + cy)
                        robot_state["agents"][ag.id] = {"lat": lat, "lon": lon}

                # Primary lat/lon = agent 0
                if game_logic.agents and transformer and 'crs' in G.graph:
                    a0 = game_logic.agents[0]
                    lon0, lat0 = transformer.transform(a0.robot_x + cx, a0.robot_y + cy)
                    robot_state["lat"] = lat0
                    robot_state["lon"] = lon0

                game_logic.update(G, effective_dt, cx, cy)
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
robot_state = {"lat": None, "lon": None, "agents": {}}

# Editor state — temporary placements before deploying
editor_state: list = []   # list of {type, lat, lon, lon_proj, lat_proj}
custom_mode: bool = False  # True when /api/map/vector is called with mode=custom


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
                "agents": [], "score": 0, "status": "No map",
                "escape_mode": False, "game_over": False}

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

    # Per-agent position + path
    agents_out = []
    for ag in game_logic.agents:
        ag_st = robot_state["agents"].get(ag.id, {})
        path_ll = []
        for p in ag.robot_auto_path[:50]:
            try:
                pl, pa = transformer.transform(p['x'] + cx, p['y'] + cy)
                path_ll.append([pa, pl])
            except:
                pass
        agents_out.append({
            "id":         ag.id,
            "color":      ag.color,
            "lat":        ag_st.get("lat"),
            "lon":        ag_st.get("lon"),
            "auto_path":  path_ll,
            "score":      ag.score,
            "escape_mode": ag.escape_mode,
            "active":     ag.active,
            "last_decision": ag.last_decision,
        })

    # Backward compat: primary robot = agent 0
    primary = agents_out[0] if agents_out else {}

    return {
        "lat":        primary.get("lat") or robot_state["lat"],
        "lon":        primary.get("lon") or robot_state["lon"],
        "enemies":    enemies_latlon,
        "targets":    targets_latlon,
        "auto_path":  primary.get("auto_path", []),
        "agents":     agents_out,
        "score":      game_logic.score,
        "status":     game_logic.status_message,
        "escape_mode": game_logic.escape_mode,
        "game_over":   game_logic.game_over,
    }


@app.get("/api/map/vector")
def fetch_vector_map(lat: float = 38.900, lon: float = 22.433, radius: int = 1500,
                     n_patrol: int = 3, n_aggr: int = 2, n_targets: int = 5,
                     patrol_speed: float = 2.5, aggr_speed: float = 4.5,
                     agent_speed: float = 6.0, n_agents: int = 1,
                     detection_radius: float = 50.0, mode: str = 'random'):
    try:
        print(f"\n{C.OSM}[OSM] Map Request: lat={lat}, lon={lon}, radius={radius}, "
              f"agents={n_agents}, mode={mode}{C.END}")

        G_directed = ox.graph_from_point((lat, lon), dist=radius, network_type='all')
        G          = G_directed.to_undirected()
        largest_cc = max(nx.connected_components(G), key=len)
        G          = G.subgraph(largest_cc).copy()
        G_proj     = ox.project_graph(G)

        for u, v, k, data in G_proj.edges(keys=True, data=True):
            hw   = data.get('highway', '')
            if isinstance(hw, list): hw = hw[0]
            mult = HIGHWAY_MULTIPLIER.get(hw, 1.2)
            if mult != 1.0:
                data['length'] = data.get('length', 1.0) * mult

        nodes_gdf = ox.graph_to_gdfs(G_proj, edges=False)
        min_x, min_y, max_x, max_y = nodes_gdf.total_bounds
        bbox_cx = (min_x + max_x) / 2.0
        bbox_cy = (min_y + max_y) / 2.0
        center_node = ox.distance.nearest_nodes(G_proj, bbox_cx, bbox_cy)
        center_x    = G_proj.nodes[center_node]['x']
        center_y    = G_proj.nodes[center_node]['y']

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
            "n_agents":    n_agents,
            "detection_radius": detection_radius,
            "center_lat":  lat,
            "center_lon":  lon,
            "radius":      radius,
        })
        robot_state["agents"] = {}

        # Custom placements need projected coords
        placements = None
        if mode == 'custom' and editor_state:
            placements = []
            for p in editor_state:
                # Convert lat/lon → proj using the new transformer (inverse)
                inv_tf = Transformer.from_crs("epsg:4326", G_proj.graph['crs'], always_xy=True)
                px, py = inv_tf.transform(p['lon'], p['lat'])
                placements.append({**p, 'lon_proj': px, 'lat_proj': py})

        game_logic.spawn_objects(
            G_proj, center_x, center_y,
            n_patrol, n_aggr, n_targets,
            patrol_speed, aggr_speed, n_agents, agent_speed,
            detection_radius, placements)

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
    """Instantly wipes enemies, targets, and agents (pre-map-load flush)."""
    game_logic.enemies.clear()
    game_logic.targets.clear()
    game_logic.agents.clear()
    game_logic._proximity_cooldowns.clear()
    game_logic.stats_history.clear()
    game_logic.mission_elapsed  = 0.0
    game_logic.game_over        = False
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
        cx = current_map_data["center_x"]; cy = current_map_data["center_y"]
        game_logic.spawn_objects(
            G, cx, cy,
            current_map_data.get("n_patrol", 3),
            current_map_data.get("n_aggr", 2),
            current_map_data.get("n_targets", 5),
            current_map_data.get("patrol_speed", 2.5),
            current_map_data.get("aggr_speed", 4.5),
            current_map_data.get("n_agents", 1),
            current_map_data.get("agent_speed", 6.0),
            current_map_data.get("detection_radius", 50.0))
        return {"status": "reset", "note": "No snapshot; used random re-spawn"}

    with _sim_lock:
        cx, cy = current_map_data["center_x"], current_map_data["center_y"]
        # Restore enemies
        game_logic.enemies.clear()
        for es in snap["enemies"]:
            e = Enemy(es["id"], es["base_type"], es["node"], G, cx, cy,
                      es["patrol_speed"], es["aggr_speed"], es.get("detection_radius", 50.0))
            game_logic.enemies.append(e)
        # Restore targets
        game_logic.targets       = [dict(t) for t in snap["targets"]]
        game_logic.targets_total = len(game_logic.targets)
        game_logic.dead_ends     = compute_dead_ends(G)
        # Restore agents
        game_logic.agents.clear()
        robot_state["agents"] = {}
        for ags in snap["agents"]:
            ag = AgentState(ags["id"], ags["node"], G, cx, cy, ags["speed"])
            ag.dead_ends = game_logic.dead_ends
            game_logic.agents.append(ag)
        game_logic._assign_targets_to_agents(G)
        # Reset mission state
        game_logic.game_over              = False
        game_logic.status_message         = "Mission Restart"
        game_logic.mission_elapsed        = 0.0
        game_logic.stats_history.clear()
        game_logic.stats_saved            = False
        game_logic._proximity_cooldowns.clear()
        game_logic.last_telemetry_time    = 0.0
        game_logic.sim_running            = False
        game_logic.sim_paused             = False

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
        "elapsed":           round(game_logic.mission_elapsed, 1),
        "score":             game_logic.score,
        "targets_remaining": len(game_logic.targets),
        "targets_total":     game_logic.targets_total,
        "enemies_count":     len(game_logic.enemies),
        "status":            game_logic.status_message,
        "game_over":         game_logic.game_over,
        "escape_mode":       game_logic.escape_mode,
        "last_decision":     game_logic.last_decision,
        "history":           game_logic.stats_history,
        "agents_stats":      [
            {
                "id":               a.id,
                "color":            a.color,
                "score":            a.score,
                "targets_collected": a.targets_collected,
                "escape_events":    a.escape_events,
                "terrain_penalty":  a.terrain_penalty,
                "active":           a.active,
                "last_decision":    a.last_decision,
                "escape_mode":      a.escape_mode,
            } for a in game_logic.agents
        ],
    }


# ---------------------------------------------------------------------------
# EDITOR ENDPOINTS
# ---------------------------------------------------------------------------

class PlacementRequest(BaseModel):
    type: str   # 'agent' | 'target' | 'patrol' | 'aggressive'
    lat: float
    lon: float


@app.post("/api/editor/place")
def editor_place(req: PlacementRequest):
    editor_state.append({"type": req.type, "lat": req.lat, "lon": req.lon})
    return {"status": "ok", "count": len(editor_state)}


@app.post("/api/editor/clear")
def editor_clear():
    editor_state.clear()
    return {"status": "cleared"}


@app.delete("/api/editor/remove")
def editor_remove(idx: int):
    if 0 <= idx < len(editor_state):
        editor_state.pop(idx)
        return {"status": "removed", "count": len(editor_state)}
    return {"status": "invalid index"}


@app.get("/api/editor/state")
def get_editor_state():
    return {"placements": editor_state}


# ---------------------------------------------------------------------------
# SCENARIO SAVE / LOAD
# ---------------------------------------------------------------------------

class ScenarioSaveRequest(BaseModel):
    name: str
    params: dict = {}


@app.post("/api/scenario/save")
def scenario_save(req: ScenarioSaveRequest):
    try:
        os.makedirs(SCENARIOS_DIR, exist_ok=True)
        safe_name = "".join(c for c in req.name if c.isalnum() or c in (' ', '_', '-')).strip()
        if not safe_name:
            raise HTTPException(400, "Invalid scenario name")
        data = {
            "name":       safe_name,
            "placements": list(editor_state),
            "params":     req.params,
            "saved_at":   time.time(),
        }
        path = os.path.join(SCENARIOS_DIR, f"{safe_name}.json")
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        return {"status": "saved", "name": safe_name}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(500, str(e))


@app.get("/api/scenario/list")
def scenario_list():
    try:
        os.makedirs(SCENARIOS_DIR, exist_ok=True)
        files = sorted(os.listdir(SCENARIOS_DIR))
        scenarios = []
        for f in files:
            if not f.endswith('.json'):
                continue
            try:
                with open(os.path.join(SCENARIOS_DIR, f)) as fh:
                    d = json.load(fh)
                scenarios.append({"name": d.get("name", f), "file": f,
                                   "saved_at": d.get("saved_at", 0),
                                   "params": d.get("params", {})})
            except:
                continue
        return {"scenarios": scenarios}
    except Exception as e:
        return {"scenarios": [], "error": str(e)}


@app.post("/api/scenario/load")
def scenario_load(body: dict):
    name = body.get("name", "")
    path = os.path.join(SCENARIOS_DIR, f"{name}.json")
    if not os.path.exists(path):
        raise HTTPException(404, "Scenario not found")
    try:
        with open(path) as f:
            data = json.load(f)
        editor_state.clear()
        editor_state.extend(data.get("placements", []))
        return {"status": "loaded", "name": name, "params": data.get("params", {}),
                "placements": editor_state}
    except Exception as e:
        raise HTTPException(500, str(e))


@app.delete("/api/scenario/delete")
def scenario_delete(name: str):
    path = os.path.join(SCENARIOS_DIR, f"{name}.json")
    if os.path.exists(path):
        os.remove(path)
        return {"status": "deleted"}
    raise HTTPException(404, "Not found")


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
    """Aggregate all saved mission files; group by n_agents (1-4)."""
    try:
        os.makedirs(STATS_DIR, exist_ok=True)
        files = sorted(os.listdir(STATS_DIR))
        # per-agent-count buckets
        buckets: dict = {}   # n_agents -> {success,partial,failed,scores,recent}
        all_scores = []
        all_recent = []
        for fname in files:
            if not fname.endswith(".json"):
                continue
            try:
                with open(os.path.join(STATS_DIR, fname)) as f:
                    m = json.load(f)
                status  = m.get("final_status", "").upper()
                score   = m.get("final_score", 0)
                ts      = m.get("timestamp", 0)
                n_ag    = m.get("n_agents", 1)
                if "SUCCESS" in status and "PARTIAL" not in status:
                    result = "success"
                elif "PARTIAL" in status:
                    result = "partial"
                else:
                    result = "failed"
                b = buckets.setdefault(n_ag, {"success": 0, "partial": 0,
                                               "failed": 0, "scores": [], "recent": []})
                b[result] += 1
                b["scores"].append(score)
                b["recent"].append({"ts": ts, "result": result, "score": score})
                all_scores.append(score)
                all_recent.append({"ts": ts, "result": result, "score": score, "n_agents": n_ag})
            except Exception:
                continue

        def bucket_summary(b):
            total = b["success"] + b["partial"] + b["failed"]
            return {
                "total":      total,
                "success":    b["success"],
                "partial":    b["partial"],
                "failed":     b["failed"],
                "best_score": max(b["scores"]) if b["scores"] else 0,
                "avg_score":  round(sum(b["scores"]) / len(b["scores"]), 1) if b["scores"] else 0,
                "recent":     sorted(b["recent"], key=lambda x: x["ts"])[-20:],
            }

        by_agents = {str(k): bucket_summary(v) for k, v in buckets.items()}
        overall_total = sum(v["success"] + v["partial"] + v["failed"] for v in buckets.values())
        return {
            "total":      overall_total,
            "success":    sum(v["success"] for v in buckets.values()),
            "partial":    sum(v["partial"] for v in buckets.values()),
            "failed":     sum(v["failed"]  for v in buckets.values()),
            "best_score": max(all_scores) if all_scores else 0,
            "avg_score":  round(sum(all_scores) / len(all_scores), 1) if all_scores else 0,
            "recent":     sorted(all_recent, key=lambda x: x["ts"])[-20:],
            "by_agents":  by_agents,   # keyed by str(n_agents)
        }
    except Exception as e:
        return {"total": 0, "success": 0, "partial": 0, "failed": 0,
                "best_score": 0, "avg_score": 0, "recent": [], "by_agents": {},
                "error": str(e)}


@app.get("/api/map/info")
def get_map_info():
    """Return current map params so mission.html can self-deploy."""
    return {
        "has_map":         current_map_data.get("G_proj") is not None,
        "lat":             current_map_data.get("center_lat"),
        "lon":             current_map_data.get("center_lon"),
        "radius":          current_map_data.get("radius", 1500),
        "n_patrol":        current_map_data.get("n_patrol", 3),
        "n_aggr":          current_map_data.get("n_aggr", 2),
        "n_targets":       current_map_data.get("n_targets", 5),
        "patrol_speed":    current_map_data.get("patrol_speed", 2.5),
        "aggr_speed":      current_map_data.get("aggr_speed", 4.5),
        "agent_speed":     current_map_data.get("agent_speed", 6.0),
        "n_agents":        current_map_data.get("n_agents", 1),
        "detection_radius":current_map_data.get("detection_radius", 50.0),
        "sim_running":     game_logic.sim_running,
        "sim_paused":      game_logic.sim_paused,
        "game_over":       game_logic.game_over,
    }

