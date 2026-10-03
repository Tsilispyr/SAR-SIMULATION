# SAR-UGV Simulation

_A Web-Based Multi-Agent Search and Rescue Platform with UAV-Cooperative Threat Intelligence on Real-World Street Networks_

**Technical Journal, Version 13 (October 2026)**

> **Revision note.** Version 12 (June 2026) documented the single-tier ground-vehicle platform. Version 13 documents the current state of the code:
>
> - the UAV-cooperative layer and its temporal-decay threat model (July 2026);
> - the 300-mission strategy study behind the IARIA AIVTS 2026 paper;
> - front-end features that already shipped in May–June but were never documented (MapLibre 3-D monitor, robot dashboard, sensors, geofences, waypoints);
> - the findings of a full code and data review carried out in October 2026.
>
> Behaviour is described as the code implements it, verified by reading the code and by running it headlessly. Where earlier documents or the paper describe it differently, the difference is called out (§4.7, §13).

## Abstract

SAR-UGV Simulation is a browser-controllable search-and-rescue simulator. Autonomous unmanned ground vehicles (UGVs) collect targets on real OpenStreetMap street graphs while evading patrolling and pursuing adversaries. A FastAPI backend executes all simulation and decision logic. Five static HTML pages provide:

- mission planning;
- a 3-D live monitor;
- a robot-eye sensor dashboard;
- analytics;
- a scenario editor.

The ground planner routes over a risk-weighted copy of the street graph. Its edge costs combine:

- road-type multipliers;
- tiered proximity penalties around known adversaries;
- an optional geofence.

Version 13 adds an optional aerial agent (UAV). It sweeps the mission area under one of four coverage strategies and feeds time-stamped adversary sightings to the ground planner through a temporal-decay penalty,

W(d, Δt) = W_base · e^(−Δt/τ) · max(0, 1 − d / r(Δt)),

whose influence radius r(Δt) grows with the age of the observation.

A 2 × 5 factorial study (two adversary densities × a no-UAV baseline and four UAV strategies, 30 missions per cell, 300 missions in total) found no statistically significant difference between any UAV strategy and the baseline:

- success rate: Fisher's exact p ≥ 0.33, omnibus χ² p = 0.27 and 0.60;
- Mission Performance Index: Welch p ≥ 0.15.

The observed orderings are therefore directional trends only. In addition, the standard-density baseline cell was generated at a different simulation speed and has to be re-run.

The journal documents the architecture, algorithms, API and evaluation, the defects and methodological issues found in the review, and a prioritised roadmap.

**Keywords:** Search and Rescue, Unmanned Ground Vehicle, UAV-UGV Cooperation, Temporal Decay, Risk-Aware Path Planning, OpenStreetMap, Multi-Agent Systems, FastAPI, Simulation

---

## 1. Introduction

Urban search and rescue (SAR) is one of the hardest settings for autonomous robots: unknown terrain, mobile threats, multiple agents and dispersed objectives under time pressure. Simulators that use real street topology let navigation policies be tested before hardware is involved.

SAR-UGV Simulation deploys autonomous agents on genuine OpenStreetMap (OSM) street graphs for any location on Earth. OSM graphs carry real intersections, road classes and path types, a level of navigational realism that synthetic grid worlds lack. The platform evolved from a Pygame prototype (`SARGV-FIN-CC.py`). That prototype proved the core pipeline (risk-weighted routing, escape logic, terrain scoring) on a single agent and a synthetic grid. The web platform removes those constraints. Version 13 adds a second tier, an aerial agent that acts as a reconnaissance sensor for the ground planner.

### 1.1 Design Philosophy

The backend is a pure HTTP API server responsible for all simulation logic: graph construction, entity state, routing, scoring and the UAV layer. The frontend is static HTML/CSS/JavaScript that polls that API. There is no build pipeline and no JavaScript framework. The frontend container is a single `COPY` into nginx. This choice favours auditability and a low barrier to entry over abstraction. It suits an academic tool whose source should be readable as written.

### 1.2 What Changed Since v12

| Area | Status | Notes |
| --- | --- | --- |
| UAV-cooperative layer | **New (July 2026)** | `UAVAgent`, four coverage strategies, `uav_intelligence` store, temporal-decay penalties in `build_penalized_graph`, `GET /api/sim/uav`, UAV overlay in `mission.html`, `--uav-strategy` in the batch harness |
| Strategy study | **New (July–September 2026)** | 300 missions, `batch_test_v2.py` stress profile, MPI metric, statistics, weight sensitivity and tier breakdown scripts, figures |
| MapLibre GL 3-D live monitor | Present since May 2026; first documented here | Building extrusions, follow camera, sensor overlays, mission report |
| Robot dashboard (`dashboard.html`) | Present since June 2026; first documented here | Three.js chase camera, simulated detector boxes, LiDAR plot |
| Onboard sensors | Present since June 2026; first documented here | Camera (90°, 80 m, building line-of-sight) and 36-ray LiDAR (30 m) |
| Planning tools | Present since June 2026; first documented here | Geofence polygons, ordered waypoint missions, manual agent spawn |
| Code review | **New (October 2026)** | §13 and `FUTURE_WORK.md` |

### 1.3 Publications and Outputs

- S. Tsilimpokos, I. Panagiotopoulos, G. Dimitrakopoulos, _Temporal Decay Intelligence Modeling for UAV-Cooperative UGV Navigation in Urban Search and Rescue Environments_, IARIA AIVTS 2026. Camera-ready version prepared September 2026. It is built on the UAV layer (§4) and the strategy study (§9).
- _A Risk-Aware Intelligent Management System for Autonomous Path Planning of Unmanned Ground Vehicles_, an IEEE VTC manuscript on the single-tier UGV planner.
- HRF 2026 poster _Web-Based SAR Mission Simulator: Autonomous Navigation & Threat Avoidance in an Urban Environment_.

### 1.4 Organisation

| Section | Content |
| --- | --- |
| §2 | Architecture |
| §3 | Ground-agent algorithms |
| §4 | UAV layer and temporal-decay model |
| §5 | Simulation loop |
| §6 | Scoring |
| §7 | REST API |
| §8 | Frontend pages |
| §9 | Evaluation |
| §10 | Deployment |
| §11 | Design decisions |
| §12 | Future work |
| §13 | Known issues |
| §14 | Repository structure |
| §15 | Conclusion |

---

## 2. System Architecture

Two Docker containers are orchestrated by `docker-compose`:

| Container | Port | Contents |
| --- | --- | --- |
| **backend** | 8000 | FastAPI + Uvicorn on `python:3.10-slim` |
| **frontend** | 8085 | nginx serving five static pages |

The frontend polls the backend over REST/JSON:

- positions every 200 ms;
- statistics every 1 s;
- UAV state every 400 ms.

The backend additionally broadcasts the position payload every tick on a WebSocket (`/ws/simulation`). No page consumes it today.

```text
 Browser ──HTTP 200 ms / 400 ms / 1 s──▶ backend :8000  (FastAPI, 5 Hz simulation loop)
    ▲                                       │  OSMnx graph + buildings, game state, UAV,
    │                                       │  risk-weighted routing, scoring, persistence
    └──────── static pages ◀── frontend :8085 (nginx: index, mission, dashboard, stats, editor)
```

### 2.1 Backend Technology Stack

| Library | Role |
| --- | --- |
| **FastAPI / Uvicorn** | REST + WebSocket server, background simulation task (lifespan) |
| **OSMnx** | Street-graph and building-footprint download, projection, nearest-node queries |
| **NetworkX** | Weighted shortest paths (Dijkstra), BFS, connectivity |
| **pyproj** | Projected metres ↔ WGS-84 for the browser |
| **Shapely** | Building polygons, `STRtree` for camera line-of-sight and LiDAR ray casting, geofence containment |
| **SciPy** | `cKDTree` over edge midpoints for proximity and UAV penalty queries |
| **NumPy / pandas / GeoPandas** | Geometry arrays and building attribute parsing |

`Backend/requirements.txt` lists the packages without versions (see K15).

### 2.2 Frontend Technology Stack

| Library | Used in | Role |
| --- | --- | --- |
| **Leaflet 1.7.1** | `index.html`, `editor.html`, mini-map in `stats.html` | 2-D planning and editing maps |
| **MapLibre GL 3.6.2 + Turf** | `mission.html` | 3-D live monitor, building extrusions, metric circles, sensor wedges |
| **Three.js r128** | `dashboard.html` | Robot chase-camera scene |
| **Chart.js 4.4.0** | `stats.html` | Live and historical charts |
| **Orbitron / Share Tech Mono** | all | Typography |

All pages call the API at a hard-coded `http://127.0.0.1:8000` or `http://localhost:8000`. They therefore work only when the browser runs on the Docker host (F10).

### 2.3 Backend Modules

| File | Lines | Contents |
| --- | --- | --- |
| `Backend/src/main.py` | 2,568 | Enemy model, sensors, `AgentState`, `UAVAgent`, `SARGameState`, simulation loop, 36 routes |
| `Backend/src/pathfinding.py` | 122 | `compute_dead_ends`, `densify_path`, `build_penalized_graph` (proximity tiers, geofence, UAV decay) |
| `Backend/src/models.py` | 42 | Pydantic request bodies |

---

## 3. Ground-Agent Algorithms

### 3.1 Map Loading and Graph Preparation

`GET /api/map/vector` runs the following pipeline:

1. Download the street network with `ox.graph_from_point(center, dist=radius, network_type='all')`. When a geofence is set, it uses `graph_from_polygon` instead. Building footprints are fetched in parallel with `features_from_point`.
   - `dist` defines a 2R × 2R bounding box, so the mission area is a square. This is visible in the 3-D screenshots.
   - `network_type='all'` includes footways, paths and cycleways, not just drivable roads.
2. Convert to an undirected graph, which discards one-way restrictions, and keep the largest connected component.
3. Project to local metres with `ox.project_graph`. **Note:** this call returns a `MultiDiGraph` with a single, arbitrary direction per street. Several components still query this projected graph directly (see K1).
4. Multiply each edge `length` by a road-class factor (§3.2) and store a physical half-width from OSM `width`/`lanes`, used for lateral positioning (§3.8).
5. Parse building heights:
   - OSM `height` if present;
   - otherwise `building:levels` × 3.2 m, plus `roof:levels` × 1.6 m;
   - otherwise a 9 m default.

   The projected footprints are indexed in a Shapely `STRtree`.
6. Spawn entities randomly or from editor placements (`mode=custom`) and build the spawn snapshot.
7. Return road geometry in centred metres.

### 3.2 Risk-Weighted Graph (`build_penalized_graph`)

Every replan constructs an undirected copy of the graph and adds penalties to edge `length`.

**Road-class multipliers** (applied once at map load):

| Road class | Multiplier |
| --- | --- |
| motorway / trunk | 0.80 / 0.85 |
| primary / secondary | 1.00 |
| tertiary / unclassified | 1.30 / 1.40 |
| residential | 1.50 |
| living_street / service | 2.00 |
| track / cycleway | 2.50 / 3.00 |
| path / footway | 3.50 |
| steps | 4.00 |
| unknown | 1.20 |

**Proximity tiers for known enemies.** Candidate edges are found with a `cKDTree` over edge midpoints within r + 30 m, where r is the detection radius. The distance used is the closest of the midpoint and the two endpoints:

| Distance from enemy | Penalty |
| --- | --- |
| 0 m | +8,000 |
| ≤ 10 m | +3,000 |
| ≤ 30 m | +1,500 |
| ≤ detection radius | +800 |
| ≤ detection radius + 30 m | +300 |

Enemies currently chasing get a further ×1.5.

**Geofence:** +6,000 on every edge whose midpoint lies outside the polygon.

**UAV decay penalties** are added on top of all the above (§4.4).

### 3.3 Route Search

All routes call `nx.shortest_path` / `nx.shortest_path_length` with `weight='length'`. That is **Dijkstra's algorithm, not A\*** as earlier documents and the papers state. The resulting paths are cost-optimal either way. A switch to `nx.astar_path` would need a heuristic scaled by the smallest multiplier (0.8) to stay admissible. Paths are converted to centred coordinates and densified to 2 m spacing (§5.4).

### 3.4 Dead Ends

`compute_dead_ends` marks nodes of degree ≤ 1. In autonomous play they are used twice:

- target selection adds +200 to targets located on a dead-end node;
- the brave maneuver refuses dead-end targets.

The edge-level "+200 on edges into cul-de-sacs" described in v12 exists only in the manual `POST /api/path/vector` endpoint. The repository contains no data behind the "≈30 % fewer contacts" figure quoted in v12.

### 3.5 Per-Agent Decision Logic

#### Every tick, in `SARGameState.update`, each active agent

1. Runs its sensors (§3.7).
2. Applies the terrain score penalty for the edge it just traversed (§6).
3. Collects any target within 8 m.
4. Prunes waypoints within 5 m.
5. Calls `autonomous_replan`.

**`autonomous_replan`** runs every 12 ticks. It runs immediately if any enemy (ground truth) is within 15 m.

- _Known enemies_ = camera detections (patrol or aggressive) ∪ aggressive enemies within 1.1 × detection radius. The second term is a ground-truth distance check without line-of-sight.
- _Evasion threshold_ = 1.5 × detection radius if a known aggressive enemy is chasing, otherwise 0.8 × detection radius (patrols only).
- **If the nearest known enemy is inside the threshold:**
  1. Pick the best target and try a **MANEUVER** ("brave") path. This is rejected when the target is a dead end, when no path exists, or when a chasing enemy can reach the target first.
  2. If the maneuver is rejected, compute an **ESCAPE** path (§3.6). This increments `escape_events`.
- **Otherwise, pursue a target:**
  - Cost is the penalised path length, with a 25 % discount for the current target, a 10 % discount for the pre-assigned target, and +200 for dead-end targets.
  - Hysteresis keeps the current target unless the new one is ≥ 25 % cheaper or fewer than 8 waypoints remain.
  - Under threat, the route is always recomputed.
- Finally, a lateral shift moves the agent to the far side of the road from the nearest threat.

#### While moving (`advance_agent`)

- **Corner slowdown:** speed is scaled down to 30 % when the forward LiDAR rays read below 10 m.
- **Path lookahead:** if any enemy (ground truth, any type) is within 10 m of the next waypoint, the path is cleared and a replan is forced. The margin shrinks to 4 m at 24 m ahead.

**Note:** the MANEUVER branch checks reachability on the half-directed projected graph (K1). Measured on the Athens graph, every rejected maneuver in the probe runs was rejected for that reason. Threatened agents therefore almost always escape, which is not what this decision tree intends.

### 3.6 Escape Pathfinding

`find_escape_path` (aggressive chasers only):

1. Collects candidate nodes 2–8 BFS hops from the agent.
2. Scores each candidate as `min_enemy_time_to_candidate − path_cost / 4`.
3. Routes to the best candidate on the penalised graph.
4. If nothing qualifies, falls back to the one-hop neighbour farthest from the closest threat.

The BFS walks `G.neighbors()` on the projected graph, which yields successors only (K1).

### 3.7 Onboard Sensors

| Sensor | Model | Use |
| --- | --- | --- |
| Camera | 90° FOV, 80 m range, line-of-sight test against the building `STRtree`; detects enemies and targets; every tick | Feeds _known enemies_; shown in the HUD and dashboard |
| LiDAR | 36 rays at 10° spacing, 30 m range, ray-cast against buildings; every 5th tick | Corner slowdown; HUD and dashboard plots |

Perception is only partly sensor-based. The aggressive-enemy proximity check, the 15 m danger override and the path lookahead all use ground-truth positions. The "no-UAV" baseline therefore has 360° ground-truth awareness of aggressive enemies within about 55 m. This bounds how much a UAV can add (§9.7).

### 3.8 Lateral Road Positioning

Agents and enemies keep a lateral offset within the road half-width:

- enemies drift randomly while patrolling;
- agents shift away from threats.

Collection, contact and sensing use these _physical_ positions.

### 3.9 Multi-Agent Coordination

Up to four agents run simultaneously.

- **Greedy pre-assignment:** each agent receives its nearest unclaimed target. Assignments are recomputed after every collection.
- **Shared target list:** a collected target disappears for everyone, which acts as an implicit blackboard.
- **Score:** the mission score is the sum of the agent scores.

**Known defect (K2):** enemies are stepped once _per active agent_ per tick. In multi-agent missions they therefore move N times faster and flip between chase and patrol. All batch experiments are single-agent and are unaffected.

### 3.10 Adversary Models

| Type | Behaviour |
| --- | --- |
| **Patrol** | Random walk on the undirected graph, avoiding immediate backtracking |
| **Aggressive** | Patrols at patrol speed until an agent is within the detection radius, then chases along the shortest path at chase speed; reverts when the agent leaves the radius |

Both types snap to a random node after 10 consecutive path failures.

---

## 4. UAV-Cooperative Layer and Temporal-Decay Threat Model

### 4.1 Overview

```text
UAVAgent (strategy) ──detections {id: (x, y, tick)}──▶ uav_intelligence (latest sighting per enemy,
        ▲                                                pruned after 90 ticks)
        │ waypoint from strategy                              │ every replan
        │                                                     ▼
  game state (targets, agents, intel)          build_penalized_graph(+ W(d, Δt)) ──▶ Dijkstra ──▶ UGV
```

### 4.2 UAV Agent

- **Motion:** starts at the mission centre and flies in a straight line toward its current waypoint at 20 m/s in the centred metric frame. There is no altitude, wind, turn-rate or energy model.
- **Detection:** every tick, every enemy within 180 m of the UAV is reported with its exact centre-line position. There is no occlusion, noise or false positives.

### 4.3 Intelligence Store

`SARGameState.uav_intelligence` maps `enemy_id → (x, y, tick_seen)` and keeps only the latest sighting of each enemy. Entries older than 90 ticks are pruned. The link is perfect and instantaneous: both tiers share one in-process state.

### 4.4 Temporal-Decay Penalty

For every stored sighting with age Δt = `current_tick − tick_seen`, every edge whose midpoint lies within r(Δt) receives

W(d, Δt) = W_base · e^(−Δt/τ) · max(0, 1 − d / r(Δt)),  r(Δt) = max(r_min, v_drift · Δt)

where d is the distance from the edge midpoint to the sighting. Entries with e^(−Δt/τ) < 0.05 are skipped, which coincides with the 90-tick prune (e^(−3) ≈ 0.0498). The penalty is additive to the proximity tiers of §3.2. It is consumed only when the agent replans (every 12 ticks or on the 15 m danger override); new intelligence does not trigger a replan.

| Symbol | Code | Value | At 1× (UI default) | At 4× (batch runs) |
| --- | --- | --- | --- | --- |
| W_base | `W_BASE` | 1,200 cost units | – | – |
| τ | `tau` | 30 ticks | 6 s | 24 s |
| r_min | literal `60.0` | 60 m | – | – |
| v_drift | `DRIFT_RATE` | 3 m/tick | 15 m/s | 3.75 m/s |
| prune age | literal `90` in `update()` | 90 ticks | 18 s | 72 s |
| cut-off | `MIN_DECAY` | 0.05 | – | – |

All three model constants are tick-denominated. The model's physical meaning therefore changes with the speed multiplier (K4). `tau` is a function argument that no caller sets, and `W_BASE` and `DRIFT_RATE` are local constants. No parameter sweep of the model is possible without code changes.

### 4.5 Coverage Strategies

| Strategy | Implemented behaviour |
| --- | --- |
| **none** | No UAV; the agent relies on §3.5–3.7 only |
| **lawnmower** | Boustrophedon strips over the square of half-width 0.95 R, strip spacing 1.35 × 180 m = 243 m, restarting when finished. One sweep is ≈ 19 km (≈ 16 min) at R = 1,000 m and ≈ 37 km (≈ 31 min) at R = 1,500 m |
| **threat_informed** | Flies to the _stalest_ entry in the intelligence store. Falls back to the lawnmower only when the store is empty |
| **target_informed** | Flies to the midpoint between agent 0 and the centroid of the remaining targets |
| **hybrid** | Repeating 150-tick cycle: target-informed for ticks 0–45 (≈ 31 %), threat-informed for the rest. Threat-informed whenever any active agent is in escape mode |

Two consequences matter for interpreting §9:

- With τ = 24 s and pruning at 72 s, a lawnmower sighting is gone long before the strip passes again (minutes later). The store therefore holds only enemies inside, or recently inside, the 180 m footprint.
- Threat-informed only revisits enemies it already knows about, so it rarely discovers new ones. In the paired probe (§9.6) it ever detected 3–5 of 8 enemies, against 7–8 for the lawnmower.

### 4.6 Activation, API and Visualisation

- **Activation:** pass `uav_strategy=lawnmower|threat_informed|target_informed|hybrid` to `GET /api/map/vector`. The batch harness does this. Neither the planner page nor the in-monitor deploy panel offers the option yet (F3).
- **State endpoint:** `GET /api/sim/uav` returns `active`, `strategy`, `x`, `y`, `lat`, `lon`, `heading`, `detect_r`, `tick` and `intel[]`. Each `intel` item contains `enemy_id`, `x`, `y`, `ticks_ago`, `lat` and `lon`.
- **Rendering:** `mission.html` polls that endpoint every 400 ms and draws:
  - a yellow ✈ marker;
  - a dashed 180 m detection ring;
  - one orange dot per intelligence entry, fading over 90 ticks.

  The new screenshot `screenshots/aerial-and-ground-vehocles-together.png` (filename misspelled) shows this layer. The UAV marker is hard to tell apart from the gold target markers, and the legend has no UAV entry.

### 4.7 Implementation Notes (differences from the paper text)

| Topic | Paper text | Code |
| --- | --- | --- |
| Route search | A* | Dijkstra (§3.3) |
| Road graph | Directed graph with one-way streets | Undirected; one-way restrictions discarded (§3.1) |
| Patrol adversaries | Cyclic waypoint routes | Random walk (§3.10) |
| Escape event | Adversary closes within a threshold; the mission continues | The agent _chooses_ an escape path. Contact (< 5 m) incapacitates the agent and ends a single-agent mission |
| Partial outcome | At least one target extracted | > 70 % of targets collected when the last agent is incapacitated (§6) |
| Hybrid strategy | Target-informed for the first 30 % of the mission, then threat-informed | Repeating 150-tick duty cycle (§4.5) |
| Target-informed | Centroid weighted by the UGV heading | Unweighted centroid averaged with the agent position |
| Drift speed | 3 m/tick = 2.4 m/s | 3 m/tick = 3.75 m/s at 0.8 s per tick. Chasing aggressors (3.5–5 m/s in the study) can exceed it |
| E_max | 5 = observed ceiling on escape events | Exceeded in 34 of 300 runs (maximum 28) |
| V2 density | 4.4 km⁻² | 4.4 km⁻² is the V2 _easy_ tier. The tier-weighted mean is 3.9 km⁻² (circle area) or 3.0 km⁻² (the 2R × 2R area actually downloaded) |
| Baseline perception | Onboard proximity sensors only | Partly ground truth (§3.7) |

---

## 5. Simulation Loop and State Management

### 5.1 Tick Processing

`simulation_loop` sleeps 0.2 s (5 Hz) and then, under `_sim_lock`, runs one tick:

1. For each active agent: advance along the path by `speed × dt`, recompute the nearest graph node, and update its lat/lon.
2. `SARGameState.update(G, dt, cx, cy)`:
   - advance elapsed time and the tick counter;
   - move the UAV and merge its detections, then prune the store;
   - run the per-agent steps of §3.5;
   - append a statistics snapshot;
   - check success;
   - move enemies and test for contact (< 5 m between physical positions; −100 points and incapacitation);
   - check game-over.
3. Schedule the WebSocket broadcast.

### 5.2 Speed Multiplier and Contact Reliability

- **Effective time step:** each tick advances `dt = 0.2 s × multiplier`. Simulated time per tick is fixed, so processing time never changes simulated outcomes; it only stretches wall-clock time.
- **Range:** `POST /api/sim/speed` clamps the multiplier to 0.25–50×. The monitor's "100×" button therefore runs at 50×.
- **Contact reliability:** contact is a once-per-tick distance test. At 4×, an agent and an aggressor can close 8.4 m per tick against a 10 m-wide contact window, so laterally offset passes can already be missed. Above about 5× misses become frequent, which is why the batch harness runs at 4×.

### 5.3 Tick-Denominated Constants

| Constant | Ticks | At 1× | At 4× |
| --- | --- | --- | --- |
| Replan interval | 12 | 2.4 s | 9.6 s |
| Decay constant τ | 30 | 6 s | 24 s |
| Intelligence prune | 90 | 18 s | 72 s |
| UAV drift bound | 3 m/tick | 15 m/s | 3.75 m/s |
| Hybrid cycle (target phase) | 150 (46) | 30 s (9.2 s) | 120 s (36.8 s) |
| LiDAR refresh | 5 | 1 s | 4 s |

Because these constants are counted in ticks, behaviour observed at 4× does not carry over to interactive use at 1× (K4).

### 5.4 Path Densification

Paths are interpolated to 2 m spacing so that agents move continuously at every speed.

### 5.5 Spawn Snapshot and Restart

`POST /api/sim/restart` restores enemies, targets and agents from the spawn snapshot without re-downloading the map. It does **not** reset the UAV's position and waypoints, the intelligence store or the tick counter, and the snapshot does not record the UAV strategy (K3). Restarted UAV missions are therefore not identical replays.

---

## 6. Scoring Model

Score(t) = Σ (+50 per target collected) − Σ (terrain penalty per edge traversed) − 100 per enemy contact

| Road class | Terrain penalty per edge |
| --- | --- |
| motorway … secondary (and links) | 0 |
| tertiary, unclassified, residential, living_street, service | 1 |
| track, path, footway, cycleway, steps | 3 |

The terrain penalty is applied only when the edge lookup on the projected graph succeeds. Because of K1, 27–42 % of moves were not charged in the probe runs.

### Outcomes

| Outcome | Condition |
| --- | --- |
| **SUCCESS** | All targets collected |
| **PARTIAL** | The last agent is incapacitated with **more than** 70 % of targets collected |
| **FAILED** | The last agent is incapacitated with 70 % or fewer collected |

Successful and partial missions are saved to `/app/data/stats/`. **Failed missions and manual stops are not saved** (K6), so the history panel in `stats.html` can never show a failure.

---

## 7. REST API Reference

36 routes: 35 HTTP and one WebSocket. All return JSON; CORS allows every origin.

### Map and scene

| Endpoint | Method | Description |
| --- | --- | --- |
| `/api/map/vector` | GET | Download graph and buildings, spawn entities (`mode=random\|custom`, `uav_strategy`, `agent_lat/agent_lon`) and return roads |
| `/api/map/info` | GET | Current mission parameters and sim flags (does not report the UAV strategy) |
| `/api/map/current` | GET | Last `/api/map/vector` response |
| `/api/map/buildings` | GET | Building footprints (WGS-84) with heights |
| `/api/sim/scene` | GET | Roads and up to 300 buildings in centred metres (for `dashboard.html`) |
| `/api/sim/costmap` | GET | Node-level cost samples for a heat map (unused; see K10) |

### Simulation control

| Endpoint | Method | Description |
| --- | --- | --- |
| `/api/robot/position` | GET | Agents (position, path, heading, camera, LiDAR, decision), enemies, targets, score, status |
| `/api/sim/start` | POST | Start or resume |
| `/api/sim/pause` | POST | Toggle pause |
| `/api/sim/stop` | POST | Force game over (not recorded) |
| `/api/sim/restart` | POST | Restore the spawn snapshot |
| `/api/sim/reset` | GET | Legacy random re-spawn with default parameters |
| `/api/sim/clear` | GET | Wipe entities before a new deployment |
| `/api/sim/state` | GET | running / paused / game_over / elapsed / map_loaded |
| `/api/sim/speed` | GET, POST | Read or set the multiplier (0.25–50×) |
| `/api/sim/uav` | GET | UAV state and intelligence store (§4.6) |
| `/ws/simulation` | WebSocket | Per-tick position broadcast |

### Planning

| Endpoint | Method | Description |
| --- | --- | --- |
| `/api/sim/geofence` | GET, POST, DELETE | Read, set or clear a WGS-84 polygon |
| `/api/mission/waypoints` | POST, DELETE | Ordered waypoint mission (optional loop) or clear it |
| `/api/path/vector` | POST | Manual risk-aware path between two points |
| `/api/robot/telemetry` | POST | External position override (currently broken; K7) |

### Statistics

| Endpoint | Method | Description |
| --- | --- | --- |
| `/api/stats` | GET | Live time series, per-agent stats, last decision |
| `/api/stats/summary` | GET | Saved missions aggregated by agent count |
| `/api/stats/history` | GET | Latest 20 saved mission files |

### Editor and scenarios

| Endpoint | Method | Description |
| --- | --- | --- |
| `/api/editor/place`, `/api/editor/remove`, `/api/editor/clear`, `/api/editor/state` | POST, DELETE, POST, GET | Manage editor placements |
| `/api/scenario/save`, `/api/scenario/load`, `/api/scenario/list`, `/api/scenario/delete` | POST, POST, GET, DELETE | Persist named scenarios as JSON |

---

## 8. Frontend Pages

All pages share a dark operational theme (Orbitron headings, Share Tech Mono read-outs).

### 8.1 `index.html`: Mission Planner (Leaflet)

- Mission centre by map click or latitude/longitude input; radius 200–5,000 m via slider or a two-click draw tool.
- Targets (1–20), patrol and aggressive enemies (0–10 each) with speeds, agents (1–4), agent speed and detection radius.
- Scenario save and load with a placement preview and automatic `mode=custom`.
- Advanced planning:
  - geofence polygon with snap-to-close and drag/insert/delete vertices;
  - ordered waypoints with loop;
  - manual agent spawn.
- Deploy chains geofence → clear → `/api/map/vector` → waypoints.
- No UAV selector.

### 8.2 `mission.html`: Live Monitor (MapLibre GL)

- OSM raster base map with a dark filter and 3-D building extrusions.
- Geofence overlay.
- Agent markers, paths, camera-FOV wedges and LiDAR rings.
- Enemy markers with metric detection circles, and target markers.
- Per-agent HUD tiles with a LiDAR plot and camera read-out.
- Mission control (start / pause / stop / restart) and speed buttons (0.5×–100×).
- Heading-up follow camera, 3-D view toggle, and an in-page "New Mission" deploy panel.
- End banner and a mission report overlay with print/export. Polling freezes at game over; "Watch Replay" resumes it.
- UAV overlay (§4.6).

### 8.3 `dashboard.html`: Robot Dashboard (Three.js)

- Chase-camera view of the selected agent with roads and extruded buildings from `/api/sim/scene`.
- Simulated detector boxes ("SAR-YOLO") projected from camera detections, a heading bar and a compass.
- LiDAR polar plot with the camera cone and detections.
- Telemetry, detection list, decision log and LiDAR statistics.

### 8.4 `stats.html`: Analytics (Chart.js)

- KPI strip, per-agent score charts, targets remaining, escape events per agent.
- AI decision log and a Leaflet mini-map.
- Saved-mission history (doughnut, best and average score, recent list).
- A score-formula panel. Its terrain values and evasion threshold are out of date (F7).

### 8.5 `editor.html`: Scenario Editor (Leaflet)

- Tool palette (agent, target, patrol, aggressive, erase) with placement counters.
- Scenario save, load and delete.
- Mission radius and geofence tool.
- "Deploy Custom Mission" always deploys one agent with default speeds (F8).

---

## 9. Evaluation

### 9.1 Harness and Metrics

#### Runners

- `evaluation/batch_test.py` (V1, standard density) and `batch_test_v2.py` (V2, stress density) drive the REST API headlessly.
- `run_all_strategies[_v2].py` runs all five conditions back to back.
- `run_tests.sh` wraps the runners in tmux for long runs on Ubuntu/WSL2.

#### Per mission

- The map is fixed at central Athens (37.9838 N, 23.7275 E).
- Entity placement is random, as are speeds: patrol 2.0–3.0 m/s, aggressive 3.5–5.0 m/s, agent 5.5–7.0 m/s, detection radius 40–60 m.
- One agent; 4× speed; a 720 s _wall-clock_ limit.
- Results go to `batch_results_<strategy>[_v2].{json,csv}`.

#### Mission Performance Index (MPI)

MPI = 0.50 · outcome + 0.30 · max(0, 1 − escapes / 5) + 0.20 · max(0, 1 − t / 2880 s)

outcome ∈ {1, 0.5, 0} for success, partial, failure.

**Analysis scripts:** `statistical_analysis.py`, `weight_sensitivity.py`, `tier_breakdown.py`, `plot_comparison.py`, `plot_architecture.py`, and `reanalysis.py` (October 2026, which reproduces every number below).

### 9.2 June 2026 Single-Agent Batch (pre-UAV)

50 missions at **8×** over the V1 tiers:

| Outcome | Missions | Share |
| --- | --- | --- |
| Success | 44 | 88 % |
| Partial | 2 | 4 % |
| Failed | 4 | 8 % |

Results are in `evaluation/SAR_Report_GR.md`. At 8× the contact test is unreliable (§5.2), so treat these as indicative.

### 9.3 UAV Strategy Study: Design

Design: 2 density profiles × 5 conditions × 30 runs = 300 missions, July 2026.

| Tier (share of runs) | V1 patrol / aggr / targets / radius | V2 patrol / aggr / targets / radius |
| --- | --- | --- |
| Easy (20 %) | 2 / 1 / 3 / 800 m | 3 / 2 / 4 / 600 m |
| Medium (30 %) | 3 / 2 / 5 / 1,000 m | 5 / 3 / 6 / 800 m |
| Hard (30 %) | 4 / 3 / 6 / 1,200 m | 7 / 5 / 8 / 1,000 m |
| Very hard (20 %) | 5 / 4 / 7 / 1,500 m | 9 / 6 / 10 / 1,200 m |

Tier-weighted enemy density, V1 → V2 (2.6× higher in V2):

- 1.5 → 3.9 km⁻² using πR²;
- 1.2 → 3.0 km⁻² using the 2R × 2R area that is actually downloaded.

### 9.4 Results

| Condition | V1 success (Wilson 95 %) | V1 MPI | V1 escapes | V2 success (Wilson 95 %) | V2 MPI | V2 escapes |
| --- | --- | --- | --- | --- | --- | --- |
| None (baseline)* | 93.3 % (78.7–98.2) | 0.873 | 1.10 | 86.7 % (70.3–94.7) | 0.816 | 1.87 |
| Lawnmower | 93.3 % (78.7–98.2) | 0.899 | 0.57 | 76.7 % (59.1–88.2) | 0.798 | 1.30 |
| Threat-informed | 96.7 % (83.3–99.4) | 0.893 | 1.07 | 80.0 % (62.7–90.5) | 0.816 | 1.47 |
| Target-informed | 83.3 % (66.4–92.7) | 0.832 | 1.43 | 86.7 % (70.3–94.7) | 0.787 | 2.53 |
| Hybrid | 83.3 % (66.4–92.7) | 0.825 | 1.63 | 73.3 % (55.6–85.8) | 0.726 | 4.70 |

\* The V1 baseline cell was generated at 8×, not 4× (§9.7, item 1).

### 9.5 Statistical Analysis

#### No condition differs significantly from the baseline at either density

| Test | Result |
| --- | --- |
| Success rate, Fisher's exact per strategy | p = 0.33–1.00 |
| MPI, Welch's t | p = 0.15–0.99 |
| MPI, Mann–Whitney U | p = 0.36–0.84 |
| Omnibus across five conditions, χ² on success | V1 p = 0.27, V2 p = 0.60 |
| Omnibus across five conditions, Kruskal–Wallis on MPI | V1 p = 0.75, V2 p = 0.74 |
| Pooled UAV vs. baseline | V1 89.2 % vs. 93.3 % (p = 0.74); V2 79.2 % vs. 86.7 % (p = 0.45) |

**Sample size:** a 95 % interval on a single success rate spans about ±13 pp at N = 30. Detecting a 10 pp difference with 80 % power needs about 160–300 runs per arm.

**Interpretation:** the orderings in §9.4 are directional trends. The data are consistent with the UAV layer, as currently implemented, having no measurable effect on success rate. The V2 tendency of all four UAV conditions to sit at or below the baseline should be re-tested with a paired, seeded design before any mechanism is attributed to it.

**Per-tier differences** quoted in the paper (for example, a 22.2 pp drop at the medium tier) correspond to 1–2 runs out of 6–9 per tier cell.

**MPI weights:** varying them (0.50/0.30/0.20, 0.40/0.45/0.15, 0.35/0.50/0.15) leaves the V1 ordering unchanged: lawnmower > threat-informed > baseline > target-informed > hybrid.

### 9.6 Paired Headless Probe (October 2026)

To see _how_ the UAV changes behaviour, the backend was run headlessly with `evaluation/headless_probe.py`:

- real Athens graph, R = 1,000 m (2,769 nodes, 4,311 edges, 7,956 buildings), V2-medium counts;
- two seeds; within each seed every condition starts from an identical spawn and RNG stream.

#### Observations (two seeds, so illustrative only)

- **UAV often changes nothing.** In 3 of 8 UAV runs the outcome was tick-for-tick identical to the baseline. The UAV intelligence never altered a routing decision; the ground-truth local awareness (§3.7) dominates.
- **Large per-scenario swings.** Seed 11: the baseline and target-informed failed at 446 s; lawnmower, threat-informed and hybrid succeeded. Seed 23: all five succeeded, and only the lawnmower changed the trajectory (0 escape decisions vs. 15).
- **Coverage per strategy:**

| Strategy | Enemies ever detected (of 8) | UAV distance flown |
| --- | --- | --- |
| Lawnmower | 7–8 | 19 km |
| Threat-informed | 3–5 | 15–19 km |
| Hybrid | 3–4 | 14–18 km |
| Target-informed | 3 | 2–4 km |

- **Planner side effects of K1.** Every rejected MANEUVER attempt failed on directed reachability, and 27–42 % of moves skipped the terrain penalty.

### 9.7 Threats to Validity and Data Quality

1. **Speed confound in the V1 baseline.** The V1 `none` cell has an effective speed (simulated ÷ wall time) of 6.6× (minimum 6.0×) and a maximum wall time of 332 s. That matches the harness's former settings (8×, 360 s limit). At 4× the effective speed cannot exceed 4.0×, and the other nine cells run at 3.3–3.6×. That cell was therefore generated at 8×, before the harness moved to 4×. Contact detection and replan timing differ at 8×, so the V1 baseline comparison and the baseline's V1 → V2 drop are confounded. **Re-run V1 `none` at 4×.**
2. **No seeding or pairing.** Each condition saw different random scenarios, which inflates variance. Paired runs (common random numbers) would remove most of it.
3. **Timeouts counted as failures.** Two runs timed out:
   - one V1 hybrid run stalled for 7,997 s wall time, a harness or host stall rather than a mission outcome;
   - one V2 threat-informed run timed out still alive with 9 of 10 targets.

   The V1 harness also records a timeout's simulated time as 720 × 8 = 5,760 s instead of 2,880 s. The budget is wall-clock, so the effective simulated budget is about 2,400–2,600 s and depends on the host.
4. **MPI properties.**
   - The safety and efficiency terms are not conditioned on the outcome. A failure after 27 s with no targets scores 0.50, above the worst partial (0.36).
   - `escapes` counts escape _decisions_, not contacts.
   - The 5-escape normaliser is exceeded in 34 of 300 runs.
5. **Tick-denominated model constants.** The results are specific to 4× (§5.3).
6. **Ground-truth local awareness of the baseline** limits the information a UAV can add (§3.7).
7. **Planner defects affect all conditions alike** (K1), but the agent does not behave as documented (no maneuvers, under-counted terrain cost).
8. **Narrow scope.** One map centre, one UAV, a perfect link, perfect detection, and no energy model.
9. **No artifact for a decay-parameter sweep.** The repository contains no code path, parameters or results for varying τ or W_base. Any sensitivity statement about the decay model needs that experiment to be run (FUTURE_WORK P0.4).

---

## 10. Deployment

```bash
docker-compose up --build          # backend :8000, frontend :8085
# http://localhost:8085/index.html     mission planner
# http://localhost:8085/mission.html   live monitor (3-D, UAV overlay)
# http://localhost:8085/dashboard.html robot dashboard
# http://localhost:8085/stats.html     analytics
# http://localhost:8085/editor.html    scenario editor
```

- `docker-compose.yml` refers to `./backend` and `./frontend`, while the folders are `Backend` and `Frontend`. This works on Windows, macOS and WSL drives, but fails on a case-sensitive Linux file system (K14).
- Backend-only development: `cd Backend && pip install -r requirements.txt && uvicorn src.main:app --reload --port 8000`.
- `Backend/data/{stats,scenarios,osmnx_cache}` are created at runtime and are git-ignored.

Running the strategy study:

```bash
python evaluation/batch_test.py    --runs 30 --uav-strategy threat_informed   # V1
python evaluation/batch_test_v2.py --runs 30 --uav-strategy threat_informed   # V2
./evaluation/run_tests.sh [--v2]                                              # all strategies in tmux
python evaluation/reanalysis.py                                               # statistics
```

---

## 11. Design Decisions

- **REST polling over WebSockets.** At 6 m/s on a 2 m waypoint grid, a 200 ms poll moves a marker about 1.2 m, so push delivery adds little. Polling keeps every endpoint `curl`-able. A per-tick WebSocket broadcast exists for future clients but costs one full position payload per tick even with no listeners.
- **OSMnx over hand-authored maps.** Real topology anywhere, at zero authoring cost. Docker hides the GEOS, PROJ and GDAL dependencies.
- **Risk as edge cost.** A single cost function (terrain × class + proximity tiers + geofence + UAV decay) replaces explicit avoidance rules and keeps every threat source composable.
- **Temporal decay for remote intelligence.**
  - A sighting is certain at Δt = 0 and becomes less informative as the adversary moves. Exponential confidence decay plus a radius that grows at the drift bound expresses both effects in one additive penalty.
  - Pruning at e^(−3) bounds the store.
  - The current tick-based parameterisation should become time-based (K4).
- **Greedy target assignment.** O(n²) spatial spreading is sufficient at n ≤ 20 targets. Hungarian assignment is unnecessary at this scale.
- **Flat JSON scenarios and stats.** Tens of small, human-readable, diff-able files; a database would add a container without benefit.
- **Vanilla HTML.** No build step; the source is auditable as written.

---

## 12. Future Work

See `FUTURE_WORK.md` for the full roadmap with fixes and effort estimates. In priority order:

| Priority | Work |
| --- | --- |
| **P0 Reproducibility of the published study** | Re-run the V1 baseline at 4×; seeded, paired runs at higher N; make τ / W_base / v_drift configurable and run the sensitivity sweep; intermediate densities to locate any threshold; pin dependency versions |
| **P1 Correctness** | Undirected planning graph (K1); per-tick enemy stepping (K2); UAV reset on restart (K3); time-based constants (K4); continuous contact test (K5); persist failures (K6) |
| **P2 UAV in the UI** | Strategy selector, legend and marker, overlay teardown and replay fixes (F1–F3) |
| **P3 Methodology** | Outcome-conditioned MPI; censored timeouts; contacts as the safety measure; sensor-only baseline option |
| **P4 Features and research** | Risk heat map, terrain overlay, tests and CI, blackboard coordination, multi-UAV, communication loss, energy budget, learned strategy switching, Godot/ROS bridge |

---

## 13. Known Issues (October 2026 review)

IDs match `FUTURE_WORK.md`.

| ID | Severity | Issue | Location |
| --- | --- | --- | --- |
| K1 | High | `ox.project_graph` returns a half-directed `MultiDiGraph`. Directed reachability queries fail (0 / 400 random pairs reachable on Athens), the MANEUVER branch rarely works, terrain penalties are skipped for 27–42 % of moves, and escape BFS sees successors only | `main.py:1599-1602`, `970`, `1170`, `905` |
| K2 | High (multi-agent) | Enemies step once per active agent per tick | `main.py:1241-1243` |
| K3 | Medium | Restart keeps UAV state, intelligence and tick; the snapshot omits `uav_strategy`; reset drops the UAV | `main.py:2081-2136`, `2017` |
| K4 | Medium | τ, prune age, drift, replan interval and hybrid cycle are counted in ticks, so behaviour depends on the speed multiplier | `pathfinding.py:29,100`, `main.py:572,991,1154` |
| K5 | Medium | Contact is a single 5 m test per tick; it tunnels at higher speeds, the UI offers 8–100×, and the backend caps at 50× | `main.py:1251`, `2156` |
| K6 | Medium | Failed missions are never saved, so failure statistics in `stats.html` are always zero | `main.py:1270-1271` |
| K7 | Medium | `/api/robot/telemetry` calls `update()` with a stale signature and reads a removed attribute | `main.py:1449-1468` |
| K8 | Low | Target-informed UAV `min(..., key=lambda a: not a.active or True)` always picks agent 0 | `main.py:538` |
| K9 | Low | Intelligence ages mix `UAVAgent._tick` and `current_tick` (in sync today, fragile) | `main.py:592`, `pathfinding.py:105` |
| K10 | Low | `/api/sim/costmap` sums total edge cost rather than the penalty, uses ground-truth enemies and ignores UAV intelligence; `/api/map/current` returns an exception object instead of raising it | `main.py:1879-1922`, `1774` |
| K11 | Low | GET endpoints read state without the lock; `/api/sim/uav` can iterate the store while it is being mutated | `main.py:2182` |
| K12 | Low | Proximity and UAV queries use edge midpoints only; edges longer than twice the query radius can escape (0.2–0.4 % of edges; 1.5 % for fresh UAV sightings) | `pathfinding.py:74,114` |
| K13 | Low | Scenario load and delete use the unsanitised name in a path (`../`); with open CORS any page can call them | `main.py:2321,2337` |
| K14 | Low | Compose paths `./backend` and `./frontend` do not match `Backend` and `Frontend` on case-sensitive file systems | `docker-compose.yml` |
| K15 | Low | Unpinned requirements (OSMnx 1.x → 2.x API changes) | `Backend/requirements.txt` |
| F1 | Medium | UAV overlay teardown calls `.remove()` on `true`; ring and intel markers persist after a non-UAV deploy, and the error repeats every 400 ms | `mission.html:1418,1451` |
| F2 | Low | "Watch Replay" restarts position and stats polling but not UAV polling | `mission.html:1077-1078` |
| F3 | Medium | No UAV strategy control in any page; `/api/map/info` omits it; no legend entry; UAV marker resembles targets | `index.html`, `mission.html` |
| F4 | Low | Speed buttons up to 100× (backend 50×), with no reliability warning | `mission.html:525-532` |
| F5 | Low | Deploy relabels the first `<button>` (Draw Region) instead of the Deploy button | `index.html:556` |
| F6 | Low | `<select id="scenario-select"` is missing `>` | `index.html:355` |
| F7 | Low | Per-agent-count table never renders (`#hs-by-agents` missing); the formula panel shows wrong penalties and threshold | `stats.html:536-561,885` |
| F8 | Low | Editor deploy hard-codes one agent and default speeds; each click moves the mission centre | `editor.html:404,494-496` |
| F9 | Low | Dashboard fetches the full scene every 200 ms; stale meshes are never removed or disposed | `dashboard.html:317-330,437-458` |
| F10 | Low | Hard-coded backend host (`127.0.0.1` / `localhost`) | all pages |
| F11 | Low | Turf loaded without a version pin | `mission.html:625` |
| E1 | High (evaluation) | V1 baseline generated at 8× (§9.7) | `batch_results_none.*` |
| E2 | Medium | Runs unseeded and unpaired | `batch_test*.py` |
| E3 | Medium | Timeouts scored as failures; V1 timeout time recorded at 5,760 s; wall-clock budget | `batch_test.py:176-185` |
| E4 | Medium | MPI not outcome-conditioned; escape count saturates | `batch_test_v2.py:55-60` |
| E5 | Low | `--clear-old` points at `evaluation/Backend/data/stats` (no-op) | `batch_test.py:313` |
| D1 | Low | `README.md` scoring values, "A*", Leaflet monitor, 100×, dead-end penalty and "≥ 70 %" are out of date | `README.md` |

---

## 14. Repository Structure

```text
SAR-SIMULATION/
├── Backend/
│   ├── Dockerfile, requirements.txt
│   └── src/
│       ├── main.py          FastAPI app, game state, sensors, UAVAgent, routes (2,568 lines)
│       ├── pathfinding.py   build_penalized_graph (tiers, geofence, UAV decay), dead ends, densify
│       └── models.py        Pydantic request models
│   # Backend/data/{stats,scenarios,osmnx_cache} created at runtime, git-ignored
├── Frontend/
│   ├── Dockerfile           nginx: COPY . /usr/share/nginx/html
│   ├── index.html           mission planner (Leaflet)
│   ├── mission.html         live monitor (MapLibre GL 3-D, UAV overlay)
│   ├── dashboard.html       robot dashboard (Three.js)
│   ├── stats.html           analytics (Chart.js)
│   └── editor.html          scenario editor (Leaflet)
├── evaluation/
│   ├── batch_test.py, batch_test_v2.py, run_all_strategies*.py, run_tests.sh, progress.py
│   ├── batch_results_<strategy>[_v2].{json,csv}   300-mission strategy study
│   ├── batch_results.{json,csv}, SAR_Report_GR.md  June single-agent batch and report
│   ├── statistical_analysis.py, weight_sensitivity.py, tier_breakdown.py, reanalysis.py
│   ├── headless_probe.py    paired headless runs against the backend module
│   ├── plot_comparison.py, plot_architecture.py, *.png
│   └── logs/                local batch logs (*.log is git-ignored)
├── screenshots/
├── SARGV-FIN-CC.py          original Pygame prototype
├── docker-compose.yml
├── README.md, FUTURE_WORK.md, SAR_UGV_Journal.md, SAR_UGV_Overview.docx
└── .gitignore
```

---

## 15. Conclusion

SAR-UGV Simulation is a working research testbed: real OSM topology, a composable risk-cost planner, onboard sensor models, a 3-D monitor and robot dashboard, scenario tooling, and a headless batch harness. Version 13 adds a UAV-cooperative layer whose temporal-decay model turns stale aerial sightings into spatially spreading, exponentially fading route costs. A first 300-mission comparison of four coverage strategies is complete.

That comparison does not yet show a statistically detectable benefit from the UAV. The review also found a speed confound in one baseline cell, unpaired sampling, and planner defects that change agent behaviour. The next step is to make the study reproducible and decisive:

- fix the half-directed planning graph and the tick-based constants;
- re-run with seeded, paired scenarios at a sample size large enough to resolve a 10 pp effect.

With those in place, the platform supports the questions the project set out to answer: when aerial reconnaissance helps a ground agent, which coverage strategy helps most, and at what adversary density the benefit disappears.

_— End of Document —_
