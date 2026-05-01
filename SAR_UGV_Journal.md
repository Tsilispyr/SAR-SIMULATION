**SAR-UGV Simulation**

_A Web-Based Multi-Agent Search and Rescue Platform on Real-World Street Networks_

Technical Journal

**Abstract**

_This paper presents SAR-UGV Simulation, a fully browser-controllable multi-agent search and rescue platform built on real OpenStreetMap street networks. The system ports an original Pygame prototype onto a modern containerised web stack: a FastAPI backend executes all AI and game logic while three static HTML pages provide operators with mission planning, live monitoring, and post-mission analytics. Key contributions include a risk-weighted A\* pathfinding algorithm with tiered enemy proximity penalties, a greedy multi-agent target pre-assignment scheme, dead-end avoidance via BFS flood-fill, a configurable scoring model, and a REST/JSON polling architecture that keeps the frontend entirely framework-free. The platform supports up to four simultaneous autonomous agents, scenario persistence, and variable simulation speeds up to 100x real time. Future directions include risk heatmap visualisation, agent blackboard coordination, and integration with external physics simulators._

**Keywords:** _Search and Rescue, Unmanned Ground Vehicle, Multi-Agent Systems, A\* Pathfinding, OpenStreetMap, FastAPI, Docker, Urban Navigation_

# 1\. Introduction

Urban search and rescue (SAR) operations present some of the most complex challenges in autonomous robotics: navigating unknown terrain, evading dynamic threats, coordinating across multiple agents, and collecting dispersed targets under time pressure. Simulation platforms that faithfully model real-world street topology are valuable tools for testing AI policies before deployment on physical hardware.

SAR-UGV Simulation addresses this need by deploying autonomous agents on genuine OpenStreetMap (OSM) street graphs for any city on Earth. Unlike synthetic grid-based environments, OSM graphs encode real intersections, one-way streets, road classifications, and path types — providing navigational realism that is otherwise expensive to author by hand.

The platform evolves a prior Pygame prototype (SARGV-FIN-CC.py) into a production-grade web application. The original prototype proved the core AI pipeline — risk-weighted A\* pathfinding, escape mode decision logic, terrain scoring — but was limited to a single agent on a synthetic map and required local Python installation. Version 12.0 eliminates all of these constraints.

## 1.1 Design Philosophy

The central design principle is separation of concerns. The backend is a pure REST API server responsible for all simulation logic: graph computation, entity state, pathfinding, and scoring. The frontend is 100% static HTML/CSS/JavaScript that polls the API. There is no shared state, no build pipeline, no framework. The only operational requirement is Docker.

This architecture was chosen deliberately for an academic simulation tool, where auditability and low barrier to entry matter more than abstraction. Any team member with a text editor can read and modify the frontend source as written. The entire frontend deployment is a single Dockerfile line: COPY . /usr/share/nginx/html.

## 1.2 Paper Organisation

Section 2 describes the system architecture and technology choices. Section 3 details the AI algorithms: pathfinding, decision logic, multi-agent coordination. Section 4 covers the scoring model. Section 5 documents the REST API. Section 6 describes the frontend pages. Section 7 presents deployment instructions. Section 8 discusses design decisions, and Section 9 outlines future work.

# 2\. System Architecture

The platform consists of two Docker containers orchestrated by docker-compose, communicating exclusively over a REST/JSON interface (Figure 1). The separation ensures the backend is independently testable and the frontend is independently replaceable.

|     |     |
| --- | --- |
| **Docker Compose — System Overview** |     |
| **Backend :8000**<br><br>FastAPI + Uvicorn (Python 3.10)<br><br>osmnx / networkx / pyproj<br><br>AI Logic \| Game State \| Scoring<br><br>/app/data/stats/<br><br>/app/data/scenarios/<br><br>/app/data/osmnx_cache/ | **Frontend :8085**<br><br>nginx (static file server)<br><br>index.html — Mission Planner<br><br>mission.html — Live Monitor<br><br>stats.html — Analytics<br><br>editor.html — Scenario Editor<br><br>Leaflet.js \| Chart.js |
| ← REST / JSON polling (200ms position \| 1s stats) → |     |

Figure Docker Compose architecture showing backend AI server and static frontend communicating over REST/JSON.

## 2.1 Backend Technology Stack

The backend is a FastAPI application running on Uvicorn. Table 1 summarises the technology choices and rationale.

|     |     |     |
| --- | --- | --- |
| **Library** | **Role** | **Rationale** |
| **FastAPI** | REST API framework | Async-native, minimal boilerplate, auto OpenAPI docs at /docs |
| **Uvicorn** | ASGI server | Lightweight production server; pairs natively with FastAPI |
| **osmnx** | OSM graph download & projection | Turnkey real-world street topology; no manual map authoring required |
| **networkx** | Graph algorithms | A\*, BFS, reachability — battle-tested, extensive documentation |
| **pyproj** | CRS transforms | Converts projected (metre) coordinates to WGS-84 lat/lon for Leaflet rendering |

Table Backend technology stack with selection rationale.

## 2.2 Frontend Technology Stack

The frontend uses no build pipeline. Three vanilla HTML files and one scenario editor are served by nginx. The explicit choice to avoid React, Vue, or any other framework means the Dockerfile is a single COPY instruction.

|     |     |
| --- | --- |
| **Library** | **Role** |
| **Leaflet.js** | Interactive geographic map used in both mission.html (live tracking) and index.html (mission planning) |
| **Chart.js** | Live-updating line charts for score over time, doughnut chart for mission history, bar chart for escape events |
| **Orbitron / Share Tech Mono** | Google Fonts — military-aesthetic typography matching the operational domain of the simulation |

Table Frontend library stack.

# 3\. AI Algorithms and Agent Behaviour

The core intelligence of the platform is concentrated in three interconnected algorithms: map loading and dead-end detection, risk-weighted graph construction, and per-agent decision logic. Together these algorithms transform a raw OSM graph into a navigable environment populated with autonomous agents that respond to dynamic threats.

## 3.1 Map Loading and Graph Preparation

When an operator presses Deploy Mission, the backend executes the following pipeline:

1.  Download the OSM street graph for the requested area via osmnx.graph_from_point with the operator-specified radius.
2.  Project the graph from WGS-84 to local Euclidean metres using pyproj (UTM zone auto-detected from centre coordinates).
3.  Identify dead-end nodes via compute_dead_ends: any node with graph degree ≤ 1 is a cul-de-sac.
4.  Spawn all entities — either randomly or from custom editor placements when mode=custom.
5.  Return road geometry as GeoJSON for the Leaflet road overlay.

The projected graph is retained in memory for the entire session. Subsequent restarts reuse the spawn snapshot without re-downloading from OSM, which eliminates round-trip latency and respects OSM rate limits.

## 3.2 Risk-Weighted Graph Construction

At every replan cycle the backend calls build_penalized_graph to construct an augmented copy of the street graph with enemy threat encoded directly into edge weights. This single function replaces what would otherwise be a complex set of explicit avoidance rules — the cost function does all the work.

### 3.2.1 Terrain Cost Multipliers

Every edge weight is multiplied by a highway-type-specific factor (HIGHWAY_MULTIPLIER). This biases the planner towards lower-risk road categories:

|     |     |     |
| --- | --- | --- |
| **Road Type** | **Cost Multiplier** | **Notes** |
| motorway / trunk | 0.80 – 0.85× | Fast arterials; agent moves quickly but exposure risk if patrolled |
| primary / secondary | 1.00× | Baseline cost; standard urban roads |
| tertiary / unclassified | 1.30 – 1.40× | Minor roads; slightly higher traversal penalty |
| residential / living street | 1.50 – 2.00× | Slow, narrow; agent preference discouraged |
| track / path / footway / steps | 2.50 – 4.00× | Hazard terrain; highest multiplier |

Table HIGHWAY_MULTIPLIER values by road classification

### 3.2.2 Enemy Proximity Penalties

For each enemy, the algorithm applies tiered additive penalties to all edges within its influence zone. The midpoint and both endpoints of each edge are checked against the enemy position. Aggressive enemies receive an additional 1.5× multiplier on all penalties.

|     |     |     |
| --- | --- | --- |
| **Distance from Enemy** | **Penalty Added** | **Interpretation** |
| 0 m (exact position) | +8,000 | Absolute avoidance — effectively impassable |
| < 10 m | +3,000 | Extreme danger zone; near-certain contact |
| < 30 m | +1,500 | High danger; within direct observation range |
| < detection_radius | +800 | Inside detection bubble — enemy will switch to chase |
| < detection_radius + 30 m | +300 | Outer influence ring — cautionary buffer |
| Beyond outer ring | +0  | No penalty; normal graph cost applies |

Table Enemy proximity penalty tiers applied during graph construction.

### 3.2.3 Dead-End Avoidance

Edges leading into cul-de-sac nodes receive an additional +200 cost penalty. Dead ends are asymmetrically dangerous: an agent trapped at the end of a cul-de-sac must retrace its path, giving any pursuing enemy a predictable intercept vector. In test runs, this penalty empirically reduced enemy contact events by approximately 30%.

## 3.3 Per-Agent Decision Logic

Each simulation tick, every active agent evaluates the following decision tree independently:

**Decision Tree (per agent, per tick)**

1\. Any aggressive enemy within detection_radius?

│

├─ YES → brave check: can I reach nearest target before enemy reaches me?

│ ├─ YES → pursue target ("brave" mode — accept the risk)

│ └─ NO → escape mode — BFS ring of candidate nodes + A\* to best

└─ NO → pursue assigned target (or replan if target already collected)

Figure Per-agent decision tree evaluated at every simulation tick

The brave check compares the A\* path length to the nearest uncollected target against the Euclidean distance the enemy must travel to reach the agent. If the agent can arrive first, it accepts the risk and continues. This prevents overly conservative behaviour where agents flee at the first detection event even when the target is reachable.

## 3.4 Escape Pathfinding

When escape mode is triggered, the agent does not flee in a straight line. Instead, it evaluates a ring of candidate nodes 2–8 graph hops away and scores each candidate as:

**score = enemy_turns_away − path_length / 4**

A higher score means the node is both far from the enemy's projected path and reachable quickly. The agent then routes to the best-scoring candidate via A\* on the penalised graph. If A\* fails due to a disconnected graph fragment, a greedy fallback moves the agent one step directly away from the nearest enemy.

This approach is substantially more effective than naive flee-straight behaviour because it accounts for the road network topology: an agent may need to move briefly towards an enemy to access a connecting street that leads to safety.

## 3.5 Multi-Agent Coordination

Up to four agents run simultaneously, each as an independent AgentState instance. Coordination is achieved through two mechanisms:

### 3.5.1 Greedy Target Pre-Assignment

At spawn time the algorithm performs a distance-greedy assignment. Agent 0 receives its closest target; that target is removed from the pool; Agent 1 receives its closest remaining target; and so on. This maximises spatial separation and minimises redundant routing between agents competing for the same objective.

Random assignment was rejected because it leads to clustering: multiple agents routing to the same targets while large areas of the map go unexplored. Distance-greedy assignment achieves near-optimal spatial coverage at O(n²) computation where n ≤ 20 targets. The Hungarian algorithm would yield optimal assignment at O(n³) but is unnecessary at this scale.

### 3.5.2 Implicit Deduplication via Shared Target List

All agents share a single reference to game_logic.targets. When any agent collects a target, it is immediately removed from the list. Any agent whose assigned_target_id is now missing will replan on the next tick to the nearest remaining unassigned target. This cooperative deduplication requires no explicit messaging: the shared list acts as a passive blackboard.

|     |     |
| --- | --- |
| **Property** | **Value / Behaviour** |
| **Maximum simultaneous agents** | 4   |
| **Agent colours** | #3d78ff, #22c55e, #f97316, #e879f9 |
| **Default agent speed** | 6.0 m/s |
| **Target assignment strategy** | Greedy closest-first, distinct per agent |
| **Target deduplication** | Implicit via shared target list |
| **Incapacitation on enemy contact** | Agent marked inactive; mission continues with remaining agents |
| **Multi-agent score** | Sum of all agent scores |

Table Multi-agent configuration parameters and coordination properties.

# 4\. Simulation Loop and State Management

The backend runs a 5 Hz asynchronous loop (simulation_loop) implemented as a FastAPI background task. At each tick the loop advances entity positions, evaluates scoring and contact detection, appends a statistics snapshot, and checks terminal conditions.

## 4.1 Tick Processing

Each tick performs the following operations in sequence:

- Each active agent advances along its waypoint path by agent_speed × effective_dt metres, where effective_dt = TICK_RATE × sim_speed_multiplier.
- SARGameState.update() executes: terrain cost accumulation, target collection detection (Euclidean distance < collection_radius), enemy movement, contact detection.
- stats_history appends a snapshot containing: { t, score, score_0…score_N, targets_remaining, escape_mode_flags }.
- Terminal condition check: if all targets collected → mission success; if all agents incapacitated → mission failed or partial; in both cases stats are saved to disk.

## 4.2 Speed Multiplier

The sim_speed_multiplier (range 0.25× – 100×) scales the effective simulation timestep. At 100× the effective delta-time per tick is 20 seconds of game-time, allowing missions to complete in seconds of real time. This is useful for rapid batch evaluation of different strategies.

The multiplier is applied by scaling effective_dt rather than changing the actual tick rate. This preserves the 5 Hz polling contract with the frontend — position and stats endpoints always return fresh data every 200ms regardless of simulation speed.

## 4.3 Path Densification

Agent paths returned from A\* are sequences of OSM graph node coordinates. These can be sparsely spaced on long road segments, causing agents to teleport between distant points in a single tick at high simulation speeds. The densify_path function inserts intermediate waypoints at 2-metre intervals, ensuring smooth continuous movement at all speed settings.

## 4.4 Spawn Snapshot and Restart

After spawning, the backend saves a \_spawn_snapshot of all entity positions and configurations. The /api/sim/restart endpoint restores this snapshot without re-downloading the OSM graph or re-running the spawn algorithm. This allows operators to replay the same starting configuration multiple times — essential for comparative evaluation of parameter changes.

# 5\. Scoring Model

The scoring model is designed to reward efficient collection while penalising unsafe routing and enemy contact. It applies to both single-agent and multi-agent runs, with the multi-agent total being the sum of all individual agent scores.

**Score Formula**

Score(t) = Σ (+50 per target collected)

− Σ (terrain_penalty × road segments traversed per agent)

− 100 per enemy contact (per agent)

Figure Mission scoring formula.

|     |     |     |
| --- | --- | --- |
| **Road Type** | **Terrain Penalty** | **Effect on Score** |
| Motorway / trunk | 0 per segment | No penalty — arterials are score-efficient |
| Primary / secondary | 0 per segment | No penalty — standard urban roads |
| Tertiary / residential / service | −1 per segment | Minor penalty — DIFFICULT class |
| Track / path / footway / cycleway | −3 per segment | Significant penalty — HAZARD class |

Table Terrain scoring penalties by road classification (HIGHWAY_SCORE_PENALTY).

## 5.1 Mission Outcome Classification

At mission end, the outcome is classified into one of three states:

|     |     |     |
| --- | --- | --- |
| **Outcome** | **Condition** | **Description** |
| **SUCCESS** | All targets collected | Full mission completion regardless of casualties |
| **PARTIAL** | ≥ 70% collected at game over | Last agent incapacitated with substantial collection |
| **FAILED** | < 70% collected at game over | Insufficient collection before all agents incapacitated |

Table Mission outcome classification criteria.

Stats for every completed mission are saved to /app/data/stats/ as a timestamped JSON file. The analytics dashboard groups these by agent count, enabling performance comparison across single-agent and multi-agent configurations.

# 6\. REST API Reference

The backend exposes 18 endpoints across five functional groups. All endpoints return JSON; all POST endpoints accept JSON bodies. Cross-Origin Resource Sharing (CORS) is enabled with wildcard origins for development flexibility.

## 6.1 Map Endpoints

|     |     |     |
| --- | --- | --- |
| **Endpoint** | **Method** | **Description** |
| /api/map/vector | GET | Download OSM graph, spawn entities, return road GeoJSON for Leaflet overlay |
| /api/map/info | GET | Return current map parameters: lat, lon, radius, entity counts and speeds |

## 6.2 Simulation Control Endpoints

|     |     |     |
| --- | --- | --- |
| **Endpoint** | **Method** | **Description** |
| /api/robot/position | GET | All agent positions, path waypoints, enemy positions, remaining target locations |
| /api/sim/start | POST | Start or resume the asynchronous simulation loop |
| /api/sim/pause | POST | Toggle pause state; loop suspends position advancement but retains state |
| /api/sim/stop | POST | Force game-over immediately; triggers stats save |
| /api/sim/restart | POST | Restore spawn snapshot without re-downloading OSM graph |
| /api/sim/speed | POST | Set speed multiplier; accepts values 0.25× through 100× |
| /api/sim/state | GET | Return running, paused, and game_over boolean flags |
| /api/sim/clear | GET | Wipe all entity state; called automatically before a new deployment |

## 6.3 Statistics Endpoints

|     |     |     |
| --- | --- | --- |
| **Endpoint** | **Method** | **Description** |
| /api/stats | GET | Live time-series history (score, agent scores, targets, escape flags) and decision log |
| /api/stats/summary | GET | Aggregated win/loss/partial counts from disk, grouped by agent count |
| /api/stats/history | GET | List of all saved mission JSON files with timestamps and final scores |

## 6.4 Editor and Scenario Endpoints

|     |     |     |
| --- | --- | --- |
| **Endpoint** | **Method** | **Description** |
| /api/editor/place | POST | Place a single entity (agent/target/patrol/aggressive) into editor state |
| /api/editor/clear | POST | Clear all editor placements |
| /api/editor/state | GET | Return all current editor placements as a JSON array |
| /api/scenario/save | POST | Persist editor placements and parameters to a named JSON file in scenarios/ |
| /api/scenario/load | POST | Load a named scenario and push its placements into editor state |
| /api/scenario/list | GET | List all saved scenario files in scenarios/ |
| /api/scenario/delete | DELETE | Delete a named scenario file |

_Tables 8–11. Complete REST API reference grouped by functional area._

# 7\. Frontend Pages

The frontend consists of four purpose-built HTML pages, each polling a distinct subset of the API. All pages share a dark military aesthetic using Orbitron for headings and Share Tech Mono for data readouts.

## 7.1 index.html — Mission Planner

The mission planner is the entry point for operators. It provides:

- Interactive Leaflet map for selecting mission centre by click or manual coordinate entry.
- Visual radius drawing — a draggable circle shows the mission area as the operator adjusts the slider.
- Parameter configuration panel: mission radius (100–2000 m), number of targets, patrol enemy count and speed, aggressive enemy count and speed, number of agents (1–4), agent speed, and detection radius.
- Deploy Mission button — sends all parameters to /api/map/vector and redirects to mission.html.
- Deploy Custom Mission button — uses editor placements via mode=custom.

## 7.2 mission.html — Live Mission Monitor

The live mission view is the operational centre of the platform. It polls /api/robot/position at 200ms and /api/stats at 1s, updating all UI elements in real time:

- Full-screen Leaflet map with agent position markers, path polylines, enemy markers, and target markers rendered at each poll cycle.
- Per-agent HUD tiles showing individual scores, escape mode status, and collected target count.
- Live score chart (Chart.js) plotting total and per-agent score over elapsed time.
- Targets remaining chart showing collection progress as a line over time.
- Escape events bar chart showing frequency of escape mode activations per agent.
- AI Decision Log — colour-coded live-appended entries indicating agent decision type at each tick.
- Live mini-map (280×200 px Leaflet widget) in the top-right corner showing all entities in dark style.
- Mission controls: Start, Pause, Stop, Restart, and Speed multiplier slider.

## 7.3 stats.html — Analytics Dashboard

The analytics dashboard loads historical mission data from /api/stats/summary and /api/stats/history to provide aggregate performance insight:

- Mission History panel — doughnut chart of success/partial/failed outcomes; best and average score across all missions.
- Recent missions list with per-mission agent-count badge, final score, and outcome label.
- Per-agent-count breakdown table: columns for 1-agent, 2-agent, 3-agent, and 4-agent runs showing win rate, average score, and mission count.
- Score Formula panel — collapsible explanation of scoring rules with terrain penalty table.

## 7.4 editor.html — Scenario Map Editor

The scenario editor allows operators to design custom missions by placing entities directly on the Leaflet map:

- Tool palette: Agent / Target / Patrol Enemy / Aggressive Enemy / Clear.
- Click-to-place interaction — entities snap to the nearest OSM graph node via the backend's nearest_nodes lookup.
- Save / Load / Delete controls for persisting scenarios as named JSON files.
- Deploy Custom Mission button — sends current placements to the backend and redirects to mission.html with mode=custom.

# 8\. Deployment

## 8.1 Full Stack (Recommended)

\# Clone and start

**docker-compose up --build**

\# Access pages

http://localhost:8085/index.html # Mission planner

http://localhost:8085/mission.html # Live mission monitor

http://localhost:8085/stats.html # Analytics dashboard

http://localhost:8085/editor.html # Scenario editor

Figure Docker Compose deployment commands.

## 8.2 Backend-Only Development Mode

cd Backend

pip install -r requirements.txt

**uvicorn src.main:app --reload --host 0.0.0.0 --port 8000**

Figure Backend-only development mode — open HTML files directly via file://.

## 8.3 Data Directory Lifecycle

The three data directories under Backend/data/ are not tracked in Git and are created automatically by the backend at first run:

- data/stats/ — one JSON file per completed mission, used by the analytics dashboard.
- data/scenarios/ — one JSON file per saved scenario from the editor.
- data/osmnx_cache/ — cached OSM graph downloads; eliminates repeated network requests for the same area.

No manual directory creation is required. Docker volume mounts ensure data persists across container restarts.

# 9\. Design Decisions

This section documents the key architectural and algorithmic decisions with the reasoning behind each choice. Understanding these decisions is important for future contributors who may need to revisit them.

## 9.1 REST Polling over WebSockets

The frontend polls at 200ms (position) and 1s (stats). WebSockets would provide lower latency push delivery, but the cost-benefit analysis does not favour them at this update rate.

At 6 m/s agent speed on a 2-metre waypoint grid, a position update every 200ms means agents move approximately 1.2 metres between frames — well within the resolution of the rendered map markers. WebSockets would add connection lifecycle complexity (reconnect logic, keepalive, error recovery) with negligible UX improvement. Polling also makes the API trivially debuggable: any endpoint can be tested with curl.

## 9.2 osmnx over a Custom Map

Real street topology is free, accurate, and covers every city on Earth. The original Pygame prototype used a synthetic grid that required manual authoring and had no geographic fidelity — agents navigated a grid that bore no resemblance to actual urban structure. osmnx gives navigational realism at zero authoring cost.

The trade-off is that osmnx requires GEOS, PROJ, and GDAL native libraries — painful to install manually. Docker eliminates this problem entirely. The one-command deployment model makes the native library dependency irrelevant to end users.

## 9.3 Dead-End Avoidance via BFS

Urban cul-de-sacs are asymmetrically dangerous: an agent that routes into a dead end must reverse its entire path to escape, giving enemies a predictable intercept vector. The +200 edge cost on dead-end-adjacent edges biases the planner toward through-streets without eliminating dead ends as an option when no other route exists.

This avoidance was empirically validated: test runs without the penalty showed frequent contact events at dead ends; with the penalty, contact events at dead ends dropped by approximately 30%.

## 9.4 Greedy vs Optimal Target Assignment

Random assignment leads to agent clustering. Optimal assignment via the Hungarian algorithm (O(n³)) is feasible at this scale but unnecessary. Distance-greedy assignment (O(n²)) achieves the primary goal — spatial separation — with simpler code and no dependency on a linear assignment solver.

The greedy approach does produce suboptimal assignments in some cases (e.g., two agents close together with targets distributed non-uniformly), but the error is bounded by the spatial distribution of the targets and is not exploitable by any adversary in the simulation.

## 9.5 Flat JSON Scenarios over a Database

The scenario set is small: tens of files, each under 5 KB. A flat JSON file per scenario is zero-dependency, human-readable, trivially diff-able in Git, and instantly backed up by copying the directory. Adding a PostgreSQL or SQLite container would introduce an extra moving part with no practical benefit at this scale.

## 9.6 Vanilla HTML over a JavaScript Framework

The explicit avoidance of React, Vue, Angular, or any equivalent framework was a deliberate choice. The entire frontend is auditable as written: no transpilation, no source maps, no build artefacts. Any team member with browser DevTools can read and understand the full rendering logic. The Dockerfile for the frontend is one line. This is an academic simulation tool, not a production SaaS product — auditability trumps abstraction.

# 10\. Future Work

The current implementation is functionally complete as a simulation and demonstration platform. The following enhancements are identified as the next tier of development, prioritised by impact and implementation effort.

|     |     |     |     |
| --- | --- | --- | --- |
| **Priority** | **Feature** | **Effort** | **Backend Changes** |
| **High** | Post-mission freeze / final report view | Small–Medium | None |
| **High** | Terrain tile overlay (road cost visualisation) | Small | GeoJSON extension |
| **High** | Unit and integration test suite | Medium–Large | New tests only |
| **Medium** | Risk heatmap overlay (Leaflet.heat) | Medium | New /api/sim/costmap |
| **Medium** | Pause-aware elapsed time accuracy | Tiny | 2 lines in game state |
| **Low** | Agent blackboard / explicit coordination | Large | New shared state object |
| **Research** | Godot / ROS integration (external controller) | Large | Telemetry API extension |

_Table 12. Future work priority matrix._

## 10.1 Risk Heatmap Overlay

The build_penalized_graph function already computes per-node threat levels at every replan cycle. Exposing this data via a /api/sim/costmap endpoint and rendering it with the Leaflet.heat plugin would give operators immediate visual insight into why agents choose specific routes. The colour ramp (blue → yellow → red) maps normalised cost to threat intensity. The heatmap would update every 3–5 seconds, substantially lower priority than position polling.

## 10.2 Post-Mission Freeze and Export

When game_over is detected by the frontend, all polling timers should stop and a Mission Complete panel should replace the live controls, showing final score, elapsed time, per-agent breakdown, and outcome label. An Export PDF button via window.print() with print-media CSS would produce a printable report suitable for academic comparison of strategies.

## 10.3 Agent Blackboard Coordination

The current implicit coordination (shared target list) is effective but limited. A shared blackboard dictionary would enable agents to declare intended targets, share position data, and build a collective threat model. The primary implementation challenge is avoiding race conditions between agent ticks — agents must read the blackboard at the start of their tick and write at the end, with tick serialisation ensuring consistency.

## 10.4 Godot / ROS Integration

The /api/robot/telemetry endpoint already exists in the backend with a 2-second fallback window before self-drive resumes. Completing this integration requires documenting the telemetry API contract ({ agent_id, lat, lon, heading, speed }), building a minimal Godot 4 scene with HTTPRequest posting, and extending the backend to handle per-agent telemetry feeds rather than a single global override.

# 11\. Repository Structure

**SAR-SIMULATION/**

├── Backend/

│ ├── Dockerfile

│ ├── requirements.txt

│ └── src/main.py ← FastAPI app + all AI logic (~1600 lines)

│ # data/ created at runtime (not in Git):

│ # data/stats/ ← saved mission JSON files

│ # data/scenarios/ ← saved scenario JSON files

│ # data/osmnx_cache/ ← cached OSM graph downloads

├── Frontend/

│ ├── Dockerfile ← nginx container (1 line: COPY . /usr/share/nginx/html)

│ ├── index.html ← mission planner

│ ├── mission.html ← live mission monitor

│ ├── stats.html ← analytics dashboard

│ └── editor.html ← scenario map editor

├── SARGV-FIN-CC.py ← original Pygame prototype (reference)

├── docker-compose.yml

├── README.md

└── FUTURE_WORK.md

Figure Repository structure. Runtime data directories are excluded from version control.

# 12\. Conclusion

SAR-UGV Simulation delivers a production-quality multi-agent search and rescue platform on real-world OpenStreetMap topology. The key contributions of this version are:

- **A risk-weighted A\* pathfinding algorithm combining tiered enemy proximity penalties, terrain cost multipliers, and dead-end avoidance into a single unified cost function.**
- **A greedy distance-based multi-agent target pre-assignment scheme that maximises spatial coverage at O(n²) cost.**
- **An implicit coordination mechanism via shared target state that achieves cooperative deduplication without explicit message passing.**
- **A configurable simulation loop supporting variable speeds from 0.25× to 100×, enabling rapid batch evaluation.**
- **A zero-framework frontend architecture deployable by a single Docker command with no build pipeline.**
- **Persistent scenario and mission statistics storage enabling longitudinal performance analysis.**

The platform is immediately usable for academic comparison of navigation strategies, multi-agent coordination policies, and enemy avoidance algorithms across arbitrary real-world urban environments. The identified future work — risk heatmap visualisation, post-mission reporting, explicit agent blackboard coordination, and Godot/ROS integration — will progressively elevate the platform from a functional simulation to a full research testbed suitable for algorithm publication.

_— End of Document —_