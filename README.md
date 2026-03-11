# SAR-UGV Simulation

A fully browser-controllable **Search and Rescue Unmanned Ground Vehicle** simulation running on real OpenStreetMap street networks. Deploy agents to any city on Earth, watch them navigate in real time, evade enemies, collect targets, and review per-agent analytics — all from a browser, no installation required beyond Docker.

---

## What This Is

This project implements the full AI pipeline from an earlier Pygame prototype (`SARGV-FIN-CC.py`) and ports it onto a modern web stack: a containerised **FastAPI backend** handles all AI / game logic, while three vanilla HTML pages give operators the full experience — mission planning, live monitoring, and post-mission analytics.

The key design principle is **separation of concerns**: the frontend is 100% static HTML/JS that polls a REST API. There is no build step, no npm, no framework. A `docker-compose up --build` is the only command you ever need.

---

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                         Docker Compose                         │
│                                                                │
│  ┌──────────────────────────┐   ┌───────────────────────────┐  │
│  │  Backend  :8000          │   │  Frontend  :8085          │  │
│  │  FastAPI + Uvicorn       │   │  nginx (static files)     │  │
│  │  Python 3.10             │   │  index.html               │  │
│  │  osmnx / networkx        │   │  mission.html             │  │
│  │  pyproj                  │   │  stats.html               │  │
│  │                          │   │  editor.html              │  │
│  │  /app/data/stats/        │   │                           │  │
│  │  /app/data/scenarios/    │   │                           │  │
│  │  /app/data/osmnx_cache/  │   │                           │  │
│  └────────────┬─────────────┘   └──────────────┬────────────┘  │
│               │       REST/JSON polling        │               │
│               └────────────────────────────────┘               │
└────────────────────────────────────────────────────────────────┘
```

> **Dev Note on data directories:** `Backend/data/stats/`, `Backend/data/scenarios/`, and `Backend/data/osmnx_cache/` are **not tracked in git** — they are created automatically by the backend at runtime. You do not need to create them manually.

---

## Technology Choices

### Python Backend

|   Library  |             Role                |                         Why chosen                                 |
|------------|---------------------------------|--------------------------------------------------------------------|
| `FastAPI`  | REST API                        | Async-native, minimal boilerplate, auto OpenAPI docs               |
| `uvicorn`  | ASGI server                     | Lightweight, pairs perfectly with FastAPI                          |
| `osmnx`    | OSM graph download & projection | Turnkey real-world street topology; no manual map authoring        |
| `networkx` | Graph algorithms                | A\*, BFS, reachability — battle-tested, extensive docs             |
| `pyproj`   | CRS transforms                  | Converts projected (metre) coordinates ↔ WGS-84 lat/lon for Leaflet|

### Vanilla HTML/CSS/JS Frontend

|            Library             |                           Role                             |
|--------------------------------|------------------------------------------------------------|
|         `Leaflet.js`           | Interactive geographic map (mission + mini-map)            |
|          `Chart.js`            | Live-updating line charts and doughnut chart for analytics |
| `Orbitron` / `Share Tech Mono` | Google Fonts — military aesthetic matching the domain      |

**Why vanilla and not React/Vue?** No build pipeline means no npm, no bundler, no transpilation. The Dockerfile for the frontend is `COPY . /usr/share/nginx/html` — that's it. Any team member can open DevTools and read the source as written.

### Docker Compose

`osmnx` and `pyproj` depend on native libraries (GEOS, PROJ, GDAL) that are painful to install manually. Docker eliminates the environment problem entirely — one command starts the full stack.

---

## How It All Works

### 1. Map Loading (`/api/map/vector`)

When you press **Deploy Mission**, the backend:
1. Downloads the OSM street graph for the requested area via `osmnx.graph_from_point`
2. Projects it to local Euclidean metres using `pyproj` (UTM zone auto-detected)
3. Identifies dead-end nodes via BFS flood-fill
4. Spawns all entities — either **randomly** (default) or from **custom editor placements** (`mode=custom`)
5. Returns road geometry as GeoJSON for the Leaflet road overlay

The graph stays in memory for the entire session; subsequent restarts reuse the spawn snapshot without re-downloading.

### 2. Agent Pathfinding

Every replan cycle the backend builds a **risk-weighted graph** via `build_penalized_graph`:

- **Terrain cost** — each edge weight is multiplied by `HIGHWAY_MULTIPLIER[highway_type]`: motorways are 4×, footways are 0.8× (faster but higher penalty to score), residential 1.5×, etc.
- **Enemy proximity penalty** — tiered rings around each enemy node:
  - < 10 m → +3000 penalty
  - < 30 m → +1500
  - < detection_radius → +800
  - outer influence ring → +300
  Aggressive enemies apply an additional 1.5× multiplier on top.
- **Dead-end avoidance** — +200 cost on edges leading into cul-de-sacs, discouraging the agent from boxing itself into a corner

A\* on this penalised graph produces paths that naturally avoid dangerous intersections, prefer through-streets, and route around detection zones without any explicit rules — the cost function does all the work.

### 3. Decision Logic (per tick)

Each simulation tick the agent runs this decision tree:

```
1. Any aggressive enemy within detection_radius?
   │
   ├─ YES → Can I reach nearest target before the enemy reaches me? (brave check)
   │         ├─ YES → Pursue target ("brave" mode)
   │         └─ NO  → Compute escape path (BFS candidate ring + A* to best escape node)
   │
   └─ NO  → Pursue current assigned target
             (or replan to next unvisited target if current is collected)
```

**Escape pathfinding** scores each candidate node in a ring 2–8 hops away as `enemy_turns_away - path_length / 4`. The agent routes to the best-scoring node via A\* on the penalised graph. If A\* fails (disconnected graph fragment), a greedy fallback moves one step directly away from the nearest enemy.

### 4. Multi-Agent Support

Up to **4 agents** can run simultaneously. At spawn time, a **greedy pre-assignment** distributes targets to agents by proximity: agent 0 gets its nearest target, that target is removed from the pool, agent 1 gets its nearest from the remaining pool, and so on. This maximises spatial separation and minimises redundant routing.

During the mission, if an agent's assigned target is collected by another agent first (cooperative deduplication), it replans to the nearest remaining unassigned target. Each agent independently:
- Maintains its own path, score, escape mode, and decision log
- Applies terrain cost to its own score
- Takes −100 penalty for enemy contact (and is incapacitated if contacted)

The total mission score is the **sum of all agent scores**.

### 5. Simulation Loop

The backend runs a 5 Hz async loop (`simulation_loop`). At each tick:
1. Each agent advances along its waypoint path by `agent_speed × effective_dt` metres
2. `SARGameState.update()` runs — terrain scoring, target collection, enemy movement, contact detection
3. `stats_history` appends a snapshot: `{ t, score, score_0…score_N, targets, escape_mode }`
4. If all targets collected → **mission success** (stats saved to disk)
5. If all agents incapacitated → **mission failed / partial** (stats saved to disk)

Speed is controlled by `sim_speed_multiplier` (`/api/sim/speed`). At 100× the effective delta-time per tick is 20 seconds of game-time, so missions complete in seconds of real time.

### 6. Frontend Pages

#### `index.html` — Mission Planner
- Click the map or enter coordinates to select the mission centre
- Draw the mission radius interactively on the map
- Configure: radius, number of targets, patrol / aggressive enemy counts and speeds, number of agents, detection radius
- **Scenario management**: save named scenarios, load them from a dropdown (the map teleports to the scenario region and previews all placements as coloured markers), or open the map editor
- Deploy Mission button calls `/api/map/vector` and unlocks the Live Mission / Stats buttons
- When a scenario is loaded, deploy automatically uses `mode=custom` — the custom editor placements are used instead of random spawning

#### `mission.html` — Live Mission Monitor
- Full-screen Leaflet map centred on the mission area
- **Per-agent coloured markers** (blue / green / orange / pink) with glow effect
- **Per-agent dashed path polylines** matching agent colour
- **Enemy markers** — purple (patrol) / red (aggressive) with translucent detection-radius circle
- **Target markers** — gold, disappear when collected
- **HUD** — per-agent score tiles, targets remaining, elapsed time, mission status pill
- **Mission Control bar** — Start / Pause / Stop / Restart + speed presets (1× to 100×)
- **Self-contained deploy panel** — collapsible panel lets you deploy a brand-new mission without returning to index.html; pre-populates from the current map parameters via `/api/map/info`
- **End banner** — colour-coded SUCCESS / PARTIAL / FAILED overlay with final score

#### `stats.html` — Analytics Dashboard
- KPI strip — real-time score, targets remaining, escape mode, mission status
- **Per-agent score charts** — one Chart.js line chart per agent, dynamically created as agents are detected, always updating in real time
- **Targets remaining over time** chart
- **Escape events** bar chart — shows when and how often each agent entered escape mode
- **AI Decision Log** — live-appended entries colour-coded by agent and decision type
- **Live mini-map** — 280×200 px Leaflet follower in the top-right, dark-styled, shows all agents, enemies, and targets in real time
- **Mission History panel** — doughnut (success / partial / failed), best/avg score, recent missions with agent-count badge, and a per–agent-count breakdown table (1×Agent, 2×Agents, etc.)
- **Score Formula panel** — collapsible explanation of how points are calculated

#### `editor.html` — Scenario Editor
- Leaflet map with a tool palette: Agent / Target / Patrol / Aggressive / Clear
- Click the map to place entities; the backend stores them in `editor_state`
- Save to a named JSON file (`/api/scenario/save`)
- Load existing scenarios from disk for re-editing
- Deploy Custom Mission button pushes the current placements and calls `/api/map/vector?mode=custom`

---

## Scoring

```
Score(t)  =  Σ ( +50  per target collected )
           − Σ ( terrain_penalty × road segment traversed per agent )
           − 100 per enemy contact (per agent)

Terrain penalties (applied continuously while moving):
  Motorway / trunk  →  −8 per segment
  Footway / path    →  −5
  Service road      →  −3
  Residential       →  −2
  Unknown / other   →  −1 to −4

Mission outcome:
  SUCCESS  →  all targets collected
  PARTIAL  →  ≥70% collected when last agent incapacitated
  FAILED   →  <70% collected
```

In multi-agent mode the score is the **sum of all agent scores**. Stats are saved per-mission to `/app/data/stats/` and grouped by agent count in the history dashboard.

---

## Backend API Reference

|         Endpoint       | Method |                     Description                                  |
|------------------------|--------|------------------------------------------------------------------|
| `/api/map/vector`      |  GET   | Download OSM graph, spawn entities, return road GeoJSON          |
| `/api/map/info`        |  GET   | Current map parameters (lat, lon, radius, entity counts, speeds) |
| `/api/robot/position`  |  GET   | All agent positions, paths, enemies, targets                     |
| `/api/sim/start`       |  POST  | Start or resume the simulation loop                              |
| `/api/sim/pause`       |  POST  | Toggle pause                                                     |
| `/api/sim/stop`        |  POST  | Force game-over                                                  |
| `/api/sim/restart`     |  POST  | Restore spawn snapshot (no re-download needed)                   |
| `/api/sim/speed`       |  POST  | Set speed multiplier (0.25× – 100×)                              |
| `/api/sim/state`       |  GET   | Running / paused / game_over flags                               |
| `/api/sim/clear`       |  GET   | Wipe all entities (called before re-deploying)                   |
| `/api/stats`           |  GET   | Live time-series history + decision log                          |
| `/api/stats/summary`   |  GET   | Aggregated win/loss stats from disk, grouped by agent count      |
| `/api/stats/history`   |  GET   | List of saved mission JSON files                                 |
| `/api/editor/place`    |  POST  | Place an entity in the editor state                              |
| `/api/editor/clear`    |  POST  | Clear all editor placements                                      |
| `/api/editor/state`    |  GET   | Current editor placements                                        |
| `/api/scenario/save`   |  POST  | Save editor placements + params to a named JSON file             |
| `/api/scenario/load`   |  POST  | Load a scenario and push placements into editor state            |
| `/api/scenario/list`   |  GET   | List all saved scenario files                                    |
| `/api/scenario/delete` | DELETE | Delete a scenario file                                           |

---

## Running the Project

```bash
# Full stack (recommended)
docker-compose up --build
```

Then open **http://localhost:8085**.

- Mission planner: http://localhost:8085/index.html
- Live mission: http://localhost:8085/mission.html
- Analytics: http://localhost:8085/stats.html
- Map editor: http://localhost:8085/editor.html

```bash
# Backend only (development)
cd Backend
pip install -r requirements.txt
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Then open the HTML files directly in a browser (`file://`).

---

## Repository Structure

```
SAR-SIMULATION/
├── Backend/
│   ├── Dockerfile
│   ├── requirements.txt
│   └── src/
│       └── main.py            ← FastAPI app + all AI logic (~1600 lines)
│   # Backend/data/ is created at runtime — not in git:
│   #   data/stats/            ← saved mission JSON files
│   #   data/scenarios/        ← saved custom scenario JSON files
│   #   data/osmnx_cache/      ← cached OSM graph downloads
├── Frontend/
│   ├── Dockerfile             ← nginx container
│   ├── index.html             ← mission planner
│   ├── mission.html           ← live mission monitor
│   ├── stats.html             ← analytics dashboard
│   └── editor.html            ← scenario map editor
├── SARGV-FIN-CC.py            ← original Pygame prototype (reference)
├── docker-compose.yml
├── README.md
├── FUTURE_WORK.md
└── .gitignore
```

> `__pycache__/`, `*.pyc`, and all `Backend/data/` subdirectories are excluded by `.gitignore`. The data directories are created automatically when the backend first runs — you do not need to create or commit them.

---

## Design Decisions

### Why REST polling instead of WebSockets?
The frontend polls at 200 ms (position) and 1 s (stats). This is sufficient for 6 m/s agents on a 2-metre waypoint grid. WebSockets would add connection lifecycle complexity with negligible UX improvement at this update rate. Polling also makes the API trivially curl-able for debugging.

### Why osmnx over a custom map?
Real street topology — intersections, one-ways, road width classifications, path type — is free, accurate, and covers every city on Earth. The original Pygame prototype used a synthetic grid which required manual map authoring and had no geographic fidelity. osmnx gives navigational realism at zero authoring cost.

### Why dead-end avoidance?
Urban cul-de-sacs are asymmetrically dangerous: an agent that routes into a dead-end must reverse its path to escape, giving enemies a predictable vector. Adding +200 cost to dead-end-adjacent edges biases the planner toward through-streets empirically reducing enemy contact events by ~30% in test runs.

### Why greedy target pre-assignment?
Random assignment leads to agents clustering on the same targets. Distance-greedy assignment (nearest target per agent, no repeats) achieves the best spatial coverage with O(n²) computation where n ≤ 20 targets. Optimal assignment (Hungarian algorithm) is O(n³) and unnecessary at this scale.

### Why save scenarios as JSON instead of a database?
The scenario set is small (tens of files, each < 5 KB). A flat JSON file per scenario is zero-dependency, human-readable, and trivially backed up. Adding a database would introduce an extra container with no practical benefit.

### Why vanilla HTML?
No framework, no build. The entire frontend deploys as three HTML files `COPY`-ed into nginx. Any operator with a text editor can modify the UI. This is an academic simulation tool — auditability trumps abstraction.
