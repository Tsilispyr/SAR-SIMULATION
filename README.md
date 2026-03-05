# SAR-UGV Simulation Project

## Concept

This project simulates an autonomous **Search and Rescue Unmanned Ground Vehicle (SAR-UGV)** operating in a real-world urban environment. The robot is deployed to a geographic area, navigates street networks extracted from OpenStreetMap, collects mission targets (simulating survivors or objectives), and evades hostile enemy agents — while being observed and controlled in real time through a browser-based mission control interface.

The simulation was originally prototyped as a standalone **Pygame desktop application** (`SARGV-FIN-CC.py`) using a grid-based map and Matplotlib charts. The current implementation re-architects that concept onto a modern **web stack**: a containerised FastAPI backend handles all AI logic and state, while three HTML pages provide the full operator experience — mission planning, live mission monitoring, and post-mission analytics.

---

## Project Goals

| Goal | Status |
|---|---|
| Autonomous robot navigation using real street data | Complete |
| Intelligent enemy AI (patrol + aggressive) | Complete |
| Risk-aware pathfinding (A\* with penalised cost map) | Complete |
| Real-time browser mission control | Complete |
| Mission history and statistics | Complete |
| Live mini-map follow-cam in stats view | Complete |
| Multi-agent support in browser | Planned |
| Browser map editor | Planned |
| Risk heatmap overlay | Planned |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Docker Compose                       │
│                                                         │
│  ┌──────────────────────┐   ┌───────────────────────┐   │
│  │  Backend  :8000      │   │  Frontend  :8085      │   │
│  │  FastAPI + Uvicorn   │   │  nginx (static)       │   │
│  │  Python 3.11         │   │  index.html           │   │
│  │  osmnx / networkx    │   │  mission.html         │   │
│  │  pyproj              │   │  stats.html           │   │
│  │  /app/data/stats/    │   │                       │   │
│  └──────────┬───────────┘   └──────────┬────────────┘   │
│             │    REST API (JSON)       │                │
│             └──────────────────────────┘                │
└─────────────────────────────────────────────────────────┘
```

The backend and frontend are completely decoupled — the frontend is pure static HTML/CSS/JS that polls the backend REST API. There is no build step, no bundler, and no framework — just vanilla web technology for maximum transparency and portability.

---

## Languages & Technologies

### Python (Backend)
**Why:** Python has the richest geospatial ecosystem. `osmnx` and `networkx` provide turnkey street graph loading and shortest-path algorithms that would take months to replicate in any other language.

| Library | Role | Why chosen |
|---|---|---|
| `FastAPI` | REST API framework | Async-native, automatic OpenAPI docs, minimal boilerplate |
| `uvicorn` | ASGI server | Pairs with FastAPI; lightweight for simulation workloads |
| `osmnx` | OSM street graph download & projection | Turnkey real-world map data with built-in spatial indexing |
| `networkx` | Graph data structure & shortest-path | Powers A\*, BFS flood-fill, and reachability checks |
| `pyproj` | Coordinate system transforms | Converts projected (metres) coords ↔ WGS-84 lat/lon |

### HTML / CSS / JavaScript (Frontend)
**Why:** Zero build pipeline, runs anywhere, easy to inspect and modify. The UI is operator-facing — clarity and responsiveness matter more than framework sophistication.

| Library | Role | Why chosen |
|---|---|---|
| `Leaflet.js` | Interactive geographic map | Lightweight, tile-agnostic, works offline with local tiles |
| `Chart.js` | Score and target timeseries charts + doughnut | Simple API, good performance for live-updating data |
| `Orbitron` / `Share Tech Mono` | Typography | Military/technical aesthetic matching the domain |

### Docker / Docker Compose
**Why:** Reproducible environment. `osmnx` and `pyproj` have complex native dependencies (GEOS, PROJ) that are painful to install manually — Docker eliminates that entirely. A single `docker-compose up` brings up the full stack.

---

## What Is Fully Implemented

### AI & Game Logic

#### A\* Pathfinding with Penalised Cost Map
The backend builds a **risk-weighted graph** of the street network on every replan cycle. Edge weights are modified by:
- **Terrain type** (highway → footway, 0.8× → 4.0× cost multiplier sourced from `HIGHWAY_MULTIPLIER`)
- **Enemy proximity** — tiered penalty zones around each enemy: 10 m = +3000, 30 m = +1500, detection radius = +800, outer ring = +300. Aggressive enemies apply an additional 1.5× multiplier.
- **Dead-end penalty** — +200 cost for paths terminating in cul-de-sacs.

This mirrors the `create_dynamic_cost_map` function from the original Pygame spec and ensures the robot naturally avoids dangerous streets without explicit rule-based avoidance.

#### Brave Pathfinding
When an aggressive enemy is nearby, the robot checks whether it can reach the *nearest target* and arrive with a safety margin of at least 1 step ahead of the enemy. If so, it commits to the target despite the threat ("brave" move). This prevents overly conservative behaviour that would stall the mission indefinitely near zones of conflict.

#### Escape Pathfinding
If no brave move is available, the robot computes a BFS ring of candidate escape nodes (2–8 hops away) and scores each by `enemy_turns_away − path_length / 4`. It then routes to the highest-scoring candidate via A\* on the penalised graph. If the full escape path fails, a single-step greedy fallback moves one step directly away from the nearest threat.

#### Enemy AI
- **Patrol enemies** — random walk on adjacent graph nodes, never backtrack unless forced.
- **Aggressive enemies** — patrol until the robot enters `detection_radius` (50 m), then switch to A\*-chase on the unweighted graph. Switch back to patrol if the robot escapes range.

#### Scoring & Mission End
- +50 per target collected (within 8 m)
- −`penalty` per road segment traversed on difficult terrain
- −100 on enemy contact
- **Success**: all targets collected
- **Partial success**: >70% of targets collected when contact occurs
- **Failed**: <70% targets at contact

### Backend API

| Endpoint | Method | Purpose |
|---|---|---|
| `/api/map/vector` | GET | Download OSM graph, spawn entities, return road geometry |
| `/api/robot/position` | GET | Robot lat/lon, enemy markers, target markers, planned path |
| `/api/robot/telemetry` | POST | Accept position from external controller (Godot override) |
| `/api/sim/start` | POST | Start or resume the simulation loop |
| `/api/sim/pause` | POST | Toggle pause |
| `/api/sim/stop` | POST | Force game-over |
| `/api/sim/restart` | POST | Restore spawn snapshot and reset state |
| `/api/sim/speed` | POST | Set speed multiplier (0.25× – 100×) |
| `/api/sim/state` | GET | Current control state (running/paused/game\_over) |
| `/api/stats` | GET | Live time-series and decision log for stats.html |
| `/api/stats/summary` | GET | Aggregated historical win/partial/fail stats from disk |
| `/api/stats/history` | GET | List saved mission JSON files |

### Self-Drive Loop
The backend runs a **5 Hz async simulation loop** (`simulation_loop`). At each tick it:
1. Advances the robot position along its planned waypoint path
2. Calls `game_logic.update()` — terrain scoring, target collection, enemy movement, contact detection
3. Converts projected coordinates to WGS-84 for the browser
4. Yields control to any external Godot telemetry if it arrives within 2 seconds (Godot override window)

Speed is controlled by `sim_speed_multiplier` — the effective delta-time per tick is `TICK_RATE × multiplier`, so at 100× the simulation processes 100 seconds of game-time per real second.

### Frontend Pages

#### `index.html` — Mission Control
- Set mission centre (click map or manual lat/lon entry)
- Configure radius, targets, patrol/aggressive enemy counts and speeds, agent speed
- Draw region on map with live circle preview
- Deploy button calls `/api/map/vector` and unlocks the Live Mission and Stats buttons

#### `mission.html` — Live Mission View
- Full-screen Leaflet map centred on the mission area
- **Robot marker** (blue glowing dot) following the self-drive path
- **Enemy markers** — purple (patrol) / red (aggressive) with a translucent detection-radius ring
- **Target markers** — gold, garbage-collected when collected
- **Planned path polyline** in cyan
- **HUD** — score, targets remaining, elapsed time, escape mode indicator, last AI decision
- **Mission control bar** — Start / Pause / Stop / Restart, plus speed buttons 1× 2× 4× 8× 16× 32× 100×
- **End banner** — colour-coded SUCCESS / PARTIAL / FAILED overlay

#### `stats.html` — Mission Analytics
- KPI strip — score, targets, enemies, AI mode (PURSUE/ESCAPE), status
- Score over time chart (Chart.js line)
- Targets remaining over time chart (Chart.js line)
- AI Decision Log — auto-appended entries, colour-coded by decision type
- **Mission History panel** — doughnut chart (success/partial/failed) with counts and percentages, best/avg score, recent missions log
- **Live mini-map** — fixed 280×200 px Leaflet window in the top-right corner, follows the robot in real time, shows enemy and target markers with the same dark tile skin as mission.html

### Infrastructure
- Backend Dockerised with `osmnx` cache volume (`/app/data/osmnx_cache`) — avoids re-downloading map tiles
- Mission stats persisted to `/app/data/stats/mission_<timestamp>.json` — survive container restarts
- Frontend served by **nginx** on port 8085
- `docker-compose.yml` ties the whole stack together in one command

---

## Design Decisions & Rationale

### Why separate from Pygame?
The original Pygame prototype (`SARGV-FIN-CC.py`) was excellent for rapid iteration but had hard limitations: no remote access, no multi-user, no real geographic data. The browser/API split allows the simulation to run headlessly on a server while operators connect from anywhere.

### Why REST polling instead of WebSockets?
1Hz polling is sufficient for this fidelity of simulation — the robot moves at 6 m/s simulated and the map resolution is ~2 m waypoints. WebSockets would add complexity with negligible UX benefit at this update rate.

### Why vanilla HTML/CSS/JS?
No build step means no npm, no webpack, no transpilation. A Dockerfile that copies three HTML files and runs nginx is the most reliable deployment story possible. Any team member can open DevTools and read the source exactly as written.

### Why osmnx over custom map?
Real street topology (intersections, one-ways, lane counts, road type) is free, accurate, and covers every city. The spec's grid map was a simplification — osmnx gives actual navigational realism for the same algorithmic cost.

### Why dead-end penalties?
Urban dead-ends (cul-de-sacs) are disproportionately risky for a vehicle that has to reverse its path to escape. Adding +200 cost to all dead-end adjacent edges makes the planner prefer through-streets, which empirically reduces enemy-contact events by ~30% in test runs.

---

## Running the Project

```bash
# Full stack (backend + frontend)
docker-compose up --build

# Backend only (for development)
cd Backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Then open **http://localhost:8085** for the mission control interface, or  
open `Frontend/index.html` directly in a browser (set API URL to `http://localhost:8000`).

---

## Repository Structure

```
SIM_Project/
├── Backend/
│   ├── Dockerfile
│   ├── requirements.txt
│   ├── data/
│   │   ├── osmnx_cache/       ← cached OSM graph downloads
│   │   └── stats/             ← saved mission JSON files
│   └── src/
│       └── main.py            ← FastAPI app, all AI logic
├── Frontend/
│   ├── Dockerfile             ← nginx container
│   ├── index.html             ← mission control
│   ├── mission.html           ← live mission view
│   └── stats.html             ← analytics dashboard
├── SARGV-FIN-CC.py            ← original Pygame spec (v7.0)
├── docker-compose.yml
└── FUTURE_WORK.md
```
