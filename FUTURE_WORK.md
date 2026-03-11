# SAR-UGV — Future Work & Remaining Roadmap

This document lists features that are **not yet implemented**, explains why they matter, and proposes concrete approaches based on the existing code architecture.

Everything listed in the original FUTURE_WORK.md as "planned" is now **fully implemented** in the current codebase:

- Multi-agent simulation (up to 4 agents)
- Per-agent browser rendering (coloured markers + path polylines)
- Per-agent HUD tiles and score charts
- Browser-based map editor (`editor.html`)
- Custom scenario save / load / delete
- Configurable enemy detection radius (slider in index.html)
- Scenario loading with map teleport and placement preview
- Self-contained live mission view (deploy new mission without leaving mission.html)
- Mission history grouped by agent count
- Real-time chart updates (every poll cycle)

What remains below is the **next tier** of features that would push this from a functional simulation into a full research or demonstration platform.

---

## 1. Risk Heatmap Overlay

### What is missing
The backend builds a `build_penalized_graph` on every replan that encodes enemy threat, terrain cost, and dead-end risk into edge weights. This data is used for pathfinding but never shown to the operator. The Pygame prototype rendered a colour-coded heatmap directly on the game field.

### Why it matters
Operators viewing the live mission cannot understand *why* an agent chose a particular route. A heatmap overlay instantly communicates the threat landscape — which streets are safe (green), which are in enemy influence zones (yellow/orange), and which are extremely dangerous (red).

### Proposed approach
1. **Backend** — add a `GET /api/sim/costmap` endpoint that runs `build_penalized_graph` (already called each tick anyway) after the fact and samples the weight of each node's cheapest outgoing edge. Returns `[{lat, lon, cost_norm}]` for every graph node (typically 500–3000 nodes for a 1 km radius area).
2. **Frontend** — render with [Leaflet.heat](https://github.com/Leaflet/Leaflet.heat) (a single ~6 KB plugin). Colour ramp: blue (0) → yellow (0.5) → red (1.0) mapped to normalised cost.
3. **Performance** — update the heatmap every 3–5 seconds (much lower priority than position polling). Downsample to every 2nd node if needed; imperceptible on a dense street graph.

### Estimated effort
Medium. The backend computation already exists; this is largely a frontend rendering task.

---

## 2. Post-Mission Freeze / Final Report

### What is missing
When a mission ends (`game_over = true`) the analytics page (`stats.html`) continues polling. Charts keep updating (with the same frozen data) and there is no clear "this is the final state" moment. The Pygame prototype had a dedicated report screen that froze all charts and displayed a structured summary.

### Why it matters
For academic use — comparing strategies, logging performance — you need a stable, printable end-state rather than a live-polling page that visually appears to keep running.

### Proposed approach
1. When `poll()` in `stats.html` detects `game_over === true`, stop all `setInterval` polling timers.
2. Show a full-width **Mission Complete** panel above the charts with: final score, elapsed time, per-agent breakdown, outcome label.
3. Add an **Export PDF / Print** button that calls `window.print()` with print-media CSS hiding the map and only showing charts + summary table.
4. Add a **"Watch Replay"** button stub that clears the freeze and re-enables polling for reviewing telemetry of a completed mission.

### Estimated effort
Small–Medium. Mostly frontend work; no new backend endpoints required.

---

## 3. Terrain Tile Overlay (Road Penalty Visualisation)

### What is missing
Every road on the Leaflet map looks identical regardless of its traversal cost. The cost multipliers in `HIGHWAY_MULTIPLIER` (motorway = 4.0×, footway = 0.8×, residential = 1.5×) influence every routing decision but are invisible to the operator.

### Why it matters
Without terrain visualisation, the operator cannot tell why an agent preferred one street over another that looks equally short. Adding even a simple colour-coded stroke makes the cost model immediately legible.

### Proposed approach
1. **Backend** — extend the `/api/map/vector` GeoJSON response to include a `terrain_class` property per road feature: `"free"` / `"difficult"` / `"hazard"` derived from the `HIGHWAY_SCORE_PENALTY` table.
2. **Frontend** — in `mission.html` and `index.html`, stroke each road polyline accordingly:
   - `"free"` → light white (unchanged from current)
   - `"difficult"` → amber (#f59e0b)
   - `"hazard"` → red (#ef4444)
3. Add a toggle checkbox in `index.html` sidebar ("Show Terrain Cost") to switch the overlay on/off.

### Estimated effort
Small. Most of the data already exists in the backend; this is a GeoJSON property + rendering change.

---

## 4. Pause-Aware Elapsed Time Accuracy

### What is missing
`mission_elapsed` increments by `effective_dt` every tick. The loop stops cleanly on pause, but there may be sub-tick drift during the pause/resume handoff — the elapsed time could gain a few milliseconds on every pause cycle.

### Why it matters
Non-critical for casual use, but matters if elapsed time is used as a benchmark metric for comparing strategies across runs. A timer that drifts by 50 ms per pause cycle is not reproducible.

### Proposed approach
1. Add `_pause_wall_start: float` to `SARGameState`.
2. On pause: record `_pause_wall_start = time.monotonic()`.
3. On resume: subtract `(time.monotonic() - _pause_wall_start) * sim_speed_multiplier` from `mission_elapsed` before the next tick.
4. Unit test: pause/resume 100 times, assert elapsed drift < 10 ms.

### Estimated effort
Tiny. Two lines of backend code + a unit test.

---

## 5. Agent Communication & Blackboard

### What is missing
Agents currently communicate implicitly — a target collected by one agent disappears from the shared `game_logic.targets` list, so others will replan away from it. There is no explicit coordination mechanism: no "I am heading to target 3, please take target 7" message passing, no spatial awareness of teammate positions.

### Why it matters
This is the core algorithmic gap between the current greedy pre-assignment and a true multi-agent cooperative system. With a blackboard, agents could:
- Avoid routing through areas another agent already covers
- Hand off targets if one agent is in danger
- Build a shared threat model updated by all agents' observations

### Proposed approach
1. Add a `blackboard: dict` to `SARGameState` — a shared key-value store all agents can read/write each tick.
2. Agents write their **intended next target** and **current position** to the blackboard at the start of each tick.
3. During target selection, an agent reads blackboard entries and adds a soft penalty to targets already claimed by teammates.
4. Add `/api/sim/blackboard` GET endpoint for the frontend to visualise coordination (optional).

### Estimated effort
Large. Requires careful design to avoid race conditions between agent ticks.

---

## 6. Unit & Integration Tests

### What is missing
There is no automated test suite. All verification has been manual (browser + log inspection). A project of this complexity needs regression tests — especially for pathfinding edge cases, multi-agent state management, and scenario save/load round-trips.

### Why it matters
Every change to `main.py` risks silently breaking pathfinding, score calculation, or restart logic. A test suite catches regressions in seconds rather than minutes of manual testing.

### Proposed approach
Create `Backend/tests/` with:
- `test_pathfinding.py` — unit tests for `build_penalized_graph`, `astar_path`, escape candidate scoring
- `test_game_state.py` — test spawn, target collection, scoring, multi-agent dedup, game_over conditions
- `test_api.py` — FastAPI `TestClient` integration tests for all REST endpoints
- `test_scenario.py` — save → load → deploy round-trip with assertion on placement count and parameters

Use `pytest` + `pytest-asyncio`. Add to CI (GitHub Actions) on push to main.

### Estimated effort
Large (first-time setup), then ongoing maintenance is small per new feature.

---

## 7. Godot / ROS Integration (External Controller Override)

### What is missing
The backend has a `/api/robot/telemetry` POST endpoint that accepts external position overrides (originally designed for a Godot 3D client). This is partially implemented but untested end-to-end. The "Godot override window" logic in `simulation_loop` waits 2 seconds for external telemetry before falling back to self-drive.

### Why it matters
Connecting a physics simulator (Godot) or a real ROS-based robot would elevate this from a waypoint simulation to a high-fidelity platform testbed — agents move with realistic dynamics rather than teleporting between graph nodes.

### Proposed approach
1. Document the telemetry API contract (`{ agent_id, lat, lon, heading, speed }`).
2. Build a minimal Godot 4 scene with a `HTTPRequest` node that posts agent position every 100 ms.
3. Extend the backend to accept per-agent telemetry (currently assumes single agent).
4. Add a `mode=telemetry` flag to `SARGameState` that disables self-drive for agents receiving external position feed.

### Estimated effort
Large. Requires Godot scene + backend protocol extension.

---

## Priority Summary

| Priority  |           Feature                 |    Effort     |
|-----------|-----------------------------------|---------------|
|    High   | Post-mission freeze / report view | Small–Medium  |
|    High   | Terrain tile overlay              | Small         |
|    High   | Unit & integration tests          | Medium–Large  |
|   Medium  | Risk heatmap overlay              | Medium        |
|   Medium  | Pause-aware timer fix             | Tiny          |
|    Low    | Agent blackboard / coordination   | Large         |
|  Research | Godot / ROS integration           | Large         |
