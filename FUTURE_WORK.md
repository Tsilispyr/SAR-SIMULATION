# SAR-UGV — Future Work & Roadmap

This document outlines the features and improvements that are **not yet implemented** in the current web stack, why they matter, and the approach we plan to take for each.

---

## 1. Multi-Agent Rendering in the Browser

### What is missing
The backend simulation engine (`SARGameState`) supports up to **4 simultaneous agents** with cooperative target de-duplication and per-agent path planning. The frontend currently tracks only a single robot marker. There are no per-agent path polylines, no per-agent HUD columns, and no colour differentiation between agents.

### Why it matters
The original Pygame spec (SARGV-FIN-CC.py) treats multi-agent co-op as a core feature — agents share target assignments and divide search area organically. Running a single agent is a subset of the full design intent.

### Planned approach
1. **Backend** — extend `/api/robot/position` to return an array of agent states (`{ id, lat, lon, path, score, status }`). The existing `SARGameState` already tracks `robot_x/y` per-instance; we need to wrap it in a list of agents or extend the state class.
2. **Frontend** — maintain a map of `agentId → { marker, polyline }` in `mission.html`, each agent coloured distinctly (blue / green / orange / pink mirroring the spec's agent colour list). HUD columns expand horizontally to show per-agent score and status.
3. **Stats** — per-agent charts for score and target contribution over time in `stats.html`.

---

## 2. Risk Heatmap Overlay (Costmap Visualisation)

### What is missing
The Pygame spec renders a **live colour-coded heatmap** below the game field showing the dynamic cost map — green (safe) through yellow (caution) to red (enemy influence). The web frontend computes the cost map on every replan but never exposes it to the browser.

### Why it matters
The heatmap is one of the most valuable analytical outputs — it gives operators an immediate intuition of why the robot chose a particular route and how the enemy threat landscape evolves over time.

### Planned approach
1. **Backend** — add a `/api/sim/costmap` endpoint that returns a list of `{ lat, lon, cost }` samples (one per graph node or grid cell) computed from `build_penalized_graph`.
2. **Frontend** — render as a Leaflet **heatmap layer** using `Leaflet.heat` (plugin, ~15 KB). Colour ramp: blue → yellow → red mapped to normalised cost 0–1. Update every 2–3 seconds (lower priority than position).
3. **Performance** — the OSM graph can have 1000–5000 nodes in a 1.5 km radius. We will downsample to a representative subset (every 3rd node) or use a fixed grid sampled from the graph for manageable payload size.

---

## 3. Browser-Based Map Editor

### What is missing
The Pygame spec includes a full **MapEditor** class allowing manual placement of agents, targets, patrol enemies, aggressive enemies, obstacles, and terrain types on a grid. The web deploy currently only accepts counts and a geographic radius — there is no way to hand-craft a specific scenario.

### Why it matters
Custom scenarios are essential for repeatable testing and demonstration — you can design a specific tactical situation (e.g. a chokepoint with two aggressive enemies guarding the final target) that is impossible to reproduce reliably with random spawns.

### Planned approach
1. Add a new **`editor.html`** page linked from `index.html`.
2. Use the existing Leaflet map as the canvas — clicking the map in different "tool modes" (agent / target / patrol / aggressive / clear) calls a backend endpoint to register the entity at that location.
3. **Backend** — add `/api/editor/place` (POST with `type`, `lat`, `lon`) and `/api/editor/clear` endpoints. These populate a separate "custom scenario" snapshot that overrides the random spawn when `/api/map/vector` is called with `mode=custom`.
4. Persist custom scenarios as JSON files so they can be replayed without re-editing.

---

## 4. Configurable Enemy Detection Radius

### What is missing
The `Enemy.detection_radius` is hardcoded to **50 metres** in the backend. The frontend deploy form exposes `aggr_speed` and `patrol_speed` but has no slider or input for detection range.

### Why it matters
Detection radius is arguably the most impactful single parameter for difficulty tuning — a 30 m radius makes the robot very safe; a 100 m radius makes most of the map a threat zone. Without control over this, scenario design is limited.

### Planned approach
1. **Backend** — add `detection_radius: float = 50.0` as a query parameter to `/api/map/vector`, passed through to `spawn_objects` and stored in `current_map_data`.
2. **Frontend** — add a range slider in `index.html` between the enemy speed sliders, labelled "Enemy Detection Radius (m)" with range 20–150 m and default 50 m.
Quick change — estimated 30 minutes total.

---

## 5. Post-Mission Freeze / Final Report View

### What is missing
In the Pygame spec, a mission end triggers a dedicated **report screen** that freezes the chart at the mission-end timestamp and shows final score, time, and per-agent breakdown before returning to the menu. The web `stats.html` continues polling after the mission ends — charts update but the "mission-end freeze" moment is not clearly delineated.

### Why it matters
Operators reviewing a completed mission need a stable, printable final state — not a live-polling page that appears to keep changing after the mission has ended.

### Planned approach
1. When `poll()` in `stats.html` detects `game_over = true`, stop all further polling and take a **snapshot** of the current chart data.
2. Display a full-width overlay panel (similar to the result banner) with the final score, elapsed time, and a breakdown of each result metric.
3. Add a **"Export Report"** button that calls `window.print()` with a print-media CSS that hides the map and control elements and shows only the final charts and summary table.

---

## 6. Tile Terrain Overlay

### What is missing
The spec's tile system classifies every cell as Free / Difficult / Hazard / Obstacle and renders each with a distinct colour. The web stack applies road-type cost multipliers (`HIGHWAY_MULTIPLIER`) to pathfinding but does **not visualise** these terrain categories on the Leaflet map — every street looks identical regardless of its traversal cost.

### Why it matters
Without terrain visualisation, operators cannot make informed decisions about route preferences. They can see where the robot went, but not *why* it avoided certain streets.

### Planned approach
1. **Backend** — extend `/api/map/vector` response to include per-edge terrain category (`free` / `difficult` / `hazard`) derived from `HIGHWAY_SCORE_PENALTY` lookup.
2. **Frontend** — render as Leaflet polylines with a colour-coded stroke: white = free, amber = difficult, red = hazard. Toggle visibility with a checkbox in the `index.html` sidebar.

---

## 7. Pause-Aware Elapsed Time Accuracy

### What is missing
The simulation's `mission_elapsed` counter increments by `effective_dt` every tick regardless of whether the simulation is in a conceptually "running" state from the operator's perspective. Pausing stops the simulation loop cleanly, but there may be edge cases where the timer accumulates fractional ticks during the pause/resume transition.

### Planned approach
1. Add a `_pause_start_time` field to `SARGameState`.
2. On pause, record `_pause_start_time = time.time()`.
3. On resume, subtract `(time.time() - _pause_start_time)` from the effective elapsed before the next tick.
4. Write a unit test in `test_ai.py` that cycles pause/resume 100× and asserts elapsed drift < 0.01 s.

---

## Priority Order

| Priority | Feature | Effort estimate |
|---|---|---|
| High | Enemy Detection Radius slider | 
| High | Post-Mission Freeze / Report |
| Medium | Pause-aware elapsed time fix |
| Medium | Terrain overlay on map | 
| Large | Risk Heatmap overlay | 
| Large | Multi-agent browser rendering | 
| Major | Browser map editor |
