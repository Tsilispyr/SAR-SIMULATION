# SAR-UGV: Future Work and Roadmap

_Updated October 2026, after a full review of `Backend/`, `Frontend/`, `evaluation/` and `screenshots/`._

This document lists what is **not yet done**, ordered by priority, with a concrete fix and an effort estimate for each item. Issue IDs (K = backend, F = frontend, E = evaluation, D = documentation) match §13 of `SAR_UGV_Journal.md`, which also explains the system each fix touches.

Effort scale:

| Effort | Time |
| --- | --- |
| Tiny | under 1 h |
| Small | up to half a day |
| Medium | 1–3 days |
| Large | more than 3 days |

---

## Status at a Glance

| Feature | Status | Notes |
| --- | --- | --- |
| Multi-agent missions (1–4 agents) | Done | Enemies step once per agent per tick (K2) |
| Map editor, scenario save/load | Done | Editor deploy ignores agent count and speeds (F8) |
| Geofence, waypoint missions, manual spawn | Done | – |
| MapLibre 3-D live monitor, camera/LiDAR sensors, robot dashboard | Done | – |
| Post-mission freeze, report, print/export, replay (v12 "High" item) | Done in `mission.html` | – |
| Pause-aware elapsed time (v12 "Medium" item) | **Closed, not needed** | Elapsed time is accumulated per executed tick, so pauses cannot drift it |
| UAV-cooperative layer, four strategies, temporal-decay model | Done (committed October 2026) | No UI selector (F3); see K3, K4, K8 |
| 300-mission strategy study, MPI, statistics, figures | Done (committed October 2026) | V1 baseline must be re-run (E1); unpaired design (E2) |
| Risk heat map overlay | Backend endpoint only | Endpoint measures the wrong quantity (K10); no rendering |
| Terrain cost overlay | Not started | – |
| Agent blackboard coordination | Partial | The UAV intelligence store is a one-way blackboard; no UGV↔UGV coordination |
| Unit and integration tests, CI | Not started | – |
| Godot / ROS external controller | Endpoint exists but is **broken** | K7 |

---

## P0: Make the Published Study Reproducible

The IARIA AIVTS 2026 paper reports the strategy study. Its code, harness, results and figures are committed, but the review found issues that affect its comparisons.

| ID | Task | Detail | Effort |
| --- | --- | --- | --- |
| P0.1 | ~~Commit the code and data that produced the paper~~ | **Done (October 2026).** `Backend/src/main.py`, `pathfinding.py`, `Frontend/mission.html`, the `evaluation/` scripts, `batch_results_*` files and figures are committed. The batch logs stay local (`*.log` is git-ignored) | – |
| P0.2 | Pin dependency versions (K15) | `pip freeze` inside the backend image into `requirements.txt`. OSMnx 1.x → 2.x changed APIs (`features_from_point`, `nearest_nodes`) | Tiny |
| P0.3 | Re-run the V1 baseline at 4× (E1) | `python evaluation/batch_test.py --runs 30 --uav-strategy none`. The current V1 `none` file ran at 8× (effective 6.6×, max wall time 332 s under the old 360 s limit); the other nine cells ran at 4×. About 2 h | Small |
| P0.4 | Make the decay model configurable and run the sensitivity sweep | Add `tau`, `w_base`, `drift_rate` and `r_min` to `/api/map/vector`, store them on `SARGameState`, pass them to `build_penalized_graph`, then sweep τ ±50 % and W_base ±25 %. **The repository currently has no code path, parameters or results for such a sweep.** Any statement about decay-parameter sensitivity needs this run first | Medium |
| P0.5 | Seeded, paired design (E2) | Add `--seed` to the harness and a `seed` query parameter that seeds spawning and enemy RNG. Run every strategy on the same seed list and analyse pairwise (McNemar for success, Wilcoxon signed-rank for MPI). Unpaired, a 10 pp effect needs 160–300 runs per arm; pairing cuts that substantially. `evaluation/headless_probe.py` already shows the pattern | Medium |
| P0.6 | Intermediate densities | Add profiles between V1 (≈ 1.5 km⁻²) and V2 (≈ 3.9 km⁻²), e.g. 2.0 / 2.5 / 3.0 / 3.5, to test whether a crossover exists at all before estimating where it lies | Medium (mostly runtime) |
| P0.7 | Censored timeouts (E3) | Record timeouts as their own outcome. Read the real `elapsed` at timeout instead of assuming 2,880 s, which the V1 harness records as 720 × 8 = 5,760 s. Budget in simulated seconds, not wall seconds | Small |

---

## P1: Backend Correctness

| ID | Problem | Fix | Effort |
| --- | --- | --- | --- |
| **K1** | `ox.project_graph` (`main.py:1602`) turns the undirected graph back into a `MultiDiGraph` with one arbitrary direction per street. On the Athens graph, 0 of 400 random node pairs are reachable in it and 20 % of nodes have no successors. The MANEUVER check (`main.py:970`) therefore almost always fails, so threatened agents escape instead. The terrain-penalty lookup (`main.py:1170`) misses 27–42 % of moves. The escape BFS (`main.py:905`) expands successors only | Project first, then convert: `G_proj = ox.project_graph(G_directed).to_undirected()`, then take the largest component. Add a regression test that asserts `not G_proj.is_directed()` | Small |
| **K2** | Enemies move once per **active agent** per tick (`main.py:1241-1243`). With N agents they are N× faster and flip between chase and patrol depending on which agent is processed | Move each enemy once per tick toward the nearest active agent, then test contact against every agent | Small |
| **K3** | `/api/sim/restart` keeps the UAV's position, waypoints and intelligence and the tick counter; the snapshot lacks `uav_strategy`. `/api/sim/reset` and the no-snapshot path drop the UAV entirely | Store `uav_strategy` and `mission_radius` in `_spawn_snapshot`; on restart rebuild `UAVAgent`, clear `uav_intelligence` and set `current_tick = 0` | Tiny |
| **K4** | τ = 30, prune = 90, drift = 3 m/tick, `REPLAN_INTERVAL = 12`, hybrid 150/45 and LiDAR every 5 ticks are all counted in ticks. One tick is 0.2 s × multiplier, so τ is 6 s at 1× but 24 s at 4×, and results do not transfer between speeds | Express each in simulated seconds (compare against `mission_elapsed`) and derive the per-tick values from `dt` | Small |
| **K5** | Contact is a single 5 m distance test per tick (`main.py:1251`), so fast passes tunnel; the UI offers 8–100×, and the backend silently caps at 50× | Swept test (closest approach of the two segments travelled during the tick) or sub-stepping above 4×; show the effective cap in the UI | Small |
| **K6** | Failed missions are never saved (`main.py:1270-1271`), so `stats.html` history can never show a failure | Save every terminal outcome with its status; keep manual stops excluded but labelled | Tiny |
| **K7** | `POST /api/robot/telemetry` calls `update()` with a stale five-argument signature and reads `game_logic.robot_auto_path` (removed), so every call raises. It also sets `last_telemetry_time`, pausing self-drive for 2 s | Rewrite for per-agent telemetry (`{agent_id, x, y, heading}`), or remove the endpoint until the Godot/ROS bridge (P4.5) is built | Small |
| K8 | Target-informed UAV uses `min(agents, key=lambda a: not a.active or True)` (`main.py:538`). The key is constant, so it always uses agent 0 even when that agent is incapacitated | `next((a for a in agents if a.active), None)` | Tiny |
| K9 | Intelligence ages mix `UAVAgent._tick` (`tick_seen`) and `SARGameState.current_tick` (decay). They are in sync today, but recreating the UAV would desynchronise them | Pass `game_state.current_tick` into the detections | Tiny |
| K10 | `/api/sim/costmap` sums total edge cost (length + penalty) per node, so it mostly shows degree × length. It also uses ground-truth enemies and ignores UAV intelligence and the geofence. `/api/map/current` returns an `HTTPException` instead of raising it | Compute penalty only (`penalised − base`) from the same inputs the planner uses; `raise HTTPException` | Small |
| K11 | GET endpoints read game state without `_sim_lock`. `/api/sim/uav` iterates `uav_intelligence` while the loop may be mutating it | Snapshot under the lock (or copy the dict) before serialising | Tiny |
| K12 | Proximity and UAV queries search edge **midpoints** only. An edge longer than twice the query radius escapes the penalty even when the enemy stands on its endpoint (0.2–0.4 % of Athens edges; 1.5 % for fresh UAV sightings at r_min = 60 m) | Query the tree with radius + half the edge length, or index densified edge points | Small |
| K13 | `/api/scenario/load` and `/api/scenario/delete` build a path from an unsanitised name (`../`), and CORS allows any origin | Reuse the save endpoint's sanitiser and reject names that resolve outside `SCENARIOS_DIR` | Tiny |
| K14 | `docker-compose.yml` uses `./backend` and `./frontend`, but the folders are `Backend` and `Frontend`, so the build fails on case-sensitive Linux file systems | Match the folder case | Tiny |
| K15 | Unpinned requirements | See P0.2 | Tiny |

---

## P2: Frontend and UAV in the UI

| ID | Problem | Fix | Effort |
| --- | --- | --- | --- |
| **F1** | `mission.html` sets `uavRing = true` and later calls `uavRing.remove()` (lines 1418, 1451). After a UAV mission is followed by a non-UAV deploy, the call throws every 400 ms, and the ring and intel markers stay on the map | Keep a boolean for the layer; on teardown clear the `uav-ring` source data and remove the intel markers before anything that can throw | Tiny |
| F2 | "Watch Replay" re-enables `poll` and `pollStats` but not `pollUAV` (lines 1077-1078) | Push `setInterval(pollUAV, 400)` too | Tiny |
| **F3** | No page can enable the UAV. `index.html` and the `mission.html` deploy panel omit `uav_strategy`, `/api/map/info` does not report it, the legend has no UAV entries, and the yellow UAV marker resembles the gold targets (see `screenshots/aerial-and-ground-vehocles-together.png`, whose filename is also misspelled) | Strategy `<select>` in both deploy forms; return `uav_strategy` from `/api/map/info`; legend rows for UAV, detection ring and intel; distinct marker colour or shape | Small |
| F4 | Speed buttons go to 100×, but the backend caps at 50× and contact detection is only reliable up to about 4–5× | Cap the buttons at 50×, label anything above 4× as unreliable for evaluation, and read back the effective multiplier | Tiny |
| F5 | `index.html:556` uses `document.querySelector('button')`, which is the "Draw Region" button, so after a deploy that button reads "Deploy Mission" | Give the Deploy button an id and use it | Tiny |
| F6 | `index.html:355`: `<select id="scenario-select"` is missing its closing `>` | Close the tag | Tiny |
| F7 | `stats.html`: the per-agent-count table writes into a missing `#hs-by-agents` element, so it never renders. The score-formula panel lists −8/−5/−3/−2 penalties and a 1.2× threshold; the code uses 0/1/3 and 1.5× | Add the container; update the text from the code constants | Tiny |
| F8 | `editor.html` deploy hard-codes `n_agents=1` and default speeds, silently dropping extra agent placements. Every click moves the mission centre to the clicked point | Derive `n_agents` from the placements; add speed and count inputs; set the centre explicitly | Small |
| F9 | `dashboard.html` fetches the full `/api/sim/scene` payload every 200 ms. Meshes of departed enemies and agents are never removed, and replaced geometries are not disposed | Re-fetch only when the map changes (poll `/api/map/info`); prune and dispose meshes | Small |
| F10 | Every page hard-codes `http://127.0.0.1:8000` or `http://localhost:8000` (mixed) | One `API` constant per page, ideally served through an nginx `/api` reverse proxy so the UI works from other hosts | Small |
| F11 | Turf is loaded from unpkg without a version pin | Pin a version | Tiny |

---

## P3: Evaluation Methodology

| ID | Problem | Fix | Effort |
| --- | --- | --- | --- |
| E1 | V1 baseline generated at 8× | P0.3 | Small |
| E2 | Unseeded, unpaired runs | P0.5 | Medium |
| E3 | Timeouts scored as failures with an assumed time | P0.7 | Small |
| **E4** | MPI's safety and efficiency terms ignore the outcome. A failure after 27 s with 0/6 targets scores 0.50, more than the worst partial success (0.36). `escape_events` counts escape _decisions_, not contacts, and E_max = 5 is exceeded in 34 of 300 runs | Condition efficiency on success (0 otherwise); measure safety by contacts or minimum separation; normalise escapes by mission time; report MPI next to the raw outcome rates, not instead of them | Small |
| E5 | `batch_test.py --clear-old` deletes from `evaluation/Backend/data/stats`, which does not exist | Use `../Backend/data/stats` | Tiny |
| E6 | The "no-UAV" baseline is not sensor-only: aggressive enemies within 1.1 × detection radius, the 15 m danger override and the path lookahead all use ground truth | Add a `perception=sensor_only` mode that gates all three on camera or LiDAR detections, and report both baselines | Medium |
| E7 | UAV detection is perfect (no occlusion, noise, false positives or latency) and the link never drops | Detection probability vs. range, position noise, a latency queue and a drop-out schedule; the decay model already degrades gracefully when updates stop | Medium |

`evaluation/reanalysis.py` reproduces every statistic quoted in the journal. `evaluation/headless_probe.py` runs paired missions without Docker for quick checks.

---

## P4: Features (carried over from v12, updated)

### P4.1 Risk Heat Map Overlay (Medium)

`/api/sim/costmap` exists but is unused and measures the wrong quantity (K10). Fix it, include UAV decay penalties, and render it in `mission.html` as a MapLibre `heatmap` layer refreshed every 3–5 s. The v12 plan's Leaflet.heat no longer applies because the monitor is MapLibre. A toggle should sit next to "Sensor FOV".

### P4.2 Terrain Cost Overlay (Small)

Add a `terrain_class` property (free, difficult or hazard, from `HIGHWAY_SCORE_PENALTY`) to the road features and colour roads by class in `mission.html` and `index.html`, with an on/off toggle.

### P4.3 Unit and Integration Tests with CI (Medium, then Small per feature)

`Backend/tests/` with pytest:

- `build_penalized_graph`: penalty tiers, geofence, UAV decay values at Δt = 0, τ and 3τ;
- a K1 regression test (projected graph undirected);
- K2: one enemy step per tick with two agents;
- K3: restart resets the UAV;
- a FastAPI `TestClient` smoke test of every route, using a small synthetic graph so that no network is needed;
- scenario save → load → deploy round trip.

Run in GitHub Actions on push.

### P4.4 Agent Blackboard Coordination (Large)

`uav_intelligence` already demonstrates a shared store. Extend the idea to UGV↔UGV:

- declared target claims with a soft penalty for claimed targets;
- shared camera detections feeding everyone's _known enemies_;
- hand-off when an agent enters escape mode.

Read at the start of the tick and write at the end to avoid order effects.

### P4.5 Godot / ROS External Controller (Large)

First fix K7. Then define a per-agent telemetry contract (`{agent_id, lat, lon, heading, speed}`), add a `mode=telemetry` per agent that disables self-drive, and build a minimal Godot 4 scene or ROS 2 node that posts at 10 Hz.

---

## P5: Research Directions

- **Exploration-aware threat-informed coverage.** The current strategy only revisits enemies it already knows, so it detected 3–5 of 8 enemies in the paired probe against 7–8 for the lawnmower. Score candidate cells by staleness × prior enemy likelihood so the UAV also searches.
- **Coverage timing matched to the decay constant.** A full lawnmower sweep takes about 16 min at R = 1 km, while sightings expire after 72 s at 4×. Either sweep smaller sectors around the UGV's corridor or adapt τ to the revisit interval.
- **Adaptive τ** from locally estimated enemy speed, as the paper proposes.
- **Multi-UAV coverage** and allocation between UAVs.
- **Energy-constrained UAV** (15–25 min endurance, return-to-base, battery swaps), a known gap acknowledged in the paper.
- **Learned strategy switching** (reinforcement learning over density, UGV state and remaining targets) to replace the fixed hybrid duty cycle.
- **UAV-assisted target search**, where targets start unknown and the UAV detects them as well as threats.
- **Other cities and topologies.** Every result so far comes from one Athens centre.

---

## Priority Summary

| Priority | Items | Total effort |
| --- | --- | --- |
| **P0** Reproducibility | P0.2–P0.7 | about 1 week, mostly unattended runtime |
| **P1** Backend correctness | K1–K3, K6 first; then K4, K5, K7; then K8–K15 | 2–3 days |
| **P2** Frontend and UAV UI | F1–F3 first; then F4–F11 | 1–2 days |
| **P3** Methodology | E4, E6, E7 | 2–4 days |
| **P4** Features | Heat map, terrain overlay, tests and CI, blackboard, Godot/ROS | 1–3 weeks |
| **P5** Research | – | Open-ended |

Fixing K1 and K4 changes agent behaviour. Re-run the full study after them (with P0.5) rather than mixing pre-fix and post-fix data.
