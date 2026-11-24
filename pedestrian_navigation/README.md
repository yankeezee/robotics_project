# Pedestrian-aware navigation: development starter

Anastasiia's Webots environment, scripted scenarios, reactive baseline and
logging are now connected to Sophia's ID-based velocity estimator through a
shared velocity-sampling planning core. This is a **Python-tested integration
draft**, not a verified predictive Webots result or final held-out evaluation.

Read [the integration guide](../docs/INTEGRATION_GUIDE_SV.md) and the
[original Anastasiia handoff](../docs/ANASTASIIA_HANDOFF.md). The earlier
`pednav.core` kinematic experiments use different parameters from the Webots
controller; do not treat their outcomes as a paired Webots comparison.

## Start

Requirements: Webots R2025b and Python 3.10 or newer. The **predictive** mode also requires NumPy; install the repository-root `requirements.txt` with the same Python interpreter used by the Webots controller. The reactive mode uses the standard library.
The first world load downloads the official TurtleBot3 Burger model and its
assets from Cyberbotics. Its source commit is pinned in `worlds/walkway.wbt`.

In PowerShell, from this project directory:

```powershell
python -m unittest discover -s tests -v
python run.py --scenario empty
python run.py --scenario static --scenario crossing --scenario oncoming --scenario stop_turn --seed 0
python run.py --scenario crossing --show
python run.py --planner predictive --scenario crossing --seed 3 --show
python development.py --scenario crossing --scenario oncoming --scenario stop_turn --seed 1 --seed 2
```

Alternatively, open `worlds/walkway.wbt` in Webots and press Run. The default
configuration runs one empty-scene case and pauses after completion. Edit
`config/development.json` to choose a different development case. Batch launches
use a separate configuration, close their own simulator after completion, and
do not modify the default configuration.

The launcher uses the Python interpreter that starts `run.py`. For direct GUI
launches, Webots must have a working Python command in Preferences.

## Scene and interface

- Ground plane: **x-y**; z is vertical (ENU). Robot forward is +x at yaw zero.
- Walkway: 8 m long, 2.6 m clear width; start (-3, 0), goal (3, 0).
- Wheel radius: 0.033 m; wheel separation: 0.160 m; wheel limit: 6.67 rad/s,
  matching the official model. Robot planning footprint: conservative 0.14 m circle.
- Orange cylinders: static obstacles. Blue cylinders: scripted pedestrians
  with a 0.25 m radius. Green disk: goal tolerance region.
- Supervisor privileges are enabled on the robot for this prototype. Scene
  management and observation are logically separated from the pure planner.
- `navigation/core.py`: `Observation`, `Command`, and shared `Parameters`.
  `planner.command(observation, previous_command, dt)` returns body velocities.
  Observations contain current pedestrian positions only: no future pedestrian
  paths, pedestrian waypoints, true velocities, or scenario identifiers are
  exposed to planning.
- `navigation/routing.py`: A* over the known static map with visibility smoothing.
  A static-map waypoint is included in each observation. Pedestrian script
  waypoints are never exposed. This shared guide avoids the symmetric-obstacle
  local minimum of goal-only scoring and must be identical for both variants.
- `navigation/reactive.py`: stationary pedestrian model.
- `navigation/predictive.py`: uses Sophia's `pednav/sophia/estimation.py`
  and predicts current positions forward by estimated constant velocity.
- `navigation/planning.py`: shared velocity sampling, two-second rollout, swept
  collision checks, a command-interval-plus-braking guard, scoring, and fallback.
- `navigation/guidance.py`: shared current-position passing preference and
  proximity slowdown. The passing heuristic is specific to this x-oriented
  walkway. It uses no pedestrian velocity or future script. Side preference
  persists during an encounter to reduce left-right switching.
- `development.py`: an explicitly kinematic development harness, not Webots.
- `navigation/scenarios.py`: deterministic development scripts for empty,
  static, crossing, oncoming, and sudden-stop/turn cases. These initial scripts
  contain zero or one pedestrian. Multiple pedestrians and held-out seeds remain
  to be added and validated before the final experiment.

Sophia's integrated `PredictivePlanner` subclasses `VelocitySamplingPlanner` and
updates velocity estimates **once** per new observation. Its rollout method only
uses the resulting cached estimate; it does not update history during trajectory
evaluation. The Webots controller gives pedestrians stable IDs. Shared guidance,
limits, sampling, scoring, buffers and braking remain identical for both modes.
No scenario-script future is passed into either planner.

## Results and definitions

Each launch creates a unique `results/<UTC timestamp>/` directory containing:

- `run_config.json`: resolved parameter snapshot; `webots.log` for batch runs.
- `source_snapshot.zip` and `source_manifest.json`: exact code and SHA-256 hashes
  for revised runs, allowing reproduction after subsequent source changes.
- Per-trial `scenario.json`: full script for reproducibility, Supervisor use only.
- `trajectory.csv`: actual pose, speed, commands, goal distance, clearances,
  passing/tracking/braking mode, speed cap, guide point, and feasible count.
- `observations.jsonl`: exact current observations available to the planner.
- `static_route.json`: shared map-based route; `scene.png` when `--snapshot` is used.
- Per-trial and batch `summary.json`: outcome, elapsed time, path length, waiting,
  minimum clearances, and whether the pedestrian margin was violated.
  The stop/turn scenario also records whether its scheduled events were reached.

Success: within 0.25 m of the goal before 120 s without a conservative geometric
collision. Collision takes priority over simultaneous success or timeout.
Collision here means overlap of the enclosing robot circle with a pedestrian,
static cylinder, or walkway wall. This deliberately conservative geometric
definition is **not** an exact physical-contact sensor. Cylinder overlaps are
also checked along relative-motion segments between simulation samples.
Waiting uses actual planar speed below 0.01 m/s. Boundary-to-boundary pedestrian
clearance below 0.20 m marks a margin violation. Empty scenes have null pedestrian
clearance, not an invented measurement. Scripted pedestrians do not yield.

Parameters are initial development choices. Do not treat these runs as the
proposal's 60 paired held-out cases or claim a predictive comparison yet.

## Initial checkpoint (November 24, 2026)

Five unit checks pass: wheel conventions, empty-scene convergence with command
limits, reproducible pedestrian motion, between-sample overlap detection, and
static-route clearance. All five development scenes also execute in Webots.
The latest seed-0 batch is saved in
`results/20261124T073024.104501Z/summary.json`:

| Development case | Outcome | Simulated time | Minimum relevant boundary clearance |
| --- | --- | ---: | ---: |
| Empty walkway | Success | 29.248 s | 1.160 m to walls |
| Static central cylinder | Success | 32.320 s | 0.310 m to static geometry |
| Crossing pedestrian | Conservative geometric collision | 14.720 s | -0.022 m to pedestrian |
| Oncoming pedestrian | Conservative geometric collision | 8.320 s | -0.009 m to pedestrian |
| Stop/turn pedestrian | Conservative geometric collision | 14.720 s | -0.022 m to pedestrian |

These are individual development runs from the original implementation, not
performance estimates. At this checkpoint, **pedestrian avoidance was not
ready**: braking alone could leave the robot in a moving pedestrian's path.
In seed 0 the stop/turn case ended
before the scripted stop, so it does not yet test the intended change of motion.
Preserve the failure logs while improving development behavior. An earlier
goal-only static baseline also timed out; the shared static-map route fixes that
observed local minimum. A verified scene snapshot is in
`results/20261124T072835.738919Z/00_static_seed0/scene_preview.png`.

## Revised pedestrian checkpoint (November 24, 2026)

Research, decisions, failed attempts, primary-source links, and final run
references are recorded in
[the development research log](research/2026-11-24_reactive_navigation.md).

The shared controller now slows down within 1.50 m pedestrian boundary clearance,
can prefer a passing side for a currently obstructing pedestrian within 4.0 m
along the walkway, and requests 0.45 m passing clearance. Candidate collision
checks include a 0.10 m pedestrian model reserve in addition to the original
0.20 m planning margin. Logged radii, geometric collision criteria, and the
0.20 m evaluation threshold are unchanged. These are development settings,
not final frozen experiment parameters.

The revised seed-0 pedestrian batch is saved in
`results/20261124T075112.859332Z/summary.json`:

| Development case | Outcome | Simulated time | Minimum pedestrian clearance | Margin violation |
| --- | --- | ---: | ---: | --- |
| Crossing | Success | 54.080 s | 0.249 m | No |
| Oncoming | Success | 35.584 s | 0.228 m | No |
| Stop/turn | Success | 68.096 s | 0.257 m | No |

The stop, turn, and final stop events are all reached in stop/turn. Empty and
static regression cases passed in `results/20261124T074925.966490Z/` at 29.248 s
and 32.320 s. Nine unit checks pass. An earlier revised oncoming run reached
only 0.185 m clearance; it is retained in that earlier batch and discussed in
the research log rather than omitted.

These outcomes establish a working development baseline for three single-person
cases. They do not establish real-world safety, performance over multiple
pedestrians, or any benefit from prediction. Proximity slowdown is intentionally
conservative and can increase travel time.

Additional seed-2 Webots checks passed in `results/20261124T075427.634085Z/`,
but crossing reached just 0.200728 m clearance, only about 0.73 mm above the
threshold. Treat this as a borderline development result; the current settings
are not yet ready to be frozen as a robust evaluation protocol. Seeds 0, 1, and
2 have been inspected as development data and must not be reused as held-out.

The before/after figure can be regenerated with optional Matplotlib:

```powershell
python tools/plot_development.py --before results/20261124T073024.104501Z --after results/20261124T075112.859332Z --output research/assets/seed0_reactive_revision.png
```

## Next work

1. Validate more development speeds and offsets and inspect any failures;
   preserve the current seed-0 evidence and source snapshots.
2. Review geometry, conservative speed regulation, and collision definitions
   together before fixing the final protocol.
3. Add 2--5 pedestrian cases, varied offsets, and separate development/test seeds.
4. Integrate Sophia's predictor, freeze shared settings, then run paired evaluation.

Official references:
- [TurtleBot3 Burger model](https://github.com/cyberbotics/webots/blob/d42e42bfe9a9f3c634cf318d9452eb92c487625a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto)
- [Supervisor API](https://cyberbotics.com/doc/reference/supervisor)
