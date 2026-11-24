# Prospective Webots paired experiment — November 24, 2026

Work branch: `integration/webots-prediction`. Do not merge into `main` as part
of this verification. The initial integration revision is `0627669`.

## Simulator verification and integration changes

The original 40 Python tests and Sophia's smoke script passed locally under
Python 3.13.11 / NumPy 2.4.2 / Webots R2025b. Both modes were then run in the
actual simulator on development seed 3, including empty/static diagnostics and
the three pedestrian families. All ten initial diagnostic runs reached the goal.
Empty/static trajectories were identical between modes. Later cases in a reused
world had tiny initial pose differences after reset (up to about 2.7e-10 rad in
the observed yaw); residual wheel/caster state is also undesirable for paired
experiments. The experiment now starts a fresh Webots process per individual
trial instead of reusing one robot across the three families.

The isolated paired smoke check is saved in
`experiments/webots_integration_smoke_2026-11-24/`. All six real Webots trials
succeeded without collisions or clearance-margin violations. It independently
verifies identical pair configurations, source snapshots, scripted paths,
initial physical states, control timestamps, and command limits.

No shared planner weights, speed limits, geometry, scenario scripts, prediction
horizon, estimator settings, or safety thresholds were tuned during integration.
Both variants now cache repeated model-position queries within one control
update. This does not round timestamps or alter model values: a regression test
checks exact command and diagnostic equality against uncached reactive and
predictive references. The predictor still updates Sophia's estimator exactly
once per observation and receives no future script information.

The launcher now supports explicit output and shared-parameter paths, isolated
ports, a wall-clock infrastructure watchdog, resumable checkpoints, and source
and pair audits. More complete logs include estimated velocities/diagnostics,
physical initial states, simulator step, Python/package versions, configurations,
and source snapshots. Infrastructure failures stop the batch rather than being
reported as planner failures or silently discarded.

## Frozen evaluation design

- Three families: crossing, oncoming, unexpected stop/turn.
- Seeds **6000–6019**, not the inspected development seeds 0–3.
- 20 paired cases per family; **60 pairs / 120 real Webots runs**.
- Exactly one scripted pedestrian per case in the current integrated generator.
  This is narrower than the proposal's eventual one-to-five-person scope.
- Same TurtleBot3 Burger model, walkway, start/goal, physical random seed,
  pedestrian script, static route, parameters, limits, cost function, guidance,
  braking code, collision semantics, and termination conditions within a pair.
- Only the pedestrian rollout model differs: stationary positions versus causal
  constant-velocity extrapolation from Sophia's four-observation estimator,
  capped at 0.8 m/s.
- New Webots process and planner instance for every trial. At most two independent
  simulator processes may run concurrently. Webots uses fixed simulation time,
  not wall time, for physics, scripts, observations, and the timeout.
- Planner execution order alternates by seed parity. Every case is retained,
  including collisions, timeouts, and threshold violations.
- Parameters are serialized in `protocol.json`/`parameters.json` before any
  evaluation trial. Source files are hashed and archived at the same time;
  any source change during execution aborts the experiment.

Shared values: 0.064 s control step; 2 s horizon; wheel radius 0.033 m;
wheel separation 0.160 m; wheel limit 6.67 rad/s; robot circle radius 0.14 m;
pedestrian radius 0.25 m; maximum forward speed 0.20 m/s; maximum yaw rate
1.5 rad/s; translational/angular acceleration bounds 0.35 m/s² / 2.5 rad/s²;
goal tolerance 0.25 m; 120 s simulation timeout. Existing shared guidance retains
the 1.50 m proximity slowdown zone, 4.0 m passing trigger, 0.45 m preferred
passing clearance, and 0.10 m pedestrian planning reserve.

## Metrics and interpretation

Success means reaching the goal before timeout without a conservative geometric
collision. Collisions and timeouts are separate outcomes. Collision means
enclosing-circle overlap or overlap with walkway walls; this is not exact
physical contact detection. Boundary clearance is checked between samples using
swept relative motion. A minimum pedestrian clearance below 0.20 m is a violation.
Waiting is accumulated from actual planar speed below 0.01 m/s.

Travel time, path length, and their paired differences are computed for cases
where **both planners succeed**. Outcome counts remain visible so failed runs
cannot appear to be faster successful deliveries. Minimum clearance and waiting
also have all-outcome summaries, with the limitation that failures terminate
early. Report medians and individual paired points; no significance or
real-world safety claim is justified by this initial comparison.

## Reproduce

From the repository root after installing `requirements.txt` and Webots R2025b:

```powershell
python -m pytest -q
python pedestrian_navigation/tools/run_pairs.py --seed-start 6000 --trials 20 --workers 2 --output-root experiments/webots_paired_2026-11-24
python pedestrian_navigation/tools/analyze_experiment.py --experiment experiments/webots_paired_2026-11-24
```

For a continuation of the exact same frozen run, repeat the launch command with
`--resume`. Completed outcomes are audited and reused; they are not rerun based
on performance. Incomplete/infrastructure-failed directories are preserved and
require investigation. No tuning or changing seeds is allowed while resuming.

Raw files remain locally under `raw/` and are also packaged in `raw_logs.zip`.
`artifact_manifest.json` records checksums. Unzip that archive into the experiment
directory before reproducing analysis from a fresh checkout. The generated
`RESULTS.md`, CSV/JSON tables, dashboard, paired-difference plots, and Sophia's
analysis outputs are ready for the team's report review.
