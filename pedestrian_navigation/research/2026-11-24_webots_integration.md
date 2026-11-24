# Webots integration verification and paired evaluation

## What was verified before collecting evaluation results

The integrated GitHub revision was `0627669` on
`integration/webots-prediction`. Its 40 Python tests and Sophia's smoke script
passed. Actual Webots R2025b runs then checked both modes on development seed 3,
including empty/static controls and all three pedestrian families. Those ten
diagnostic trials are preserved separately from the paired evaluation.

The active Webots implementation uses `navigation.planning.VelocitySamplingPlanner`
for both modes. Sophia's ID-based estimator supplies the predictive pedestrian
positions. The older `pednav.core.Config` development values are not the
parameters used by the Webots controller. This distinction matters because the
repository contains both packages and their historical experiments.

## Shared changes and their rationale

Every experimental trial now starts a fresh Webots process. A reused world can
carry wheel/caster state between trials; the initial diagnostic batches also
showed tiny pose differences after earlier reactive/predictive encounters. The
fresh-process design makes the initial physical state directly comparable,
rather than relying only on setting the robot's position and clearing velocity.
The audits compare actual initial positions, orientations, velocities and wheel
sensor values, not just the nominal starting coordinates.

Repeated trajectory-model queries are cached per observation in both modes.
The predictor's history is updated once before planning. No timestamps are
rounded, no candidate velocities are removed, and no scores are changed. An
exact-equality regression checks cached versus uncached commands and diagnostics
for both variants. This reduces repeated work without redefining the comparison.

New experiment tools retain source snapshots, environments, complete diagnostics,
explicit run directories, checkpoints and input hashes. A strict audit recomputes
summary metrics from raw logs and checks scripted positions, timestamps and
command limits. Forty-five Python tests pass after these additions. A six-run
isolated seed-3 smoke test also passes in Webots.

The official [Webots Supervisor API](https://raw.githubusercontent.com/cyberbotics/webots/d42e42bfe9a9f3c634cf318d9452eb92c487625a/docs/reference/supervisor.md)
documents the privileged scene and state access used for simulation observation
and management. Planner inputs remain current observations; scripted future
positions belong only to the scene manager.

## Prospective evaluation

The code was committed at `f57b428` before any seed-6000 evaluation run. Shared
parameters, exact cases and source hashes were frozen in the experiment protocol.
The design is 20 seeds, 6000–6019, across crossing, oncoming and stop/turn:
60 pairs / 120 actual Webots trials. Planner order alternates by seed parity.
Two independent simulator processes may execute concurrently; all control and
pedestrian timing use simulation time.

No weights, geometry, control limits, thresholds, scripts, horizon or estimator
settings were tuned while observing these evaluation outcomes. Margin violations
are retained even if a run reaches the goal. Goal success and 0.20 m clearance
compliance are separate measurements.

## Case selected for review

Oncoming seed 6008 was selected during collection to illustrate a close encounter,
not as representative aggregate evidence. Both modes reach the goal, with minimum
gaps of approximately 0.157 m reactive and 0.118 m predictive. Its supplemental
plot shows measured paths, clearance, actual/commanded speed and lateral position.
These traces support further investigation; they do not alone establish a causal
explanation or an overall performance conclusion.

## Interpretation boundaries

The integrated generator currently uses one pedestrian per case, with accurate
Supervisor-provided positions. It does not test camera perception, uncertain
tracking, multi-person negotiation or the proposal's full one-to-five-person
scope. Travel/waiting time is simulated time; controller CPU cost and real-time
deadline behavior are not part of those metrics. The shared proximity slowdown
and passing preference can limit how much efficiency gain prediction can show.

Travel time and path length comparisons use only pairs where both modes succeed.
Collision, timeout and margin-violation counts remain visible for every trial.
The completed results and all artifacts are linked from
[`experiments/`](../../experiments/README.md), with a checksummed raw-log archive.

## Completed measurements

All 120 evaluation runs reached the goal without geometric collisions or
timeouts. No source, parameter, script or environment change occurred during
collection; all 60 pairs had exactly matching measured initial states. Every
recorded command passed its limits and every summary matched its raw samples.

Median paired predictive-minus-reactive travel-time differences were +0.064 s
crossing, +0.160 s oncoming, and 0.000 s stop/turn. Median paired path changes
were approximately +0.0015 m, +0.0012 m and -0.0013 m respectively. Oncoming
had two reactive versus three predictive margin violations, with worst gaps
0.157 m versus 0.118 m. No crossing or stop/turn run violated the 0.20 m margin.
The oncoming median paired waiting difference was +0.608 s; it was zero for the
other families. All scheduled stop/turn events were reached in all 40 runs.

The appropriate initial conclusion is a limited one: the current common
controller shows no median paired travel-time improvement from prediction,
and the observed oncoming proximity results deserve review. This is informative
even though it does not match the hoped-for efficiency improvement. It does
not justify extrapolation to multiple pedestrians, uncertain perception, different
guidance or physical delivery robots.
