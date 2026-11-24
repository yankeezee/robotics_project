# Anastasiia's Webots implementation: integration handoff

This branch adds `pedestrian_navigation/` alongside the existing
`PedestrianNav_Shared_Foundation/`. It does not replace the existing foundation,
merge Sophia's branch, or claim that her predictor is already integrated.

## What is implemented and checked

- Webots R2025b world using the official TurtleBot3 Burger model.
- Current-position observations from a Supervisor, scripted development
  pedestrians, static obstacles, and a static-map route.
- Reactive velocity sampling with shared guidance, proximity speed regulation,
  wheel and command-acceleration limits, trajectory rejection, and braking guard.
- Trial reset, raw observations, diagnostic trajectories, summaries, event
  coverage, source archives, and SHA-256 manifests for revised Webots batches.
- Nine unit checks plus the documented single-pedestrian Webots development runs.

Original and revised failures are retained. A single successful run is not a
performance estimate, and the borderline seed-2 clearance is explicitly noted.

## Known integration differences

Sophia's handoff on `feature/sophia-prediction-analysis` proposes the shared
`pednav.core.LocalPlanner` and `PlannerAdapter` contract. The independently
developed Webots project currently has the following differences:

| Item | Anastasiia's current Webots implementation | Existing Python foundation / Sophia's proposed contract |
| --- | --- | --- |
| Planner API | `command(Observation, previous_command, dt)` | Adapter `.step(NavigationObservation)` / `LocalPlanner.command(...)` |
| Control update | 0.064 s Webots basic step | 0.20 s development setting |
| Prediction/rollout horizon | 2.0 s | 3.0 s development setting |
| Robot circle radius | 0.14 m | 0.20 m |
| Pedestrian radius | 0.25 m | 0.18 m |
| Maximum forward speed | 0.20 m/s | 0.18 m/s |
| Translational acceleration | 0.35 m/s² | 0.15 m/s² |
| Observed pedestrian identity | Stable within-trial node ordering | Explicit persistent IDs, potentially reordered observations |
| Output | Per-trial JSON and CSV trajectory logs | Shared flat `summary.csv` expected by Sophia's analysis |
| Extra guidance | Passing preference, proximity slowdown, and explicit braking guard | Different shared planner logic |

Both use x-y ground coordinates, positive yaw about +z, a 0.25 m goal tolerance,
a 120 s timeout, and a monitored 0.20 m clearance threshold. Matching those
few values does not make the implementations directly comparable.

## Required integration work before comparison

1. Select one shared controller core and one set of geometry and control
   parameters. Carry the useful Webots validation into that shared core.
2. Apply guidance, scoring, sampling, limits, buffers, and emergency rules
   identically to reactive and predictive variants. A parameter change must not
   be made independently in only one mode.
3. Connect persistent pedestrian IDs and reset observation history per trial.
   Keep future scripted paths entirely outside both planners.
4. Export the agreed outcome schema, including conservative geometric collision
   semantics, to the `summary.csv` consumed by Sophia's analysis.
5. Run matched reactive/predictive Webots smoke cases before the common freeze.
   Seeds 0, 1, and 2 are already inspected development data, not held-out seeds.

The import intentionally records this gap rather than presenting the separate
implementations' outputs as a valid paired experiment.
