# Code review: integration of Sophia and Anastasiia

## Findings from uploaded branch ZIP

1. Anastasiia's controller contains runnable scene management, a pinned R2025b TurtleBot3 model, logging, scripted one-person crossing/oncoming/stop-turn scenarios, and a reactive subclass of a common velocity sampler. The supplied development log claims several Webots development runs. They have **not** been independently rerun in this environment.
2. Sophia's branch provides an ID-based, causal 4-observation velocity estimator, a separate `pednav.core` adapter, 15 Python tests and paired result analysis. Her earlier kinematic experiments have **different geometry and controller parameters** from Anastasiia's Webots world.
3. Neither original branch included a valid `predictive` mode in Anastasiia's Webots controller. An integration change is required; merging files alone would not enable prediction.
4. Most direct comparison is to keep Anastasiia's `VelocitySamplingPlanner` as common core and override **only** the pedestrian trajectory model using Sophia's estimator. Both modes retain shared static routing, guidance, collision checks, braking, and tuning.
5. Webots run `summary.json` records differ from Sophia analysis schema. `tools/export_paired_summary.py` translates verified run folders and checks version/parameter invariants; `tools/run_pairs.py` automates same-seed runs.

## Verification completed here

- Python `pytest` and `unittest` integration checks, including ID reordering, reset, shared core, limits, and raw-output conversion tests.
- Python code compilation.
- **No local Webots executable was available**, so no Webots simulation was run here.

## Risks and scope

- The predictor uses NumPy in Webots. The Webots controller must use a Python environment with NumPy available.
- Strong current-position passing preferences and conservative braking may dominate the effect of prediction. Report both successes and failures; there is no guarantee of improved performance.
- The single-pedestrian development scripts do **not** fulfill the original 1--5 pedestrian scope. Expand/test after first successful paired Webots runs.
- The predictive model extrapolates constant velocity; sudden turns and stops are a known failure mode. Predictions are not ground truth.
- Sample size and experimental protocol must be frozen before the held-out batch; don't treat already examined seeds 0-2 as test data.
