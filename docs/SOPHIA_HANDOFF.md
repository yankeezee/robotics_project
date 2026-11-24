# Sophia to Anastasiia: integration contract

**Project:** ME4670J Pedestrian-Aware Navigation for a Delivery Robot.  
**Time-sensitive plan:** implementation Nov 24-29; paired evaluation Nov 30; analysis and report Dec 1; presentation Dec 2; joint submission check Dec 3, 2026.

## Division of work

Sophia's additive files are under `pednav/sophia/`, `scripts/sophia_*`, `tests/test_sophia*`, `report/sophia_*`, and `results/sophia_kinematic/`. These files do **not** change your Webots world generator, scenario scripts, baseline planner or trial-runner. The module internally delegates velocity sampling, rollout, constraints, scoring and braking to our existing shared `pednav.core.LocalPlanner`, with the mode switching only pedestrian prediction.

Anastasiia owns `webots/`, `pednav/scenarios.py`, the reactive baseline and generation of `results/webots/summary.csv`. **Please do not swap the two implementations of the planner independently; keep one shared core to make the method comparison valid.**

## Interface (contract to agree before merge)

1. World frame: `+x` along the walkway, `+y` lateral. Pose `(x,y,theta)` in m,m,rad; `theta=0` faces world `+x`, positive yaw counterclockwise.
2. `NavigationObservation(time_s, robot, pedestrian_ids, pedestrian_xy, goal_xy, walkway_halfwidth_m)` contains only *current* data. `pedestrian_xy` is `N x 2`. IDs must be stable across frames and may change row order; number of pedestrians may change. `time_s` must increase strictly within each trial.
3. `PlannerAdapter(mode='predictive')` keeps a rolling history and calls the **shared** `LocalPlanner` on observations. Call `adapter.reset()` on every new trial, even when reusing the same Python process.
4. `.step(obs)` returns `PlannerOutput(linear_m_s, angular_rad_s, pedestrian_velocity_m_s, predicted_xy_at_horizon, mode)`. Use `.wheel_velocities(output)` to obtain `(left_rad_s, right_rad_s)`.
5. For fair comparison, instantiate the same adapter with `mode='reactive'` for a test, OR connect Anastasiia's reactive baseline through this exact interface. Hold shared robot state, limits, geometric collision checking, cost function, replan interval and braking constant.
6. `predicted_xy_at_horizon` is diagnostic only. Neither planner must receive `scenario.at(t+tau)` or any future scripted position.

## Webots controller wiring pseudocode

```python
from pednav.sophia import PlannerAdapter, NavigationObservation
from pednav.core import Config, RobotState

cfg = Config()
adapter = PlannerAdapter(mode=mode, config=cfg)   # once, per trial
last_v, last_w = 0.0, 0.0

# Each controller update after reading CURRENT Supervisor positions:
obs = NavigationObservation(
    time_s=t,
    robot=RobotState(x, y, theta, last_v, last_w),
    pedestrian_ids=tuple(f'PED{i+1}' for i in range(len(current_ped_xy))),
    pedestrian_xy=current_ped_xy,
    goal_xy=sc.goal,
    walkway_halfwidth_m=sc.halfwidth,
)
output = adapter.step(obs)
left, right = adapter.wheel_velocities(output)
left_motor.setVelocity(left)
right_motor.setVelocity(right)
last_v, last_w = output.linear_m_s, output.angular_rad_s
```

**Webots caveat:** `pedestrian_ids` must identify physical pedestrians (persistent Webots DEF names), not arbitrarily changing detection indices. The supplied current controller holds one `PED1`, `PED2`, etc. each trial. Confirm actual TurtleBot3 orientation, wheel motor naming, wall collision geometry, timestep and positive wheel signs on the installed version of Webots. No Webots run has been verified here.

## How to synchronize

- Agree on all field names and units above before both branches diverge.
- Keep a stable header in Anastasiia's `summary.csv`:

`backend,family,seed,planner,success,collision,timeout,time_s,path_m,min_clearance_m,near_margin,wait_s,steps`

- Store raw Webots results under `results/webots/`, not `results/kinematic/`.
- Make one paired smoke run (same family + seed for both modes), then all 20 *held-out* seeds per family only after freezing code/settings.
- Share a single `summary.csv` and use:

```bash
python scripts/sophia_analyze.py --input results/webots/summary.csv --output results/sophia_webots --backend webots --expected-pairs 20
```

- This command **deliberately fails** if 60 pairs are incomplete, a CSV row is duplicated, mode/backends mismatch, or outcome labels conflict. Do not remove failed trials from the CSV.

## Initial freeze candidate (must be approved by both)

`dt=0.20 s, horizon=3.0 s, max_v=0.18 m/s, max_w=1.25 rad/s, a_v=0.15 m/s2, a_w=2.0 rad/s2, wheel_radius=0.033 m, wheel_base=0.16 m, wheel_limit=6.67 rad/s, robot_radius=0.20 m, pedestrian_radius=0.18 m, goal=0.25 m, timeout=120 s, clearance monitor=0.20 m, wait threshold=0.01 m/s`.

Original proposal says **2.0 s** horizon; 3.0 s is the *development-adjusted Python parameter*, not approved Webots evaluation tuning. Make an explicit freeze decision together on November 29 and document it before using held-out Webots seeds.

## Important open issues

- Does the Webots world load and allow a positive forward command to move the robot in +x?
- Is the pedestrian motion supplied at the same time as the robot observation (no one-step time mismatch)?
- Are true Webots contacts and the geometric collision rule consistent? Record both when possible.
- Which Webots version and Python interpreter are used?
- For `stop_turn`, are very close encounters caused by bad constant-velocity extrapolation or a shared local-planner limitation?

## Sync checklist

- [ ] Both can run `python -m pytest -q`.
- [ ] Anastasiia can run one predictive and one reactive Webots trial using the same interface.
- [ ] Seed, time, units, IDs, frame, logging schema and collision conventions are confirmed.
- [ ] Planner code/settings frozen before held-out evaluation.
- [ ] Sophia rechecks pair completeness and raw logs before making report figures.
- [ ] PR reviewed by both before merging.
