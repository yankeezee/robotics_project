
# Pedestrian-Aware Navigation for a Delivery Robot

ME4670J Robotics Project

Team members:
- Anastasiia Ianke
- Sophia Olsson

## Project Overview

This project investigates whether short-horizon
pedestrian motion prediction improves navigation
for a differential-drive delivery robot.

We compare two local navigation strategies:

1. Reactive planning: pedestrians are assumed
   stationary during each planning rollout.
2. Predictive planning: pedestrian velocities
   are estimated from recent observations and
   used to predict future positions.

Both methods should use identical robot
parameters, control constraints, collision
checks, and scoring functions.

## Project Structure

### Shared Python foundation

`pednav/`

Contains the shared kinematic navigation
framework and simulation utilities.

### Sophia - Prediction and Analysis

`pednav/sophia/`

Contains:
- Pedestrian velocity estimation
- Predictive planner interface
- Controller integration
- Paired experiment analysis

Additional tests and analysis scripts are
located in `tests/` and `scripts/`.

### Anastasiia - Webots and Reactive Navigation

`pedestrian_navigation/`

Contains:
- Webots simulation environment
- TurtleBot3 robot controller
- Scripted pedestrian scenarios
- Reactive navigation implementation
- Scenario runner and logging
- Development experiments

Development documentation is located in
`pedestrian_navigation/research/`.

## Installation

Python 3.10 or newer is required.

Install shared Python dependencies:

```bash
python3 -m pip install -r requirements.txt
```

## Run Python Tests

From the repository root:

```bash
python3 -m pytest -q
python3 scripts/sophia_smoke.py
```

## Run Webots

Webots R2025b is the intended version.

From the Webots project directory:

```bash
cd pedestrian_navigation
python3 -m unittest discover -s tests -v
python3 run.py --scenario crossing --show
```

Both modes are now verified in Webots using the same scenes and shared controller:

```bash
python3 run.py --planner predictive --scenario crossing --seed 3 --show
```

## Experiments

Three scenario families are planned:

1. Crossing pedestrians
2. Oncoming pedestrians
3. Unexpected stops or turns

The completed Webots evaluation uses 20 paired trials per scenario family,
seeds 6000–6019, for a total of 60 pairs / 120 runs. The current integrated
scenarios use one pedestrian per case.

See the [full results and plots](experiments/webots_paired_2026-11-24/RESULTS.md),
[experiment protocol](docs/WEBOTS_EXPERIMENT_PROTOCOL.md), and
[reproducibility artifacts](experiments/README.md).

Metrics include:
- Goal success rate
- Collision count
- Travel time
- Path length
- Minimum pedestrian clearance
- Waiting time

## Development Status

The integrated project passes **45 Python tests**. Both planners have been
verified in Webots R2025b, with fresh processes and audited matching inputs.
Both reached the goal in all 60 evaluation cases, with no collisions/timeouts.
Oncoming clearance-margin violations occurred in 2/20 reactive and 3/20
predictive cases; prediction did not improve median paired travel time in
these tests. See the report for all six requested metrics and scope limits.

Code, parameters, seeds, raw observations, trajectories, diagnostics, source
archives, integrity checks and analysis outputs are saved. Raw logs are bundled
in each experiment's `raw_logs.zip`. Work remains on
`integration/webots-prediction`; no merge into `main` is included here.

Results from the earlier Python kinematic
simulator must remain separate from Webots
experimental results.

## Team Responsibilities

Anastasiia:
Webots environment, reactive planning,
pedestrian scenarios, and experiment logging.

Sophia:
Pedestrian motion prediction, shared
planner integration, and results analysis.

Shared:
Experiment design, integration testing,
report, demonstration, and presentation.
