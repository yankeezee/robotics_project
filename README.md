# Pedestrian-Aware Navigation for a Delivery Robot

ME4670J project by Anastasiia Ianke and Sophia Olsson.

This branch imports Anastasiia's Webots development project and the research
notes and evidence produced during reactive-baseline development.

| Directory | Purpose |
| --- | --- |
| [`pedestrian_navigation/`](pedestrian_navigation/README.md) | Runnable Webots R2025b project, reactive planner, scripted pedestrians, runner, tests, and development logs |
| [`pedestrian_navigation/research/`](pedestrian_navigation/research/2026-11-24_reactive_navigation.md) | Primary-source research, design decisions, failed attempts, and measured development results |
| [`PedestrianNav_Shared_Foundation/`](PedestrianNav_Shared_Foundation/README.md) | Existing provisional Python foundation from `main`, preserved unchanged |
| [`docs/ANASTASIIA_HANDOFF.md`](docs/ANASTASIIA_HANDOFF.md) | Integration differences to resolve with Sophia's separate prediction branch |

## Run Anastasiia's implementation

Install Webots R2025b and use Python 3.10 or newer. The controller, tests, and
launchers use the Python standard library; optional plotting uses Matplotlib.

```powershell
cd pedestrian_navigation
python -m unittest discover -s tests -v
python run.py --scenario crossing --show
python run.py --scenario crossing --scenario oncoming --scenario stop_turn --seed 0
```

Alternatively, open `pedestrian_navigation/worlds/walkway.wbt` in Webots and
start the simulation. Initial model loading requires internet access to retrieve
the pinned official TurtleBot3 Burger model and its assets.

## Development status

The revised reactive controller reaches the goal without collisions or recorded
0.20 m margin violations in the three seed-0 pedestrian examples tested in
Webots. Nine unit checks pass. Additional seed-2 tests include a borderline
crossing clearance of 0.200728 m; this is not a robust safety margin. Read the
research log for the earlier failures, parameter changes, exact run folders,
and limitations.

These are development results. A predictive Webots comparison, multiple-person
validation, shared parameter freeze, and held-out evaluation are still pending.
Sophia's existing `feature/sophia-prediction-analysis` branch uses a different
planner foundation and has not been merged into this import. The two
implementations must be reconciled before reporting a fair comparison.
