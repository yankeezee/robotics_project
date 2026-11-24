# Pedestrian-Aware Navigation — shared starting point

ME4670J. Team: Anastasiia Ianke and Sophia Olsson.

This is a **provisional shared Python foundation**, not a validated Webots project and not Anastasiia's completed implementation. It implements a compact kinematic planner, scenario generation and a reusable controller interface. Both team members must review and freeze common parameters before any final Webots comparison.

## Installation and initial check

```bash
python -m pip install -r requirements.txt
python -c "from pednav.core import LocalPlanner, Config; print('Shared foundation loaded:', Config().horizon)"
```

## Division of responsibility

- Sophia: shared planner interface, pedestrian velocity estimation, predictive mode integration and paired analysis (separate feature branch).
- Anastasiia: Webots environment, scripted pedestrians, baseline integration, experiment runner and trial logging (separate feature branch).
- Joint: collision semantics, coordinate conventions, parameter freeze, testing and results.

`pednav/core.py` includes a **reference reactive mode** to share scoring/sampling with predictive mode. It is not yet a verified Webots baseline and should not be represented as Anastasiia's authored code.

Next: upload Sophia's add-on on branch `feature/sophia-prediction-analysis`.
