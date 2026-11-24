# Two-person Webots integration — working draft

This is a **review snapshot** combining Anastasiia's Webots development source
and Sophia's prediction, estimator and analysis source. Use it to review an
integration pull request, not as a claim that a Webots experiment has succeeded.

**Start here:** [`docs/INTEGRATION_GUIDE_SV.md`](docs/INTEGRATION_GUIDE_SV.md)

## Run tests (repository root)

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -q
```

## Webots smoke comparison (from pedestrian_navigation/)

```bash
python3 run.py --planner reactive --scenario crossing --seed 3 --show
python3 run.py --planner predictive --scenario crossing --seed 3 --show
```

Both modes use the **same** robot model, script, planner core, scoring,
static guidance, and control constraints. The predictive mode reuses
`pednav/sophia/estimation.py` without changing that estimator.

## Evidence level

- Tests run in a normal Python process, **not Webots**.
- The supplied Anastasiia research note describes earlier *reactive* Webots
  development runs; these were not independently rerun here.
- No measured predictive Webots results are part of this package.
- Sophia's original 120 kinematic trials use a different controller and must be
  reported separately from future Webots comparisons.

The files under `pedestrian_navigation/` remain a development project with
one scripted pedestrian per scenario family; multiple-pedestrian validation
and held-out trial data are still required.
