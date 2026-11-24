"""Deterministic 2-D *kinematic* experiment harness, not Webots physics."""
from collections import deque
import csv
from pathlib import Path
import numpy as np
from .core import Config, RobotState, LocalPlanner, estimate_ped_velocity, integrate
from .scenarios import make_scenario


HEADERS = ['backend', 'family', 'seed', 'planner', 'success', 'collision', 'timeout',
           'time_s', 'path_m', 'min_clearance_m', 'near_margin', 'wait_s', 'steps']


def run_case(family, seed, mode, cfg=None, trace=False):
    c = cfg or Config()
    sc = make_scenario(family, seed)
    controller = LocalPlanner(c, mode)
    state = RobotState(*sc.robot_start)
    history = deque(maxlen=4)
    path = 0.0
    wait = 0.0
    min_clearance = float('inf')
    collision = False
    success = False
    rows = []
    total_steps = round(c.timeout / c.dt)
    for k in range(total_steps + 1):
        t = round(k * c.dt, 8)
        peds = sc.at(t)
        history.append((t, peds.copy()))
        estimates = estimate_ped_velocity(history)
        clearance = (np.min(np.linalg.norm(peds - [state.x, state.y], axis=1))
                     - c.robot_radius - c.pedestrian_radius) if len(peds) else float('inf')
        min_clearance = min(min_clearance, clearance)
        collision = clearance <= 0.0 or (sc.halfwidth - abs(state.y) - c.robot_radius) <= 0.0
        success = np.hypot(state.x - sc.goal[0], state.y - sc.goal[1]) <= c.goal_tolerance and not collision
        if trace:
            rows.append({'t': t, 'x': state.x, 'y': state.y, 'theta': state.theta,
                         'v': state.v, 'w': state.w, 'min_clearance': clearance,
                         'pedestrians': peds.tolist()})
        if collision or success or k == total_steps:
            break
        v, w = controller.command(state, peds, estimates, sc.goal, sc.halfwidth)
        nxt = integrate(state, v, w, c.dt)
        path += float(np.hypot(nxt.x - state.x, nxt.y - state.y))
        if v < c.wait_threshold:
            wait += c.dt
        state = nxt
    outcome = dict(zip(HEADERS, ['kinematic', family, int(seed), mode, bool(success), bool(collision),
                                 bool(not collision and not success), float(t), path, float(min_clearance),
                                 bool(min_clearance < c.clearance_margin), wait, k]))
    return (outcome, rows) if trace else outcome


def run_experiment(outdir, seeds=range(1000, 1020), families=('crossing', 'oncoming', 'stop_turn')):
    dest = Path(outdir)
    dest.mkdir(parents=True, exist_ok=True)
    out = []
    for family in families:
        for seed in seeds:
            for mode in ('reactive', 'predictive'):
                out.append(run_case(family, seed, mode))
    with (dest / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=HEADERS)
        writer.writeheader()
        writer.writerows(out)
    return out
