"""Fast kinematic development harness, explicitly separate from Webots evidence."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from navigation.core import Command, Observation, Parameters, advance, clearance, segment_distance
from navigation.reactive import ReactivePlanner
from navigation.routing import static_route
from navigation.scenarios import FAMILIES, make_scenario

ROOT = Path(__file__).resolve().parent


def simulate(family, seed=0, parameters=None, dt=0.064, mode='reactive'):
    p = parameters or Parameters()
    scenario = make_scenario(family, seed)
    if mode not in ('reactive', 'predictive'):
        raise ValueError('Unsupported planning mode')
    if mode == 'predictive':
        from navigation.predictive import PredictivePlanner
        planner = PredictivePlanner(p)
    else:
        planner = ReactivePlanner(p)
    pose, previous, t = scenario.start, Command(), 0.0
    route = static_route((pose.x, pose.y), scenario.goal, scenario.obstacles, scenario.half_width, p)
    waypoint_index, waiting, distance = 0, 0.0, 0.0
    min_ped, min_static = math.inf, math.inf
    outcome = 'timeout'
    while t < p.timeout:
        pedestrians = tuple(script.position(t) for script in scenario.pedestrians)
        while waypoint_index < len(route)-1 and math.dist((pose.x, pose.y), route[waypoint_index]) < 0.08:
            waypoint_index += 1
        obs = Observation(t, pose, scenario.goal, pedestrians, scenario.obstacles,
                          scenario.half_width, route[waypoint_index],
                          tuple(f'pedestrian_{j}' for j in range(len(pedestrians))))
        cmd = planner.command(obs, previous, dt)
        next_pose = advance(pose, cmd, dt)
        next_pedestrians = tuple(script.position(t+dt) for script in scenario.pedestrians)
        gap = clearance(next_pose, next_pedestrians, p.robot_radius)
        for old, new in zip(pedestrians, next_pedestrians):
            gap = min(gap, segment_distance((pose.x-old.x, pose.y-old.y),
                                           (next_pose.x-new.x, next_pose.y-new.y))-p.robot_radius-new.radius)
        static_gap = min(clearance(next_pose, scenario.obstacles, p.robot_radius),
                         scenario.half_width-abs(next_pose.y)-p.robot_radius)
        min_ped, min_static = min(min_ped, gap), min(min_static, static_gap)
        distance += math.dist((pose.x, pose.y), (next_pose.x, next_pose.y))
        waiting += dt if abs(cmd.v) < p.waiting_speed else 0
        pose, previous, t = next_pose, cmd, t+dt
        if min(gap, static_gap) <= 0:
            outcome = 'collision'
            break
        if math.dist((pose.x, pose.y), scenario.goal) <= p.goal_tolerance:
            outcome = 'success'
            break
    return dict(engine='kinematic_development', family=family, seed=seed, planner=mode, outcome=outcome,
                elapsed_s=t, pose=asdict(pose), waiting_s=waiting, path_length_m=distance,
                min_pedestrian_clearance_m=min_ped if math.isfinite(min_ped) else None,
                min_obstacle_clearance_m=min_static,
                clearance_violation=min_ped < p.safety_margin,
                events_reached={name: t >= time for name, time in scenario.events})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=FAMILIES, action='append')
    parser.add_argument('--seed', type=int, action='append', help='Repeat for several development seeds.')
    parser.add_argument('--planner', choices=('reactive', 'predictive'), default='reactive')
    args = parser.parse_args()
    p = Parameters()
    results = []
    for seed in args.seed or [0]:
        for family in args.scenario or FAMILIES:
            result = simulate(family, seed, p, mode=args.planner)
            results.append(result)
            print(json.dumps(result), flush=True)
    path = ROOT/'results'/('kinematic_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    path.mkdir(parents=True)
    (path/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    (path/'parameters.json').write_text(json.dumps(asdict(p), indent=2), encoding='utf-8')
    print(path, flush=True)


if __name__ == '__main__':
    main()
