"""Audit actual trial artifacts independently of planner decisions."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.core import Command, Parameters, wheel_speeds
from navigation.scenarios import Scenario, PedestrianScript, make_scenario


def source_files(repo):
    repo = Path(repo)
    root = repo/'pedestrian_navigation'
    return sorted({*root.glob('navigation/*.py'), *root.glob('controllers/*/*.py'),
                   *root.glob('tests/*.py'), *root.glob('config/*.json'), *root.glob('tools/*.py'),
                   *root.glob('worlds/*.wbt'), root/'run.py', root/'development.py',
                   *repo.glob('pednav/**/*.py'), *repo.glob('scripts/*.py'),
                   *repo.glob('tests/*.py'), repo/'requirements.txt', repo/'pytest.ini'})


def source_manifest(repo):
    repo = Path(repo)
    return {p.relative_to(repo).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source_files(repo)}


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def audit_pair(reactive, predictive):
    folders = [Path(reactive), Path(predictive)]
    configs = [read_json(p/'run_config.json') for p in folders]
    if [c['planner'] for c in configs] != ['reactive', 'predictive']:
        raise ValueError('Pair mode mismatch')
    if any(len(c['scenarios']) != 1 for c in configs):
        raise ValueError('A fresh simulator process is required for every trial')
    if any(configs[0][k] != configs[1][k] for k in ('seed','scenarios','parameters')):
        raise ValueError('Pair seed/scenario/parameter mismatch')
    for filename in ('source_manifest.json', 'environment.json'):
        if read_json(folders[0]/filename) != read_json(folders[1]/filename):
            raise ValueError(f'Pair {filename} mismatch')
    trials = [next(p.glob('00_*')) for p in folders]
    scenarios = [read_json(t/'scenario.json') for t in trials]
    if scenarios[0] != scenarios[1]:
        raise ValueError('Actual scenario scripts differ within pair')
    if read_json(trials[0]/'static_route.json') != read_json(trials[1]/'static_route.json'):
        raise ValueError('Static guidance differs within pair')
    starts = [read_json(t/'initial_state.json') for t in trials]
    max_initial_difference = max(abs(a-b) for key in starts[0]
                                 for a,b in zip(starts[0][key], starts[1][key]))
    if max_initial_difference > 1e-9:
        raise ValueError(f'Physical starting states differ: {max_initial_difference}')
    return dict(family=configs[0]['scenarios'][0], seed=configs[0]['seed'],
                scenario_sha256=hashlib.sha256(json.dumps(scenarios[0],sort_keys=True).encode()).hexdigest(),
                max_initial_state_difference=max_initial_difference,
                identical_configuration=True, identical_source=True, identical_environment=True)


def audit_trial(folder):
    folder = Path(folder)
    config, environment = read_json(folder/'run_config.json'), read_json(folder/'environment.json')
    if environment['backend'] != 'webots' or environment['basic_time_step_ms'] != 64:
        raise ValueError('Unexpected backend/control step')
    trial = next(folder.glob('00_*'))
    summary = read_json(trial/'summary.json')
    scenario = make_scenario(summary['scenario'], summary['seed'])
    if read_json(trial/'scenario.json') != json.loads(json.dumps(scenario.to_dict())):
        raise ValueError('Archived scenario differs from source-generated scenario')
    p = Parameters(**config['parameters'])
    with (trial/'trajectory.csv').open(newline='',encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    observations = [json.loads(line) for line in (trial/'observations.jsonl').read_text().splitlines()]
    diagnostic_count = sum(1 for _ in (trial/'planner_diagnostics.jsonl').open())
    if len(rows) != len(observations) or len(rows) != diagnostic_count or len(rows) < 2:
        raise ValueError('Missing raw samples')
    previous, max_script_error, max_velocity_violation = Command(), 0.0, 0.0
    for index, (row, obs) in enumerate(zip(rows, observations)):
        t = float(row['time_s'])
        if abs(t-obs['time']) > 1e-9 or abs(t-index*.064) > 1e-7:
            raise ValueError('Nonuniform or misaligned control timestamps')
        for measured, script in zip(obs['pedestrians'], scenario.pedestrians):
            expected = script.position(t)
            error = math.hypot(measured['x']-expected.x, measured['y']-expected.y)
            max_script_error = max(max_script_error, error)
        if index == len(rows)-1:
            continue  # Terminal zero command is not followed by another control interval.
        cmd = Command(float(row['command_v_m_s']),float(row['command_omega_rad_s']))
        violation = max(-cmd.v,cmd.v-p.max_speed,abs(cmd.omega)-p.max_turn_rate,
                        max(map(abs,wheel_speeds(cmd,p)))-p.max_wheel_speed,
                        abs(cmd.v-previous.v)-p.max_acceleration*.064,
                        abs(cmd.omega-previous.omega)-p.max_angular_acceleration*.064)
        max_velocity_violation = max(max_velocity_violation, violation)
        previous = cmd
    if max_script_error > 1e-8 or max_velocity_violation > 1e-8:
        raise ValueError(f'Script/command-limit audit failed: {max_script_error}, {max_velocity_violation}')
    def check(value, expected):
        if abs(value-expected) > 1e-6:
            raise ValueError(f'Raw/summary mismatch: {value} != {expected}')
    check(float(rows[-1]['time_s']), summary['elapsed_s'])
    check(min(float(r['pedestrian_clearance_m']) for r in rows), summary['min_pedestrian_clearance_m'])
    path = sum(math.hypot(float(b['x_m'])-float(a['x_m']),float(b['y_m'])-float(a['y_m']))
               for a,b in zip(rows,rows[1:]))
    check(path, summary['path_length_m'])
    check(sum(.064 for r in rows[1:] if float(r['speed_m_s']) < p.waiting_speed),summary['waiting_s'])
    last_distance = float(rows[-1]['goal_distance_m'])
    gap = min(summary['min_pedestrian_clearance_m'], summary['min_obstacle_clearance_m'])
    expected_outcome = 'collision' if gap <= 0 else ('success' if last_distance <= p.goal_tolerance else 'timeout')
    if summary['outcome'] != expected_outcome:
        raise ValueError('Terminal outcome disagrees with raw geometry')
    if expected_outcome == 'timeout' and summary['elapsed_s'] < p.timeout:
        raise ValueError('Premature timeout')
    if summary['clearance_violation'] != (summary['min_pedestrian_clearance_m'] < p.safety_margin):
        raise ValueError('Clearance flag disagrees with threshold')
    return dict(family=summary['scenario'], seed=summary['seed'], planner=summary['planner'],
                samples=len(rows), max_pedestrian_script_error_m=max_script_error,
                max_command_limit_violation=max_velocity_violation, raw_metrics_verified=True)
