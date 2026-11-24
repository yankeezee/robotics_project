"""Owns scene scripts and logging; passes only current observations to planning."""
import csv
import hashlib
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import traceback
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from controller import Supervisor
from navigation.core import (Circle, Command, Observation, Parameters, Pose,
                             clearance, segment_distance, wheel_speeds)
from navigation.reactive import ReactivePlanner
from navigation.scenarios import make_scenario
from navigation.routing import static_route


def cylinder_node(name, circle, color):
    return f'''DEF {name} Solid {{
      translation {circle.x} {circle.y} 0.45
      name "{name}"
      children [ Shape {{
        appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
        geometry Cylinder {{ radius {circle.radius} height 0.9 subdivision 20 }}
      }} ]
      boundingObject Cylinder {{ radius {circle.radius} height 0.9 }}
    }}'''


def run(supervisor):
    step = int(supervisor.getBasicTimeStep())
    dt = step / 1000.0
    config_path = Path(os.environ.get('NAV_RUN_CONFIG', ROOT / 'config/development.json'))
    settings = json.loads(config_path.read_text(encoding='utf-8-sig'))
    planner_name = settings.get('planner', 'reactive')
    if planner_name not in ('reactive', 'predictive'):
        raise ValueError('planner must be reactive or predictive')
    p = Parameters(**settings.get('parameters', {}))
    if not 0 < p.max_speed <= p.wheel_radius*p.max_wheel_speed:
        raise ValueError('Invalid maximum forward speed.')
    identifier = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output = Path(settings.get('output_dir', ROOT / 'results' / identifier))
    output.mkdir(parents=True, exist_ok=True)
    (output / 'run_config.json').write_text(json.dumps({**settings, 'parameters': asdict(p)}, indent=2), encoding='utf-8')
    # Archive both the Webots planner and the actual Sophia estimator source.
    repo = ROOT.parent
    sources = sorted({*ROOT.glob('navigation/*.py'), *ROOT.glob('controllers/*/*.py'),
                      *ROOT.glob('tests/*.py'), *ROOT.glob('config/*.json'),
                      *ROOT.glob('worlds/*.wbt'), ROOT/'run.py', ROOT/'development.py',
                      repo/'pednav/sophia/estimation.py', repo/'pednav/sophia/interface.py',
                      repo/'pednav/sophia/controller.py', repo/'pednav/core.py'})
    manifest = {str(path.relative_to(repo)).replace('\\', '/'): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sources}
    (output/'source_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    with zipfile.ZipFile(output/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(path, path.relative_to(repo))
    robot = supervisor.getSelf()
    translation, rotation = robot.getField('translation'), robot.getField('rotation')
    motors = [supervisor.getDevice('left wheel motor'), supervisor.getDevice('right wheel motor')]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0)
    children = supervisor.getRoot().getField('children')
    summary = []
    for index, family in enumerate(settings['scenarios']):
        scenario = make_scenario(family, int(settings.get('seed', 0)))
        if planner_name == 'predictive':
            # Load the NumPy-based estimator only when it is actually requested.
            from navigation.predictive import PredictivePlanner
            planner = PredictivePlanner(p)
        else:
            planner = ReactivePlanner(p)
        # New instance per trial resets the prediction history and passing state.
        route = static_route((scenario.start.x, scenario.start.y), scenario.goal,
                             scenario.obstacles, scenario.half_width, p)
        waypoint_index = 0
        translation.setSFVec3f([scenario.start.x, scenario.start.y, 0])
        rotation.setSFRotation([0, 0, 1, scenario.start.yaw])
        robot.resetPhysics()
        for motor in motors:
            motor.setVelocity(0)
        nodes, pedestrian_nodes = [], []
        for j, obstacle in enumerate(scenario.obstacles):
            name = f'OBSTACLE_{index}_{j}'
            children.importMFNodeFromString(-1, cylinder_node(name, obstacle, '0.85 0.45 0.12'))
            nodes.append(supervisor.getFromDef(name))
        for j, script in enumerate(scenario.pedestrians):
            name = f'PEDESTRIAN_{index}_{j}'
            children.importMFNodeFromString(-1, cylinder_node(name, script.position(0), '0.20 0.65 0.95'))
            node = supervisor.getFromDef(name)
            nodes.append(node)
            pedestrian_nodes.append(node)
        children.importMFNodeFromString(-1, f'''DEF GOAL_{index} Pose {{
          translation {scenario.goal[0]} {scenario.goal[1]} 0.003
          children [ Shape {{ appearance PBRAppearance {{ baseColor 0.2 0.85 0.3 metalness 0 roughness 1 }}
          geometry Cylinder {{ radius {p.goal_tolerance} height 0.006 }} }} ] }}''')
        nodes.append(supervisor.getFromDef(f'GOAL_{index}'))
        # Let contact physics settle before starting trial time or measurements.
        for _ in range(8):
            if supervisor.step(step) == -1:
                return
        origin = supervisor.getTime()
        trial_dir = output / f'{index:02d}_{family}_seed{scenario.seed}'
        trial_dir.mkdir()
        (trial_dir / 'scenario.json').write_text(json.dumps(scenario.to_dict(), indent=2), encoding='utf-8')
        (trial_dir / 'static_route.json').write_text(json.dumps(route, indent=2), encoding='utf-8')
        if settings.get('capture_image', False):
            supervisor.exportImage(str(trial_dir / 'scene.png'), 100)
        previous = Command()
        last_pose = None
        last_pedestrians = None
        distance = waiting = elapsed = 0.0
        min_pedestrian = min_obstacle = math.inf
        outcome = 'interrupted'
        with (trial_dir / 'trajectory.csv').open('w', newline='', encoding='utf-8') as log, \
             (trial_dir / 'observations.jsonl').open('w', encoding='utf-8') as observations:
            writer = csv.writer(log)
            writer.writerow(['time_s', 'x_m', 'y_m', 'yaw_rad', 'speed_m_s', 'omega_rad_s',
                             'command_v_m_s', 'command_omega_rad_s', 'goal_distance_m',
                             'pedestrian_clearance_m', 'obstacle_clearance_m', 'planner_mode',
                             'proximity_speed_limit_m_s', 'feasible_candidates', 'guide_x_m', 'guide_y_m'])
            while True:
                elapsed = supervisor.getTime() - origin
                position, orientation, velocity = robot.getPosition(), robot.getOrientation(), robot.getVelocity()
                pose = Pose(position[0], position[1], math.atan2(orientation[3], orientation[0]))
                # Read observed nodes, never the next scripted position, for planning.
                pedestrians = tuple(Circle(node.getPosition()[0], node.getPosition()[1], script.radius)
                                    for node, script in zip(pedestrian_nodes, scenario.pedestrians))
                while waypoint_index < len(route)-1 and math.dist((pose.x, pose.y), route[waypoint_index]) < 0.08:
                    waypoint_index += 1
                obs = Observation(elapsed, pose, scenario.goal, pedestrians, scenario.obstacles,
                                  scenario.half_width, route[waypoint_index],
                                  tuple(f"pedestrian_{j}" for j in range(len(pedestrians))))
                gap = clearance(pose, pedestrians, p.robot_radius)
                obstacle_gap = min(clearance(pose, scenario.obstacles, p.robot_radius),
                                   scenario.half_width - abs(pose.y) - p.robot_radius)
                speed = math.hypot(velocity[0], velocity[1])
                if last_pose is not None:
                    distance += math.hypot(pose.x - last_pose.x, pose.y - last_pose.y)
                    if speed < p.waiting_speed:
                        waiting += dt
                    # Swept relative motion avoids missing between-sample cylinder overlaps.
                    for old, new in zip(last_pedestrians, pedestrians):
                        gap = min(gap, segment_distance((last_pose.x-old.x, last_pose.y-old.y),
                                                       (pose.x-new.x, pose.y-new.y)) - p.robot_radius-new.radius)
                    for obstacle in scenario.obstacles:
                        obstacle_gap = min(obstacle_gap, segment_distance((last_pose.x-obstacle.x, last_pose.y-obstacle.y),
                                                                          (pose.x-obstacle.x, pose.y-obstacle.y))
                                           - p.robot_radius-obstacle.radius)
                min_pedestrian, min_obstacle = min(min_pedestrian, gap), min(min_obstacle, obstacle_gap)
                goal_distance = math.hypot(pose.x-scenario.goal[0], pose.y-scenario.goal[1])
                if min(gap, obstacle_gap) <= 0:
                    outcome = 'collision'
                elif goal_distance <= p.goal_tolerance:
                    outcome = 'success'
                elif elapsed >= p.timeout:
                    outcome = 'timeout'
                command = planner.command(obs, previous, dt) if outcome == 'interrupted' else Command()
                diagnostic = planner.diagnostics if outcome == 'interrupted' else {'mode': 'terminated'}
                target = diagnostic.get('target', ('', ''))
                writer.writerow([elapsed, pose.x, pose.y, pose.yaw, speed, velocity[5], command.v,
                                 command.omega, goal_distance, gap if math.isfinite(gap) else '', obstacle_gap,
                                 diagnostic.get('mode', ''), diagnostic.get('speed_limit_m_s', ''),
                                 diagnostic.get('feasible_candidates', ''), *target])
                observations.write(json.dumps(asdict(obs)) + '\n')
                supervisor.setLabel(0, f'{family} | {planner_name} | {elapsed:.1f}s | goal {goal_distance:.2f}m',
                                    0.02, 0.02, 0.07, 0xFFFFFF)
                for motor, target in zip(motors, wheel_speeds(command, p)):
                    motor.setVelocity(target)
                if outcome != 'interrupted':
                    break
                # Only the scene manager sees scripted future motion.
                for node, script in zip(pedestrian_nodes, scenario.pedestrians):
                    next_position = script.position(elapsed + dt)
                    node.getField('translation').setSFVec3f([next_position.x, next_position.y, 0.45])
                last_pose, last_pedestrians, previous = pose, pedestrians, command
                if supervisor.step(step) == -1:
                    break
        result = dict(scenario=family, seed=scenario.seed, planner=planner_name, outcome=outcome,
                      elapsed_s=elapsed, path_length_m=distance, waiting_s=waiting,
                      min_pedestrian_clearance_m=min_pedestrian if math.isfinite(min_pedestrian) else None,
                      min_obstacle_clearance_m=min_obstacle,
                      clearance_violation=min_pedestrian < p.safety_margin,
                      events_reached={name: elapsed >= time for name, time in scenario.events},
                      collision_definition='conservative circle overlap or walkway wall overlap')
        (trial_dir / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        summary.append(result)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print('TRIAL_RESULT ' + json.dumps(result), flush=True)
        for node in reversed(nodes):
            node.remove()
        for motor in motors:
            motor.setVelocity(0)
    print(f'RESULTS_DIRECTORY {output}', flush=True)
    if settings.get('quit_on_complete', False):
        supervisor.simulationQuit(0)
    else:
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)


if __name__ == '__main__':
    supervisor = Supervisor()
    try:
        run(supervisor)
    except Exception:
        traceback.print_exc()
        if os.environ.get('NAV_RUN_CONFIG'):
            supervisor.simulationQuit(1)
        else:
            supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
