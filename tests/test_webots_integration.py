"""Integration tests for Sophia prediction through Anastasiia's shared planner.

Tests use synthetic observations only; they are NOT Webots experiment results.
"""
from dataclasses import replace
from pathlib import Path
import json
import math
import pytest
from navigation.core import Circle, Command, Observation, Parameters, Pose
from navigation.planning import VelocitySamplingPlanner
from navigation.reactive import ReactivePlanner
from navigation.predictive import PredictivePlanner
from pednav.sophia.estimation import OnlinePedestrianVelocityEstimator


def obs(t, pedestrians, ids, goal=(3,0)):
    return Observation(t, Pose(-1, 0, 0), goal, tuple(pedestrians), (), 1.3,
                       None, tuple(ids))


def test_prediction_only_overrides_pedestrian_position_model():
    assert issubclass(PredictivePlanner, VelocitySamplingPlanner)
    assert issubclass(ReactivePlanner, VelocitySamplingPlanner)
    for name in ('command','_step','braking_admissible'):
        assert getattr(PredictivePlanner,name) is getattr(ReactivePlanner,name) or (name == 'command' and PredictivePlanner.command.__qualname__.startswith('PredictivePlanner'))


def test_sophia_estimator_is_the_actual_reused_class():
    planner = PredictivePlanner(Parameters())
    assert isinstance(planner.estimator, OnlinePedestrianVelocityEstimator)


def test_reactive_remains_stationary_at_all_rollout_times():
    p=ReactivePlanner(Parameters())
    current=obs(0.0,[Circle(0,1,.25)],['person-A'])
    assert p.pedestrian_positions(current,3.5) == current.pedestrians


def test_predictive_uses_past_positions_and_stable_ids():
    p=PredictivePlanner(Parameters(horizon=2.0))
    a=obs(0.0,[Circle(0,1,.25),Circle(1,-1,.25)],['alpha','beta'])
    p.command(a,Command(),.064)
    b=obs(.2,[Circle(1,-.9,.25),Circle(0,.9,.25)],['beta','alpha'])
    p.command(b,Command(),.064)
    future=p.pedestrian_positions(b,2.0)
    assert future[0].y == pytest.approx(.1)  # beta +0.5 m/s *2
    assert future[1].y == pytest.approx(-.1) # alpha -0.5 m/s *2
    assert b.pedestrians[0].y == pytest.approx(-.9) # input not mutated


def test_new_trial_resets_track_history():
    planner=PredictivePlanner(Parameters())
    planner.command(obs(0.0,[Circle(0,1,.25)],['a']),Command(),.064)
    planner.command(obs(.2,[Circle(0,.9,.25)],['a']),Command(),.064)
    assert planner._velocities[0][1] < 0
    planner.reset()
    planner.command(obs(0.0,[Circle(0,-1,.25)],['a']),Command(),.064)
    assert planner._velocities == ((0.,0.),)


def test_missing_or_duplicate_ids_rejected():
    p=PredictivePlanner(Parameters())
    with pytest.raises(ValueError, match='stable ID'):
        p.command(obs(0.0,[Circle(0,1,.25)],[]),Command(),.064)
    with pytest.raises(ValueError, match='duplicate'):
        p.command(obs(.2,[Circle(0,1,.25), Circle(0,-1,.25)],['a','a']),Command(),.064)


def test_kinematic_observation_preserves_control_limits():
    params=Parameters()
    p=PredictivePlanner(params)
    cmd=p.command(obs(0.0,[Circle(2,1,.25)],['person']),Command(),.064)
    assert 0 <= cmd.v <= params.max_acceleration*.064+1e-9
    assert abs(cmd.omega) <= params.max_angular_acceleration*.064+1e-9


def test_predictive_and_reactive_use_identical_params():
    p=Parameters(horizon=2.0, robot_radius=.14)
    assert ReactivePlanner(p).p is PredictivePlanner(p).p


def test_scenarios_do_not_leak_scripted_future_to_observation():
    from navigation.scenarios import make_scenario
    scenario=make_scenario('stop_turn',1)
    persons=tuple(person.position(0) for person in scenario.pedestrians)
    observed=obs(0, persons, [f'pedestrian_{i}' for i in range(len(persons))])
    assert not hasattr(observed,'waypoints')
    assert all(not hasattr(person,'waypoints') for person in observed.pedestrians)


def test_run_modes_are_available_in_cli():
    run=Path(__file__).parents[1]/'pedestrian_navigation/run.py'
    s=run.read_text()
    assert "choices=('reactive', 'predictive')" in s


def test_webots_controller_not_hardcoded_to_reactive():
    control=Path(__file__).parents[1]/'pedestrian_navigation/controllers/navigation_supervisor/navigation_supervisor.py'
    text=control.read_text()
    assert 'PredictivePlanner' in text
    assert 'planner=planner_name' in text
    assert 'tuple(f"pedestrian_{j}"' in text
