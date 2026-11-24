"""Automated Sophia-module tests; no Webots installation required."""
import numpy as np
import pytest
from pednav.core import Config, LocalPlanner, RobotState, wheel_speeds
from pednav.sophia import NavigationObservation, PlannerAdapter, OnlinePedestrianVelocityEstimator


def obs(t, ids=('ped_1',), xy=((0., 1.),)):
    return NavigationObservation(float(t), RobotState(-2, 0, 0), tuple(ids),
                                 np.asarray(xy, dtype=float), (2., 0.), 1.15)


def test_causality_and_constant_velocity():
    estimator = OnlinePedestrianVelocityEstimator(window=4)
    assert np.allclose(estimator.update(0, ('A',), [[0, 0]]), [[0, 0]])
    assert np.allclose(estimator.update(.2, ('A',), [[.1, 0]]), [[.5, 0]])
    assert np.allclose(estimator.update(.4, ('A',), [[.2, 0]]), [[.5, 0]])


def test_speed_clipping():
    e = OnlinePedestrianVelocityEstimator(speed_cap_m_s=.8)
    e.update(0, ('A',), [[0, 0]])
    velocities = e.update(.1, ('A',), [[2, 0]])
    assert np.allclose(velocities, [[.8, 0]])


def test_ids_survive_reordering():
    e = OnlinePedestrianVelocityEstimator()
    e.update(0, ('A','B'), [[0,0],[1,0]])
    velocities = e.update(.2, ('B','A'), [[1,-.1],[.1,0]])
    assert np.allclose(velocities, [[0,-.5],[.5,0]])


def test_new_and_disappearing_pedestrians():
    e = OnlinePedestrianVelocityEstimator()
    e.update(0, ('A',), [[0,0]])
    v = e.update(.2, ('B',), [[1,1]])
    assert np.allclose(v, [[0,0]])
    v = e.update(.4, ('A',), [[.5,0]])
    assert np.allclose(v, [[0,0]])


def test_timestamp_validation_and_reset():
    e = OnlinePedestrianVelocityEstimator()
    e.update(0, ('A',), [[0,0]])
    with pytest.raises(ValueError, match='strictly increase'):
        e.update(0, ('A',), [[.1,0]])
    e.reset()
    assert np.allclose(e.update(0, ('A',), [[.1,0]]), [[0,0]])


def test_input_validation():
    with pytest.raises(ValueError, match='unique'):
        obs(0, ('A','A'), [[0,0],[1,1]])
    with pytest.raises(ValueError, match='shape'):
        obs(0, ('A',), [[0,0],[1,1]])
    with pytest.raises(ValueError, match='finite'):
        obs(float('nan'))


def test_shared_planner_interface_and_prediction_models():
    predictive = PlannerAdapter(mode='predictive')
    reactive = PlannerAdapter(mode='reactive')
    for t, pos in [(0, [[0,1]]), (.2, [[0,.9]])]:
        a = predictive.step(obs(t, xy=pos))
        b = reactive.step(obs(t, xy=pos))
    assert np.allclose(a.pedestrian_velocity_m_s, [[0,-.5]])
    assert np.allclose(a.predicted_xy_at_horizon, [[0, .9 - .5 * Config().horizon]])
    assert np.allclose(b.predicted_xy_at_horizon, [[0, .9]])
    assert a.mode == 'predictive' and b.mode == 'reactive'
    assert predictive.wheel_velocities(a) == wheel_speeds(a.linear_m_s,a.angular_rad_s, Config())


def test_identical_motion_when_no_pedestrians():
    a = PlannerAdapter(mode='predictive').step(obs(0, (), np.empty((0, 2))))
    b = PlannerAdapter(mode='reactive').step(obs(0, (), np.empty((0, 2))))
    assert (a.linear_m_s, a.angular_rad_s) == (b.linear_m_s, b.angular_rad_s)


def test_adaptor_equals_existing_core_for_same_inputs():
    p = PlannerAdapter(mode='predictive')
    p.step(obs(0, xy=[[0, 1]]))
    sample = obs(.2, xy=[[0, .9]])
    out = p.step(sample)
    manual = LocalPlanner(mode='predictive').command(sample.robot, sample.pedestrian_xy,
                                                    [[0,-.5]], sample.goal_xy, sample.walkway_halfwidth_m)
    assert np.allclose((out.linear_m_s, out.angular_rad_s), manual)
