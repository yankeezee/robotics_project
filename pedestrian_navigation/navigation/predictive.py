"""Sophia's causal pedestrian prediction in Anastasiia's SHARED planner core.

The estimator imported below is the SAME ID-based implementation as in
pednav/sophia/estimation.py. Both controller variants inherit the same sampler,
scoring, guidance, wheel limits, collision checks, and braking fallback.
Only pedestrian_positions() differs during candidate rollouts.
"""
from pathlib import Path
import sys

# Webots runs the controller with pedestrian_navigation/ as its import root.
# The shared Sophia module lives in the containing Git repository.
_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import numpy as np
from pednav.sophia.estimation import OnlinePedestrianVelocityEstimator
from .core import Circle
from .planning import VelocitySamplingPlanner


class PredictivePlanner(VelocitySamplingPlanner):
    """Constant-velocity predictions estimated from current/past positions.

    command(...) updates the velocity history exactly ONCE per observation.
    Velocity estimates are then held fixed for all candidate trajectory checks
    in that planning step. A new planner instance is created for each trial.
    """

    def __init__(self, parameters, window=4, speed_cap_m_s=0.8):
        super().__init__(parameters)
        self.estimator = OnlinePedestrianVelocityEstimator(
            window=window, speed_cap_m_s=speed_cap_m_s)
        self._velocities = ()

    def reset(self):
        self.estimator.reset()
        self._velocities = ()
        self.passing.index, self.passing.side = None, 0
        self._position_cache.clear()
        self._cache_observation = None

    def command(self, observation, previous, dt):
        pedestrians = observation.pedestrians
        ids = observation.pedestrian_ids
        if len(ids) != len(pedestrians):
            raise ValueError('Predictive observations need one stable ID per pedestrian.')
        xy = np.asarray([(p.x, p.y) for p in pedestrians], dtype=float).reshape((-1, 2))
        velocities = self.estimator.update(observation.time, ids, xy)
        self._velocities = tuple((float(v[0]), float(v[1])) for v in velocities)
        output = super().command(observation, previous, dt)
        self.diagnostics['estimated_pedestrian_velocity_m_s'] = self._velocities
        return output

    def pedestrian_positions(self, observation, tau):
        if len(self._velocities) != len(observation.pedestrians):
            raise RuntimeError('Update velocities before planning candidate rollouts.')
        return tuple(Circle(p.x + vx*tau, p.y + vy*tau, p.radius)
                     for p, (vx, vy) in zip(observation.pedestrians, self._velocities))
