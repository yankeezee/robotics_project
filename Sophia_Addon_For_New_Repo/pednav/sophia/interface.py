"""Planner/controller contract. All 2-D positions are world-frame metres.

Robot orientation: radians from +x; positive yaw is counterclockwise.
Time: seconds (monotonically increasing within one trial).
Pedestrian ID ordering is arbitrary, but IDs MUST be stable per person.
The observation deliberately contains NO future pedestrian trajectory.
"""
from dataclasses import dataclass
import math
from typing import Tuple
import numpy as np
from pednav.core import RobotState


@dataclass(frozen=True)
class NavigationObservation:
    time_s: float
    robot: RobotState
    pedestrian_ids: Tuple[str, ...]
    pedestrian_xy: np.ndarray
    goal_xy: Tuple[float, float]
    walkway_halfwidth_m: float

    def __post_init__(self):
        ids = tuple(str(x) for x in self.pedestrian_ids)
        positions = np.asarray(self.pedestrian_xy, dtype=float)
        if positions.shape != (len(ids), 2):
            raise ValueError(f'pedestrian_xy shape must be ({len(ids)}, 2), got {positions.shape}')
        if len(set(ids)) != len(ids):
            raise ValueError('pedestrian IDs must be unique in each observation')
        if not math.isfinite(self.time_s):
            raise ValueError('time_s must be finite')
        robot_values = (self.robot.x, self.robot.y, self.robot.theta, self.robot.v, self.robot.w)
        if not all(math.isfinite(float(v)) for v in robot_values):
            raise ValueError('robot state must be finite')
        if len(self.goal_xy) != 2 or not all(math.isfinite(float(v)) for v in self.goal_xy):
            raise ValueError('goal_xy must contain two finite coordinates')
        if not math.isfinite(self.walkway_halfwidth_m) or self.walkway_halfwidth_m <= 0:
            raise ValueError('walkway_halfwidth_m must be positive and finite')
        if not np.isfinite(positions).all():
            raise ValueError('pedestrian_xy must be finite')
        # Freeze a copy to avoid caller mutations subtly changing this observation.
        positions = positions.copy()
        positions.flags.writeable = False
        object.__setattr__(self, 'pedestrian_ids', ids)
        object.__setattr__(self, 'pedestrian_xy', positions)
        object.__setattr__(self, 'goal_xy', tuple(float(a) for a in self.goal_xy))


@dataclass(frozen=True)
class PlannerOutput:
    linear_m_s: float
    angular_rad_s: float
    pedestrian_velocity_m_s: np.ndarray
    predicted_xy_at_horizon: np.ndarray
    mode: str

    def __post_init__(self):
        if self.mode not in ('reactive', 'predictive'):
            raise ValueError('mode must be reactive or predictive')
        if not all(math.isfinite(v) for v in (self.linear_m_s, self.angular_rad_s)):
            raise ValueError('command must be finite')
