"""Additive adapter to the project's *shared* DWA-inspired LocalPlanner.

Both variants use the SAME sampler, objective, constraints and braking rule
from pednav.core.LocalPlanner. The only model difference is that the original
LocalPlanner uses measured velocity in predictive mode and zero velocity in
reactive mode. This wrapper does not replace Anastasiia's reactive implementation.
"""
import numpy as np
from pednav.core import Config, LocalPlanner, wheel_speeds
from .estimation import OnlinePedestrianVelocityEstimator
from .interface import NavigationObservation, PlannerOutput


class PlannerAdapter:
    def __init__(self, mode='predictive', config=None, window=4, speed_cap_m_s=0.8):
        if mode not in ('predictive', 'reactive'):
            raise ValueError('unsupported mode')
        self.mode = mode
        self.config = config or Config()
        self.tracker = OnlinePedestrianVelocityEstimator(window=window, speed_cap_m_s=speed_cap_m_s)
        self._planner = LocalPlanner(cfg=self.config, mode=mode)

    def reset(self):
        self.tracker.reset()

    def step(self, observation: NavigationObservation) -> PlannerOutput:
        velocity = self.tracker.update(observation.time_s,
                                       observation.pedestrian_ids,
                                       observation.pedestrian_xy)
        v, w = self._planner.command(observation.robot,
                                     observation.pedestrian_xy,
                                     velocity,
                                     observation.goal_xy,
                                     observation.walkway_halfwidth_m)
        model_vel = velocity if self.mode == 'predictive' else np.zeros_like(velocity)
        future = observation.pedestrian_xy + model_vel * self.config.horizon
        return PlannerOutput(linear_m_s=v,
                             angular_rad_s=w,
                             pedestrian_velocity_m_s=velocity.copy(),
                             predicted_xy_at_horizon=future.copy(),
                             mode=self.mode)

    def wheel_velocities(self, output: PlannerOutput):
        """(left, right) wheel speeds in rad/s for the robot's motor API."""
        return wheel_speeds(output.linear_m_s, output.angular_rad_s, self.config)
