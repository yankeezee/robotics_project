"""Reactive variant: keep every pedestrian at its current observed position."""
from .planning import VelocitySamplingPlanner


class ReactivePlanner(VelocitySamplingPlanner):
    def pedestrian_positions(self, observation, tau):
        return observation.pedestrians
