"""Shared current-position guidance for this x-oriented development walkway.

No velocity estimates, future paths, or scenario labels are used here.
This preference is a heuristic, and never replaces trajectory rejection.
"""
import math
from .core import Observation, Parameters, clearance


class PassingPreference:
    def __init__(self, p: Parameters):
        self.p = p
        self.index = None
        self.side = 0

    def target(self, obs: Observation):
        p = self.p
        nominal = obs.waypoint or obs.goal
        direction = 1 if obs.goal[0] >= obs.pose.x else -1
        wall_limit = obs.half_width-p.robot_radius-p.safety_margin-p.route_buffer
        if self.index is not None:
            if (self.index >= len(obs.pedestrians) or
                direction*(obs.pedestrians[self.index].x-obs.pose.x) < -0.60):
                self.index, self.side = None, 0
        if self.index is None:
            candidates = []
            for i, pedestrian in enumerate(obs.pedestrians):
                forward = direction*(pedestrian.x-obs.pose.x)
                required = pedestrian.radius+p.robot_radius+p.pedestrian_pass_clearance
                if 0 <= forward <= p.pedestrian_detour_distance and abs(pedestrian.y-obs.pose.y) < required:
                    preferred = -1 if pedestrian.y >= obs.pose.y else 1
                    for side in (preferred, -preferred):
                        lane = pedestrian.y+side*required
                        if abs(lane) <= wall_limit:
                            candidates.append((forward, i, side))
                            break
            if candidates:
                _, self.index, self.side = min(candidates)
        if self.index is None:
            return nominal
        pedestrian = obs.pedestrians[self.index]
        required = pedestrian.radius+p.robot_radius+p.pedestrian_pass_clearance
        lane = pedestrian.y+self.side*required
        if abs(lane) > wall_limit:
            self.index, self.side = None, 0
            return nominal
        return (obs.pose.x+direction*p.passing_lookahead, lane)


def proximity_speed_limit(obs: Observation, p: Parameters):
    gap = clearance(obs.pose, obs.pedestrians, p.robot_radius)
    if math.isinf(gap) or gap >= p.pedestrian_slowdown_distance:
        return p.max_speed
    span = p.pedestrian_slowdown_distance-p.safety_margin
    if span <= 0:
        raise ValueError('Pedestrian slowdown distance must exceed the safety margin.')
    return p.max_speed*max(0.0, min(1.0, (gap-p.safety_margin)/span))
