"""Shared velocity sampling, braking guard and scoring.

A future predictive variant overrides pedestrian_positions() only. All guidance,
sampling, dynamics limits, scoring and fallback remain shared. No formal DWA or
moving-obstacle safety guarantee is claimed.
"""
import math
from .core import Command, Observation, Parameters, advance, brake, segment_distance, wheel_speeds
from .guidance import PassingPreference, proximity_speed_limit


def samples(low, high, count):
    return [low+(high-low)*i/(count-1) for i in range(count)]


def segment_clearance(start, end, pedestrians_before, pedestrians_after, obs, p, arc_error=0.0):
    gap = obs.half_width-max(abs(start.y), abs(end.y))-p.robot_radius-arc_error
    for a, b in zip(pedestrians_before, pedestrians_after):
        distance = segment_distance((start.x-a.x, start.y-a.y), (end.x-b.x, end.y-b.y))
        gap = min(gap, distance-p.robot_radius-a.radius-p.pedestrian_model_buffer-arc_error)
    for c in obs.obstacles:
        distance = segment_distance((start.x-c.x, start.y-c.y), (end.x-c.x, end.y-c.y))
        gap = min(gap, distance-p.robot_radius-c.radius-arc_error)
    return gap


class VelocitySamplingPlanner:
    def __init__(self, parameters: Parameters):
        self.p = parameters
        if min(parameters.max_acceleration, parameters.max_angular_acceleration,
               parameters.rollout_step, parameters.horizon) <= 0:
            raise ValueError('Acceleration, horizon and rollout-step values must be positive.')
        self.passing = PassingPreference(parameters)
        self.diagnostics = {}

    def pedestrian_positions(self, observation: Observation, tau: float):
        return observation.pedestrians

    def _step(self, obs, pose, command, time, dt):
        end = advance(pose, command, dt)
        # Bound deviation of a constant-twist arc from its chord. A future
        # linear pedestrian model has no additional chord error.
        error = abs(command.v*command.omega)*dt*dt/8
        gap = segment_clearance(pose, end, self.pedestrian_positions(obs, time),
                                self.pedestrian_positions(obs, time+dt), obs, self.p, error)
        return end, gap

    def braking_admissible(self, obs, command, control_dt):
        """One command interval followed by a bounded stop against model obstacles."""
        p = self.p
        pose, gap = self._step(obs, obs.pose, command, 0.0, control_dt)
        if gap < p.safety_margin:
            return False
        elapsed, current = control_dt, command
        while abs(current.v)+abs(current.omega) > 1e-9:
            step = min(p.rollout_step, max(abs(current.v)/p.max_acceleration,
                                          abs(current.omega)/p.max_angular_acceleration))
            # Hold the pre-braking command over the interval before reducing it.
            # This is a discrete braking model, not a proof for the physical robot.
            pose, gap = self._step(obs, pose, current, elapsed, step)
            if gap < p.safety_margin:
                return False
            current = brake(current, step, p)
            elapsed += step
        return True

    def command(self, obs: Observation, previous: Command, dt: float) -> Command:
        if dt <= 0:
            raise ValueError('Control dt must be positive.')
        p = self.p
        if math.dist((obs.pose.x, obs.pose.y), obs.goal) <= p.goal_tolerance:
            self.diagnostics = {'mode': 'goal_braking'}
            return brake(previous, dt, p)
        target = self.passing.target(obs)
        limit = proximity_speed_limit(obs, p)
        low = max(0.0, previous.v-p.max_acceleration*dt)
        high = max(low, min(p.max_speed, previous.v+p.max_acceleration*dt, limit))
        vs = samples(low, high, 7)
        ws = samples(max(-p.max_turn_rate, previous.omega-p.max_angular_acceleration*dt),
                     min(p.max_turn_rate, previous.omega+p.max_angular_acceleration*dt), 13)
        if ws[0] <= 0 <= ws[-1]:
            ws.append(0.0)
        best, best_score, feasible_count = brake(previous, dt, p), -math.inf, 0
        initial = math.dist((obs.pose.x, obs.pose.y), target)
        for v in vs:
            for w in ws:
                cmd = Command(v, w)
                if max(map(abs, wheel_speeds(cmd, p))) > p.max_wheel_speed+1e-9:
                    continue
                if not self.braking_admissible(obs, cmd, dt):
                    continue
                pose, minimum, elapsed, feasible = obs.pose, 2.0, 0.0, True
                while elapsed < p.horizon-1e-9:
                    duration = min(p.rollout_step, p.horizon-elapsed)
                    pose, gap = self._step(obs, pose, cmd, elapsed, duration)
                    if gap < p.safety_margin:
                        feasible = False
                        break
                    minimum, elapsed = min(minimum, gap), elapsed+duration
                if not feasible:
                    continue
                feasible_count += 1
                remaining = math.dist((pose.x, pose.y), target)
                heading = math.atan2(target[1]-pose.y, target[0]-pose.x)
                score = (4*(initial-remaining)+0.4*math.cos(heading-pose.yaw)
                         +0.08*minimum+0.2*v-0.03*abs(w)-0.03*abs(w-previous.omega))
                if score > best_score:
                    best, best_score = cmd, score
        self.diagnostics = dict(mode='braking' if not feasible_count else
                                ('passing' if self.passing.index is not None else 'tracking'),
                                speed_limit_m_s=limit, target=target,
                                feasible_candidates=feasible_count)
        return best
