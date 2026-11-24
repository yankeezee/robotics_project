"""Common local planner and differential drive geometry.

Model intentionally simplifies the Dynamic Window Approach and makes no
formal stopping-safety claims. The only algorithmic difference between
'reactive' and 'predictive' is how future pedestrian positions are modeled.
"""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np


@dataclass(frozen=True)
class Config:
    dt: float = 0.2
    horizon: float = 3.0
    max_v: float = 0.18
    max_w: float = 1.25
    accel_v: float = 0.15
    accel_w: float = 2.0
    wheel_radius: float = 0.033
    wheel_base: float = 0.16
    max_wheel_speed: float = 6.67
    robot_radius: float = 0.20  # deliberately conservative circular footprint
    pedestrian_radius: float = 0.18
    clearance_margin: float = 0.20  # monitored threshold, not certified safe distance
    goal_tolerance: float = 0.25
    timeout: float = 120.0
    wait_threshold: float = 0.01
    nv: int = 5
    nw: int = 7


@dataclass
class RobotState:
    x: float
    y: float
    theta: float
    v: float = 0.0
    w: float = 0.0


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def integrate(s: RobotState, v: float, w: float, dt: float) -> RobotState:
    if abs(w) < 1e-9:
        return RobotState(s.x + v * dt * math.cos(s.theta), s.y + v * dt * math.sin(s.theta), s.theta, v, w)
    th2 = s.theta + w * dt
    return RobotState(s.x + v / w * (math.sin(th2) - math.sin(s.theta)),
                      s.y - v / w * (math.cos(th2) - math.cos(s.theta)), float(wrap(th2)), v, w)


def wheel_speeds(v: float, w: float, cfg: Config):
    return ((v - cfg.wheel_base * w / 2) / cfg.wheel_radius,
            (v + cfg.wheel_base * w / 2) / cfg.wheel_radius)


def estimate_ped_velocity(history: deque, limit: float = 0.8):
    """Velocity from chronological (time, 2-D positions) samples; no future access."""
    if len(history) < 2:
        return np.zeros_like(history[-1][1], dtype=float)
    t0, p0 = history[0]
    t1, p1 = history[-1]
    dt = t1 - t0
    if dt <= 1e-8:
        return np.zeros_like(p1, dtype=float)
    vel = (np.asarray(p1) - np.asarray(p0)) / dt
    norm = np.linalg.norm(vel, axis=1)
    vel *= np.minimum(1.0, limit / np.maximum(norm, 1e-12))[:, None]
    return vel


class LocalPlanner:
    def __init__(self, cfg=None, mode='reactive'):
        if mode not in ('reactive', 'predictive'):
            raise ValueError('mode must be reactive or predictive')
        self.cfg = cfg or Config()
        self.mode = mode

    def command(self, robot: RobotState, peds: np.ndarray, estimates: np.ndarray,
                goal: tuple, halfwidth: float):
        """Returns v,w. peds shape (N,2), estimates shape (N,2)."""
        c = self.cfg
        peds = np.asarray(peds, dtype=float).reshape(-1, 2)
        estimates = np.asarray(estimates, dtype=float).reshape(-1, 2)
        if len(estimates) != len(peds):
            raise ValueError('pedestrian positions and velocities must match')
        # Shared emergency rule: maintain a low commanded velocity when already very close.
        # This is NOT a guarantee against a moving pedestrian entering a stationary robot.
        if len(peds) and np.min(np.linalg.norm(peds - np.array([robot.x, robot.y]), axis=1)) < (c.robot_radius + c.pedestrian_radius + 0.03):
            return max(0.0, robot.v - c.accel_v * c.dt), float(np.clip(0., robot.w - c.accel_w * c.dt, robot.w + c.accel_w * c.dt))
        v_lower = max(0.0, robot.v - c.accel_v * c.dt)
        v_upper = min(c.max_v, robot.v + c.accel_v * c.dt)
        w_lower = max(-c.max_w, robot.w - c.accel_w * c.dt)
        w_upper = min(c.max_w, robot.w + c.accel_w * c.dt)
        vs = np.linspace(v_lower, v_upper, c.nv)
        ws = np.linspace(w_lower, w_upper, c.nw)
        vs = np.unique(np.r_[vs, np.clip(robot.v, v_lower, v_upper)])
        ws = np.unique(np.r_[ws, np.clip(robot.w, w_lower, w_upper), np.clip(0.0, w_lower, w_upper)])
        V, W = np.meshgrid(vs, ws, indexing='ij')
        v = V.ravel()
        w = W.ravel()
        left = (v - .5 * c.wheel_base * w) / c.wheel_radius
        right = (v + .5 * c.wheel_base * w) / c.wheel_radius
        valid_wheels = (np.abs(left) <= c.max_wheel_speed + 1e-9) & (np.abs(right) <= c.max_wheel_speed + 1e-9)
        x = np.full_like(v, robot.x)
        y = np.full_like(v, robot.y)
        th = np.full_like(v, robot.theta)
        feasible = valid_wheels.copy()
        min_gap = np.full_like(v, 2.0)
        times = np.arange(1, round(c.horizon / c.dt) + 1) * c.dt
        pred_vel = estimates if self.mode == 'predictive' else np.zeros_like(estimates)
        for tau in times:
            # Exact constant-twist arc integration.
            th_next = th + w * c.dt
            straight = np.abs(w) < 1e-8
            w_safe = np.where(straight, 1.0, w)
            x += np.where(straight, v * c.dt * np.cos(th), v / w_safe * (np.sin(th_next) - np.sin(th)))
            y += np.where(straight, v * c.dt * np.sin(th), -v / w_safe * (np.cos(th_next) - np.cos(th)))
            th = th_next
            wall_gap = halfwidth - c.robot_radius - np.abs(y)
            feasible &= wall_gap >= 0.0
            min_gap = np.minimum(min_gap, wall_gap)
            if len(peds):
                future = peds + pred_vel * tau
                distances = np.sqrt((x[:, None] - future[None, :, 0]) ** 2 +
                                    (y[:, None] - future[None, :, 1]) ** 2)
                ped_gap = np.min(distances, axis=1) - c.robot_radius - c.pedestrian_radius
                feasible &= ped_gap >= 0.0
                min_gap = np.minimum(min_gap, ped_gap)
        if not np.any(feasible):
            # Limit deceleration to the configured bound. Zero is possible only near a stop.
            return max(0.0, robot.v - c.accel_v * c.dt), float(np.clip(0.0, w_lower, w_upper))
        before = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
        after = np.hypot(goal[0] - x, goal[1] - y)
        progress = before - after
        bearing = np.arctan2(goal[1] - y, goal[0] - x)
        alignment = np.cos(wrap(bearing - th))
        score = 8.0 * progress + 0.28 * alignment + 2.8 * np.clip(min_gap, 0, 0.7) + 0.18 * v
        score -= 0.04 * np.abs(w) + 0.08 * np.abs(w - robot.w) + 0.03 * np.abs(v - robot.v)
        score[~feasible] = -np.inf
        j = int(np.argmax(score))
        return float(v[j]), float(w[j])
