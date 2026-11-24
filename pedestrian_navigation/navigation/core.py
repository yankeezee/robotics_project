"""Shared planner interface. Units: metres, seconds, radians; x-y ground plane."""
from dataclasses import dataclass
import math
from typing import Protocol


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Observation:
    time: float
    pose: Pose
    goal: tuple[float, float]
    pedestrians: tuple[Circle, ...]
    obstacles: tuple[Circle, ...]
    half_width: float
    waypoint: tuple[float, float] | None = None
    # Stable within-trial IDs. Order may change but IDs must stay attached to people.
    pedestrian_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class Command:
    v: float = 0.0
    omega: float = 0.0


@dataclass(frozen=True)
class Parameters:
    wheel_radius: float = 0.033
    wheel_separation: float = 0.160
    max_wheel_speed: float = 6.67
    robot_radius: float = 0.14
    max_speed: float = 0.20
    max_turn_rate: float = 1.5
    max_acceleration: float = 0.35
    max_angular_acceleration: float = 2.5
    safety_margin: float = 0.20
    horizon: float = 2.0
    rollout_step: float = 0.10
    goal_tolerance: float = 0.25
    timeout: float = 120.0
    waiting_speed: float = 0.01
    route_buffer: float = 0.10
    pedestrian_slowdown_distance: float = 1.50  # Boundary-to-boundary distance.
    pedestrian_detour_distance: float = 4.00  # Along the x-oriented walkway.
    pedestrian_pass_clearance: float = 0.45
    passing_lookahead: float = 0.40
    pedestrian_model_buffer: float = 0.10  # Extra planning reserve; evaluation stays at 0.20 m.


class Planner(Protocol):
    def command(self, observation: Observation, previous: Command, dt: float) -> Command: ...


def wheel_speeds(command: Command, p: Parameters) -> tuple[float, float]:
    return ((command.v - p.wheel_separation * command.omega / 2) / p.wheel_radius,
            (command.v + p.wheel_separation * command.omega / 2) / p.wheel_radius)


def advance(pose: Pose, command: Command, dt: float) -> Pose:
    """Exact constant-twist integration, including straight motion."""
    theta = pose.yaw + command.omega * dt
    if abs(command.omega) < 1e-9:
        return Pose(pose.x + command.v * math.cos(pose.yaw) * dt,
                    pose.y + command.v * math.sin(pose.yaw) * dt, theta)
    radius = command.v / command.omega
    return Pose(pose.x + radius * (math.sin(theta) - math.sin(pose.yaw)),
                pose.y - radius * (math.cos(theta) - math.cos(pose.yaw)), theta)


def brake(previous: Command, dt: float, p: Parameters) -> Command:
    def reduce(value: float, maximum_change: float) -> float:
        return math.copysign(max(0.0, abs(value) - maximum_change), value)
    return Command(reduce(previous.v, p.max_acceleration * dt),
                   reduce(previous.omega, p.max_angular_acceleration * dt))


def clearance(pose: Pose, circles: tuple[Circle, ...], radius: float) -> float:
    return min((math.hypot(pose.x - c.x, pose.y - c.y) - radius - c.radius
                for c in circles), default=math.inf)


def segment_distance(start: tuple[float, float], end: tuple[float, float]) -> float:
    """Minimum distance from the origin to a relative-motion segment."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    alpha = 0.0 if length2 == 0 else max(0.0, min(1.0, -(start[0]*dx + start[1]*dy)/length2))
    return math.hypot(start[0] + alpha*dx, start[1] + alpha*dy)
