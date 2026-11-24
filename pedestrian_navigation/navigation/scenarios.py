"""Supervisor-only development scripts. Never pass these paths to a planner."""
from dataclasses import asdict, dataclass
import random
from .core import Circle, Pose


@dataclass(frozen=True)
class PedestrianScript:
    radius: float
    # Time-stamped x-y positions; piecewise linear, holding at the last point.
    waypoints: tuple[tuple[float, float, float], ...]

    def position(self, time: float) -> Circle:
        if time <= self.waypoints[0][0]:
            return Circle(*self.waypoints[0][1:], self.radius)
        for (t0, x0, y0), (t1, x1, y1) in zip(self.waypoints, self.waypoints[1:]):
            if time <= t1:
                f = (time - t0)/(t1 - t0)
                return Circle(x0 + f*(x1-x0), y0 + f*(y1-y0), self.radius)
        return Circle(*self.waypoints[-1][1:], self.radius)


@dataclass(frozen=True)
class Scenario:
    family: str
    seed: int
    start: Pose
    goal: tuple[float, float]
    half_width: float
    pedestrians: tuple[PedestrianScript, ...]
    obstacles: tuple[Circle, ...]
    events: tuple[tuple[str, float], ...] = ()

    def to_dict(self):
        return asdict(self)


FAMILIES = ('empty', 'static', 'crossing', 'oncoming', 'stop_turn')


def make_scenario(family: str, seed: int = 0) -> Scenario:
    if family not in FAMILIES:
        raise ValueError(f'Unknown scenario {family!r}; choose from {FAMILIES}')
    rng = random.Random(seed)
    start, goal, half_width = Pose(-3.0, 0.0, 0.0), (3.0, 0.0), 1.3
    pedestrians, obstacles, events = (), (), ()
    if family == 'static':
        obstacles = (Circle(0.0, 0.0, 0.25),)
    elif family in ('crossing', 'stop_turn'):
        speed = rng.uniform(0.45, 0.65)
        crossing_time = rng.uniform(13.0, 16.0)
        begin, end = crossing_time - 1.0/speed, crossing_time + 1.0/speed
        if family == 'crossing':
            points = ((0.0, 0.0, -1.0), (begin, 0.0, -1.0), (end, 0.0, 1.0))
        else:
            # Abrupt stop near the path, followed by a turn along the walkway.
            stop = crossing_time - 0.1/speed
            points = ((0.0, 0.0, -1.0), (begin, 0.0, -1.0),
                      (stop, 0.0, -0.1), (stop + 2.0, 0.0, -0.1),
                      (stop + 6.0, 1.6, -0.1))
            events = (('pedestrian_0_stops', stop), ('pedestrian_0_turns', stop+2.0),
                      ('pedestrian_0_finishes', stop+6.0))
        pedestrians = (PedestrianScript(0.25, points),)
    elif family == 'oncoming':
        speed = rng.uniform(0.35, 0.50)
        y = rng.uniform(-0.15, 0.15)
        pedestrians = (PedestrianScript(0.25, ((0.0, 2.8, y), (5.6/speed, -2.8, y))),)
    return Scenario(family, seed, start, goal, half_width, pedestrians, obstacles, events)
