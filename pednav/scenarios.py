"""Seeded pedestrian motion; only the environment knows future path segments."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Scenario:
    family: str
    seed: int
    halfwidth: float
    robot_start: tuple
    goal: tuple
    pedestrians: tuple

    def at(self, t):
        return np.array([p(t) for p in self.pedestrians], dtype=float).reshape(-1, 2)


def make_scenario(family: str, seed: int) -> Scenario:
    if family not in ('crossing', 'oncoming', 'stop_turn'):
        raise ValueError('Unknown scenario ' + family)
    rng = np.random.default_rng(int(seed) + 10000 * {'crossing': 1, 'oncoming': 2, 'stop_turn': 3}[family])
    ped = []
    if family == 'crossing':
        halfwidth = 1.15
        count = 2 if seed % 5 == 0 else 1
        for i in range(count):
            x = float(rng.uniform(-.35, .35)) + (i * .75)
            speed = float(rng.uniform(.28, .47))
            start = float(rng.uniform(7.0, 10.0)) + i * 4.0
            direction = 1 if i % 2 == 0 else -1
            # Starts outside the walkway, enters at 'start'.
            ped.append(lambda t, x=x, speed=speed, start=start, direction=direction:
                       (x, float(np.clip(direction * (-1.55 + speed * max(0., t - start)), -2., 2.))))
    elif family == 'oncoming':
        halfwidth = 0.82
        speed = float(rng.uniform(.11, .19))
        side = float(rng.uniform(.35, .52)) * (1 if seed % 2 else -1)
        ped.append(lambda t, speed=speed, side=side: (float(1.9 - speed * t), side))
        if seed % 6 == 0:
            ped.append(lambda t: (float(2.0 - .13 * max(0., t - 6.)), -.43))
    else:
        halfwidth = 1.15
        x = float(rng.uniform(-.1, .5))
        speed = float(rng.uniform(.27, .40))
        start = float(rng.uniform(7.0, 9.0))
        stop = start + float(rng.uniform(2.6, 4.0))
        resume = stop + float(rng.uniform(2.5, 4.5))
        def reversal(t, x=x, speed=speed, start=start, stop=stop, resume=resume):
            # Cross, unexpectedly stop, then reverse toward starting side.
            if t < start:
                y = -1.35
            elif t < stop:
                y = -1.35 + speed * (t - start)
            elif t < resume:
                y = -1.35 + speed * (stop - start)
            else:
                y = -1.35 + speed * (stop - start) - speed * .75 * (t - resume)
            return (x, float(max(-2., y)))
        ped.append(reversal)
        if seed % 7 == 0:
            ped.append(lambda t: (1.1, float(np.clip(1.6 - .30 * max(0., t - 13.), -2., 2.))))
    return Scenario(family, int(seed), halfwidth, (-2.0, 0.0, 0.0), (2.0, 0.0), tuple(ped))
