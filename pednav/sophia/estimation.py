"""Causal pedestrian velocity estimation with ID-based temporal association.

Finite differences use earliest/latest observed *past and present* positions
within a rolling 4-observation window, per pedestrian ID. Missing objects are
removed from the active tracking state; a returning ID starts with zero speed.
No future scripted scenario waypoints are accessed.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass
class OnlinePedestrianVelocityEstimator:
    window: int = 4
    speed_cap_m_s: float = 0.8
    _tracks: dict = field(default_factory=dict, init=False, repr=False)
    _last_time_s: float = field(default=float('-inf'), init=False, repr=False)

    def __post_init__(self):
        if self.window < 2 or not isinstance(self.window, int):
            raise ValueError('window must be an integer >= 2')
        if not math.isfinite(self.speed_cap_m_s) or self.speed_cap_m_s <= 0:
            raise ValueError('speed_cap_m_s must be finite and positive')

    def reset(self):
        """Call at the start of EVERY independently seeded trial."""
        self._tracks.clear()
        self._last_time_s = float('-inf')

    def update(self, time_s, pedestrian_ids, pedestrian_xy):
        """Return an (N, 2) array aligned with pedestrian_ids."""
        ids = tuple(str(i) for i in pedestrian_ids)
        xy = np.asarray(pedestrian_xy, dtype=float)
        if xy.shape != (len(ids), 2):
            raise ValueError('pedestrian_xy must be an N x 2 array aligned with IDs')
        if len(set(ids)) != len(ids):
            raise ValueError('duplicate pedestrian ID')
        if not np.isfinite(xy).all() or not math.isfinite(time_s):
            raise ValueError('nonfinite positions or timestamp')
        if time_s <= self._last_time_s:
            raise ValueError('timestamps must strictly increase; call reset() for new trial')
        self._last_time_s = float(time_s)
        active = set(ids)
        for missing_id in set(self._tracks) - active:
            del self._tracks[missing_id]
        speeds = np.zeros_like(xy)
        for index, (name, pos) in enumerate(zip(ids, xy)):
            track = self._tracks.setdefault(name, deque(maxlen=self.window))
            track.append((float(time_s), pos.copy()))
            if len(track) < 2:
                continue
            dt = track[-1][0] - track[0][0]
            if dt <= 0:
                continue
            vel = (track[-1][1] - track[0][1]) / dt
            size = float(np.linalg.norm(vel))
            if size > self.speed_cap_m_s:
                vel *= self.speed_cap_m_s / size
            speeds[index] = vel
        return speeds
