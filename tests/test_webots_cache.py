"""Caching must preserve both models' commands exactly, not just approximately."""
from navigation.core import Circle, Command, Observation, Parameters, Pose
from navigation.predictive import PredictivePlanner
from navigation.reactive import ReactivePlanner


class UncachedPredictive(PredictivePlanner):
    def _positions_for(self, observation, tau):
        return self.pedestrian_positions(observation, tau)


class UncachedReactive(ReactivePlanner):
    def _positions_for(self, observation, tau):
        return self.pedestrian_positions(observation, tau)


def test_cache_preserves_reactive_and_predictive_commands_exactly():
    for kind, reference in ((ReactivePlanner, UncachedReactive), (PredictivePlanner, UncachedPredictive)):
        a, b = kind(Parameters()), reference(Parameters())
        previous = Command(.1, .05)
        for frame in range(12):
            observation = Observation(frame*.064, Pose(-1, .03, .02), (3,0),
                                      (Circle(.2, .9-frame*.03, .25),), (), 1.3,
                                      None, ('person',))
            actual, expected = a.command(observation, previous, .064), b.command(observation, previous, .064)
            assert actual == expected
            assert a.diagnostics == b.diagnostics
            previous = actual
