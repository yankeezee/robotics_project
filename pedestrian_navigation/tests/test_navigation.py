import math
import unittest
from navigation.core import Circle, Command, Observation, Parameters, Pose, advance, segment_distance, wheel_speeds
from navigation.reactive import ReactivePlanner
from navigation.guidance import PassingPreference, proximity_speed_limit
from navigation.routing import static_route
from navigation.scenarios import make_scenario


class NavigationChecks(unittest.TestCase):
    def test_braking_guard_checks_beyond_a_short_rollout(self):
        from dataclasses import replace
        p = replace(Parameters(), horizon=0.05)
        planner = ReactivePlanner(p)
        obs = Observation(0, Pose(0, 0, 0), (3, 0), (), (Circle(0.49, 0, 0.1),), 1.3)
        # The 0.05 s nominal arc is free; a control interval plus bounded stop is not.
        end = advance(obs.pose, Command(0.2, 0), p.horizon)
        self.assertGreater(0.49-end.x-p.robot_radius-0.1, p.safety_margin)
        self.assertFalse(planner.braking_admissible(obs, Command(0.2, 0), 0.064))

    def test_reactive_rollout_never_moves_pedestrian(self):
        planner = ReactivePlanner(Parameters())
        obs = Observation(7, Pose(0, 0, 0), (3, 0), (Circle(1, 0, 0.25),), (), 1.3)
        self.assertEqual(planner.pedestrian_positions(obs, 0), obs.pedestrians)
        self.assertEqual(planner.pedestrian_positions(obs, 100), obs.pedestrians)

    def test_passing_preference_holds_side_for_small_position_changes(self):
        from dataclasses import replace
        p, preference = Parameters(), PassingPreference(Parameters())
        obs = Observation(0, Pose(-2, 0, 0), (3, 0), (Circle(0, 0.001, 0.25),), (), 1.3)
        first = preference.target(obs)
        second = preference.target(replace(obs, time=0.064, pedestrians=(Circle(0, -0.001, 0.25),)))
        self.assertLess(first[1], 0)
        self.assertLess(second[1], 0)
        self.assertLessEqual(abs(first[1]), obs.half_width-p.robot_radius-p.safety_margin)

    def test_proximity_cap_preserves_reachable_deceleration(self):
        p, dt = Parameters(), 0.064
        obs = Observation(0, Pose(0, 0, 0), (3, 0), (Circle(0.9, 0, 0.25),), (), 1.3)
        self.assertLess(proximity_speed_limit(obs, p), 0.20)
        previous = Command(0.20, 0)
        command = ReactivePlanner(p).command(obs, previous, dt)
        self.assertLessEqual(abs(command.v-previous.v), p.max_acceleration*dt+1e-9)
        self.assertLessEqual(max(map(abs, wheel_speeds(command, p))), p.max_wheel_speed+1e-9)

    def test_static_route_clears_symmetric_obstacle(self):
        p, obstacle = Parameters(), Circle(0, 0, 0.25)
        points = [(-3, 0)] + static_route((-3, 0), (3, 0), (obstacle,), 1.3, p)
        self.assertEqual(points[-1], (3, 0))
        self.assertGreater(len(points), 2)
        for a, b in zip(points, points[1:]):
            distance = segment_distance((a[0]-obstacle.x, a[1]-obstacle.y),
                                        (b[0]-obstacle.x, b[1]-obstacle.y))
            self.assertGreaterEqual(distance, obstacle.radius+p.robot_radius+p.safety_margin+p.route_buffer)

    def test_differential_drive_wheel_convention(self):
        p = Parameters()
        left, right = wheel_speeds(Command(0.1, 0.5), p)
        self.assertAlmostEqual(p.wheel_radius*(left+right)/2, 0.1)
        self.assertAlmostEqual(p.wheel_radius*(right-left)/p.wheel_separation, 0.5)
        self.assertGreater(right, left)

    def test_relative_motion_detects_between_sample_crossing(self):
        self.assertAlmostEqual(segment_distance((-1, 0), (1, 0)), 0)
        self.assertAlmostEqual(segment_distance((-1, 0.5), (1, 0.5)), 0.5)

    def test_script_holds_and_seed_is_reproducible(self):
        scenario = make_scenario('stop_turn', 42)
        self.assertEqual(scenario, make_scenario('stop_turn', 42))
        pedestrian = scenario.pedestrians[0]
        start, end = pedestrian.waypoints[2][0], pedestrian.waypoints[3][0]
        before, during = pedestrian.position(start), pedestrian.position((start+end)/2)
        self.assertAlmostEqual(before.x, during.x)
        self.assertAlmostEqual(before.y, during.y)

    def test_empty_scene_reaches_goal_with_limits(self):
        p, previous, dt = Parameters(), Command(), 0.064
        planner = ReactivePlanner(p)
        pose, goal = Pose(-0.5, 0, 0), (0.5, 0)
        for tick in range(200):
            observation = Observation(tick*dt, pose, goal, (), (), 1.3)
            command = planner.command(observation, previous, dt)
            self.assertLessEqual(max(map(abs, wheel_speeds(command, p))), p.max_wheel_speed+1e-8)
            self.assertLessEqual(abs(command.v-previous.v), p.max_acceleration*dt+1e-8)
            self.assertLessEqual(abs(command.omega-previous.omega), p.max_angular_acceleration*dt+1e-8)
            pose, previous = advance(pose, command, dt), command
            if math.hypot(pose.x-goal[0], pose.y-goal[1]) <= p.goal_tolerance:
                return
        self.fail('Empty-scene goal was not reached.')


if __name__ == '__main__':
    unittest.main()
