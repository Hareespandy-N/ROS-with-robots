import math
import unittest

from amr_research_lab.navigation import Pose, Rect, World
from amr_research_lab.simulation import Limits, Scenario, Simulation, swept_clear


class SimulationTests(unittest.TestCase):
    def test_both_controllers_reach_goal_and_obey_shared_limits(self):
        for controller in ('baseline', 'regulated'):
            with self.subTest(controller=controller):
                sim = Simulation('open_floor', controller).run()
                self.assertEqual(sim.status, 'completed')
                self.assertLessEqual(sim.metrics()['final_xy_error_m'], 0.04)
                self.assertLessEqual(sim.metrics()['final_yaw_error_rad'], 0.04)
                for previous, sample in zip(sim.trace, sim.trace[1:]):
                    self.assertLessEqual(abs(sample.v), 0.600001)
                    self.assertLessEqual(abs(sample.w), 1.500001)
                    self.assertLessEqual(abs(sample.v - previous.v), 0.025001)
                    self.assertLessEqual(abs(sample.w - previous.w), 0.100001)
                    self.assertGreater(sample.clearance, 0)

    def test_warehouse_path_and_final_heading_converge(self):
        sim = Simulation('warehouse', 'regulated').run()
        self.assertEqual(sim.status, 'completed')
        self.assertLessEqual(sim.metrics()['final_yaw_error_rad'], 0.04)
        self.assertGreater(sim.metrics()['distance_m'], 8)

    def test_temporary_blockage_stops_then_resumes(self):
        sim = Simulation('temporary_blockage', 'regulated').run()
        self.assertEqual(sim.status, 'completed')
        self.assertGreater(sim.metrics()['blocked_time_s'], 0.5)
        self.assertGreater(min(s.clearance for s in sim.trace), 0)
        self.assertTrue(any(3 < s.t < 7 and s.v < 0.01 for s in sim.trace))

    def test_clearance_regulation_changes_the_experiment(self):
        scenario = Scenario('narrow', World(5, 1), Pose(0.6, 0.5, 0), Pose(4.4, 0.5, 0))
        baseline = Simulation(scenario, 'baseline').run()
        regulated = Simulation(scenario, 'regulated').run()
        self.assertEqual(baseline.status, 'completed')
        self.assertEqual(regulated.status, 'completed')
        self.assertGreater(regulated.metrics()['duration_s'], baseline.metrics()['duration_s'])

    def test_swept_check_detects_an_obstacle_between_endpoints(self):
        world = World(3, 3, (Rect(0.95, 0.2, 1.05, 2.8),))
        self.assertFalse(swept_clear(world, Pose(0.5, 1, 0), 1, 0, 1, 0.1))

    def test_timeout_and_collision_are_failures(self):
        self.assertEqual(Simulation('open_floor', limits=Limits(timeout=0.1)).run().status, 'timeout')
        sim = Simulation('open_floor')
        sim.pose = Pose(0.1, 2, 0)
        sim.step()
        self.assertEqual(sim.status, 'collision')

    def test_invalid_configuration_fails_before_running(self):
        for kwargs in ({'dt': 0}, {'max_v': -1}, {'radius': float('nan')}, {'timeout': -2}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Limits(**kwargs)
        with self.assertRaises(ValueError):
            Simulation('open_floor', 'unknown')

    def test_new_obstacle_cannot_win_a_race_with_completion(self):
        for window in ((0.3, 1.0), (0.01, 0.04)):
            with self.subTest(window=window):
                scenario = Scenario('goal_spawn', World(3, 3), Pose(1, 1, 0),
                                    Pose(1, 1, 0), Rect(0.5, 0.5, 1.5, 1.5), window)
                self.assertEqual(Simulation(scenario).run().status, 'collision')

    def test_invalid_blockage_windows_fail_at_construction(self):
        for window in ((float('nan'), 7), (3, float('inf')), (7, 3), (3,), (-1, 3), '37'):
            with self.subTest(window=window), self.assertRaises(ValueError):
                Scenario('bad_window', World(3, 3), Pose(1, 1, 0), Pose(2, 2, 0),
                         Rect(0.5, 0.5, 1.5, 1.5), window)
        with self.assertRaises(ValueError):
            Scenario('bad_obstacle', World(3, 3), Pose(1, 1, 0), Pose(2, 2, 0), 'rectangle')


if __name__ == '__main__':
    unittest.main()
