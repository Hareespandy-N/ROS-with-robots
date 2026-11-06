"""Behavior checks: unsafe paths, bad input, and wrong integration must fail."""
import math
import unittest

from amr_research_lab.navigation import Pose, Rect, World, integrate, plan


class NavigationTests(unittest.TestCase):
    def test_straight_and_circular_integration(self):
        p = integrate(Pose(0, 0, 0), 1, 0, 2)
        self.assertAlmostEqual(p.x, 2)
        self.assertAlmostEqual(p.y, 0)
        p = integrate(Pose(0, 0, 0), 1, 1, math.pi / 2)
        self.assertAlmostEqual(p.x, 1)
        self.assertAlmostEqual(p.y, 1)
        self.assertAlmostEqual(p.yaw, math.pi / 2)

    def test_planner_detours_around_inflated_rectangle(self):
        world = World(4, 4, (Rect(1.5, 0, 2.0, 2.6),), 0.1)
        path = plan(world, (0.5, 0.5), (3.5, 0.5), 0.2)
        self.assertEqual(path[0], (0.5, 0.5))
        self.assertEqual(path[-1], (3.5, 0.5))
        self.assertGreater(max(p[1] for p in path), 2.8)
        for p in path:
            self.assertGreater(world.clearance(*p), 0.2)

    def test_blocked_and_unreachable_goals_fail_explicitly(self):
        world = World(3, 3, (Rect(1.3, 0, 1.7, 3),), 0.1)
        for target in ((1.5, 1.5), (2.5, 1.5), (-1, 1)):
            with self.subTest(target=target), self.assertRaises(ValueError):
                plan(world, (0.5, 1.5), target, 0.2)

    def test_diagonal_corner_cannot_be_crossed(self):
        world = World(3, 3, (Rect(0, 1.0, 1.0, 3), Rect(1.0, 0, 3, 1.0)), 0.1)
        with self.assertRaises(ValueError):
            plan(world, (0.5, 0.5), (2, 2), 0.05)

    def test_invalid_inputs_are_rejected(self):
        for values in ((float('nan'), 0, 0), (0, 0, float('inf'))):
            with self.assertRaises(ValueError):
                Pose(*values)
        for resolution in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                World(3, 3, (), resolution)
        with self.assertRaises(ValueError):
            integrate(Pose(0, 0, 0), 1, 0, -1)
        with self.assertRaises(ValueError):
            plan(World(3, 3), (0.5, 0.5), (2, 2), -0.1)


if __name__ == '__main__':
    unittest.main()
