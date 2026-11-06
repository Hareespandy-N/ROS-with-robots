import math
import unittest

from amr_research_lab.navigation import Pose
from amr_research_lab.docking import DockingController, run_docking


class DockingTests(unittest.TestCase):
    def test_alignment_precedes_approach(self):
        dock = DockingController(Pose(1, 0, 0))
        v, w = dock.update(Pose(0, 0, math.pi / 2), 0.05)
        self.assertEqual(v, 0)
        self.assertLess(w, 0)
        self.assertEqual(dock.state, 'ALIGN')

    def test_simulated_docking_converges(self):
        result = run_docking()
        self.assertEqual(result['status'], 'DOCKED')
        self.assertLessEqual(result['xy_error_m'], 0.02)
        self.assertLessEqual(result['yaw_error_rad'], 0.03)

    def test_blockage_and_timeout_fail_without_retry(self):
        dock = DockingController(Pose(1, 0, 0))
        self.assertEqual(dock.update(Pose(0, 0, 0), 0.05, obstructed=True), (0, 0))
        self.assertEqual(dock.state, 'FAILED')
        self.assertEqual(dock.update(Pose(0, 0, 0), 0.05), (0, 0))
        dock = DockingController(Pose(1, 0, 0), timeout=0.05)
        dock.update(Pose(0, 0, 0), 0.1)
        self.assertEqual(dock.state, 'FAILED')

    def test_settling_requires_pose_and_heading(self):
        dock = DockingController(Pose(1, 0, 0))
        for _ in range(20):
            dock.update(Pose(1, 0, 0.2), 0.05)
        self.assertNotEqual(dock.state, 'DOCKED')
        for _ in range(20):
            dock.update(Pose(1, 0, 0), 0.05)
        self.assertEqual(dock.state, 'DOCKED')


if __name__ == '__main__':
    unittest.main()
