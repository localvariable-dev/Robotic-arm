import math
import unittest

from src.kinematics.arm4dof import Arm4DOF, KinematicsError


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm4DOF(base_world=(0, 0, 0))

    def test_forward_returns_finite_world_position(self):
        p = self.arm.forward((0.2, 0.3, -0.4, 0.1))
        self.assertEqual(len(p), 3)
        self.assertTrue(all(math.isfinite(x) for x in p))

    def test_joint_limit_enforced(self):
        with self.assertRaises(KinematicsError):
            self.arm.forward((0, 99, 0, 0))

    def test_inverse_rejects_far_target(self):
        result = self.arm.inverse((10, 0, 10))
        self.assertFalse(result.reachable)

    def test_inverse_reaches_inside_workspace(self):
        target = self.arm.forward((0.1, 0.4, -0.5, 0.0))
        result = self.arm.inverse(target, seed=(0.1, 0.2, -0.3, 0.0), position_tolerance=0.025)
        self.assertTrue(result.reachable, result)
        self.assertLessEqual(result.position_error, 0.025)


if __name__ == "__main__":
    unittest.main()
