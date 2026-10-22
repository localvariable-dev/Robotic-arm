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
        q = (0.1, 0.4, -0.5, 0.0)
        target = self.arm.forward(q)
        result = self.arm.inverse(target, wrist_pitch=q[3], seed=(0.1, 0.2, -0.3, 0.0), position_tolerance=0.025)
        self.assertTrue(result.reachable, result)
        self.assertLessEqual(result.position_error, 0.025)

    def test_inverse_honours_nonzero_wrist_pitch(self):
        q = (0.2, 0.3, -0.4, 0.35)
        target = self.arm.forward(q)
        result = self.arm.inverse(target, wrist_pitch=q[3], seed=(0.2, 0.2, -0.3, 0.35), position_tolerance=0.025)
        self.assertTrue(result.reachable, result)
        self.assertLessEqual(result.position_error, 0.025)
        self.assertAlmostEqual(result.joints[3], q[3], places=6)


if __name__ == "__main__":
    unittest.main()
