"""4-DOF yaw/shoulder/elbow/wrist kinematics independent of CoppeliaSim."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


class KinematicsError(ValueError):
    """Raised when a target or joint configuration is invalid."""


@dataclass(frozen=True)
class IKResult:
    joints: tuple[float, float, float, float]
    position_error: float
    reachable: bool
    iterations: int


class Arm4DOF:
    """Position kinematics for a yaw joint and a three-link planar chain.

    Joint order: base yaw, shoulder pitch, elbow pitch, wrist pitch.
    Angles are radians. In the arm plane, link directions are shoulder,
    shoulder+elbow, and shoulder+elbow+wrist. World coordinates are metres.
    The IK solves end-point position for a caller-specified wrist pitch; it
    does not solve arbitrary tool roll/yaw or collision avoidance.
    """

    def __init__(
        self,
        link_lengths: Sequence[float] = (0.28, 0.30, 0.20),
        joint_limits: Sequence[Sequence[float]] = (
            (-2.8, 2.8), (-1.6, 1.6), (-2.2, 2.2), (-2.0, 2.0)
        ),
        base_world: Sequence[float] = (0.0, 0.0, 1.03),
    ) -> None:
        if len(link_lengths) != 3 or any(not math.isfinite(float(x)) or float(x) <= 0 for x in link_lengths):
            raise ValueError("Exactly three positive finite link lengths are required")
        if len(joint_limits) != 4 or any(
            len(pair) != 2 or not math.isfinite(float(pair[0])) or not math.isfinite(float(pair[1])) or pair[0] >= pair[1]
            for pair in joint_limits
        ):
            raise ValueError("Four valid (min, max) joint-limit pairs are required")
        if len(base_world) != 3 or not all(math.isfinite(float(v)) for v in base_world):
            raise ValueError("base_world must be a finite XYZ triple")
        self.l1, self.l2, self.l3 = map(float, link_lengths)
        self.limits = tuple((float(a), float(b)) for a, b in joint_limits)
        self.base = tuple(map(float, base_world))

    @staticmethod
    def _clamp(value: float, limits: tuple[float, float]) -> float:
        return max(limits[0], min(limits[1], value))

    def within_limits(self, joints: Sequence[float], tolerance: float = 1e-9) -> bool:
        return len(joints) == 4 and all(
            low - tolerance <= float(q) <= high + tolerance
            for q, (low, high) in zip(joints, self.limits)
        )

    def forward(self, joints: Sequence[float]) -> tuple[float, float, float]:
        """Return tool-centre XYZ in world coordinates for four joint angles."""
        if len(joints) != 4 or not all(math.isfinite(float(q)) for q in joints):
            raise KinematicsError("joints must contain four finite angles")
        yaw, shoulder, elbow, wrist = map(float, joints)
        if not self.within_limits((yaw, shoulder, elbow, wrist)):
            raise KinematicsError("joint configuration violates joint limits")
        a1, a2, a3 = shoulder, shoulder + elbow, shoulder + elbow + wrist
        radial = self.l1 * math.cos(a1) + self.l2 * math.cos(a2) + self.l3 * math.cos(a3)
        z = self.base[2] + self.l1 * math.sin(a1) + self.l2 * math.sin(a2) + self.l3 * math.sin(a3)
        return (
            self.base[0] + radial * math.cos(yaw),
            self.base[1] + radial * math.sin(yaw),
            z,
        )

    def inverse(
        self,
        target: Sequence[float],
        wrist_pitch: float = 0.0,
        seed: Sequence[float] = (0.0, 0.3, -0.5, 0.0),
        position_tolerance: float = 0.015,
        max_iterations: int = 100,
    ) -> IKResult:
        """Numerically solve position IK with damped least-squares updates.

        Wrist pitch is fixed for each solve. Multiple initial configurations
        reduce sensitivity to the starting seed. The result is marked
        reachable only if the actual world-space FK error and joint limits
        satisfy the requested tolerances.
        """
        if len(target) != 3 or not all(math.isfinite(float(v)) for v in target):
            raise KinematicsError("target must contain finite XYZ coordinates")
        if len(seed) != 4 or not all(math.isfinite(float(v)) for v in seed):
            raise KinematicsError("seed must contain four finite joint angles")
        if not math.isfinite(wrist_pitch):
            raise KinematicsError("wrist pitch must be finite")
        if position_tolerance <= 0 or max_iterations < 1:
            raise ValueError("position_tolerance and max_iterations must be positive")
        tx, ty, tz = map(float, target)
        raw_yaw = math.atan2(ty - self.base[1], tx - self.base[0])
        yaw = self._clamp(raw_yaw, self.limits[0])
        target_r = math.hypot(tx - self.base[0], ty - self.base[1])
        target_z = tz - self.base[2]
        if math.hypot(target_r, target_z) > self.l1 + self.l2 + self.l3 + position_tolerance:
            q = (yaw, self._clamp(float(seed[1]), self.limits[1]),
                 self._clamp(float(seed[2]), self.limits[2]),
                 self._clamp(float(wrist_pitch), self.limits[3]))
            return IKResult(q, math.dist(self.forward(q), target), False, 0)

        wrist_in_range = self.limits[3][0] <= wrist_pitch <= self.limits[3][1]
        wrist = self._clamp(float(wrist_pitch), self.limits[3])
        initial_pairs = [
            (float(seed[1]), float(seed[2])),
            (-1.2, 1.8), (1.2, -1.8), (0.5, 0.8), (-0.5, -0.8),
            (-1.0, 1.0), (1.0, -1.0), (0.0, 1.0), (0.0, -1.0),
        ]
        best_q = (
            yaw,
            self._clamp(float(seed[1]), self.limits[1]),
            self._clamp(float(seed[2]), self.limits[2]),
            wrist,
        )
        best_error = math.dist(self.forward(best_q), target)
        total_iterations = 0

        for initial_shoulder, initial_elbow in initial_pairs:
            shoulder = self._clamp(initial_shoulder, self.limits[1])
            elbow = self._clamp(initial_elbow, self.limits[2])
            for iteration in range(max_iterations):
                a1 = shoulder
                a2 = shoulder + elbow
                a3 = shoulder + elbow + wrist
                radial = self.l1 * math.cos(a1) + self.l2 * math.cos(a2) + self.l3 * math.cos(a3)
                vertical = self.l1 * math.sin(a1) + self.l2 * math.sin(a2) + self.l3 * math.sin(a3)
                er, ez = target_r - radial, target_z - vertical
                planar_error = math.hypot(er, ez)
                candidate = (yaw, shoulder, elbow, wrist)
                world_error = math.dist(self.forward(candidate), target)
                total_iterations += 1
                if world_error < best_error:
                    best_q, best_error = candidate, world_error
                if planar_error <= position_tolerance and world_error <= position_tolerance:
                    return IKResult(candidate, world_error, wrist_in_range, total_iterations)

                # Jacobian columns are partial derivatives of (radial, vertical)
                # with respect to shoulder and elbow. Damped least squares keeps
                # updates finite near singular/fully extended arm configurations.
                j11 = -self.l1 * math.sin(a1) - self.l2 * math.sin(a2) - self.l3 * math.sin(a3)
                j12 = -self.l2 * math.sin(a2) - self.l3 * math.sin(a3)
                j21 = self.l1 * math.cos(a1) + self.l2 * math.cos(a2) + self.l3 * math.cos(a3)
                j22 = self.l2 * math.cos(a2) + self.l3 * math.cos(a3)
                damping = 0.015
                aa = j11*j11 + j12*j12 + damping*damping
                bb = j11*j21 + j12*j22
                cc = j21*j21 + j22*j22 + damping*damping
                determinant = aa*cc - bb*bb
                if abs(determinant) < 1e-12:
                    break
                u = (cc*er - bb*ez) / determinant
                v = (-bb*er + aa*ez) / determinant
                delta_shoulder = max(-0.25, min(0.25, j11*u + j21*v))
                delta_elbow = max(-0.25, min(0.25, j12*u + j22*v))
                shoulder = self._clamp(shoulder + delta_shoulder, self.limits[1])
                elbow = self._clamp(elbow + delta_elbow, self.limits[2])

        return IKResult(best_q, best_error, best_error <= position_tolerance and wrist_in_range, total_iterations)

    def reachable(self, target: Sequence[float], position_tolerance: float = 0.015) -> bool:
        return self.inverse(target, position_tolerance=position_tolerance).reachable
