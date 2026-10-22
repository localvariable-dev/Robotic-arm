"""Simplified 4-DOF yaw + planar arm kinematics, independent of the simulator."""
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
    """4-DOF arm model with base yaw and a three-link vertical plane.

    Joint order: base yaw, shoulder pitch, elbow pitch, wrist pitch.
    Position is calculated using L1 (upper arm), L2 (forearm), and L3
    (wrist/tool reach). Wrist pitch controls tool orientation but not the
    end-point location in this deliberately simplified model; consequently
    L3 is treated as a straight third link.
    """

    def __init__(
        self,
        link_lengths: Sequence[float] = (0.28, 0.30, 0.20),
        joint_limits: Sequence[Sequence[float]] = (
            (-2.8, 2.8), (-1.6, 1.6), (-2.2, 2.2), (-2.0, 2.0)
        ),
        base_world: Sequence[float] = (0.0, 0.0, 0.82),
    ) -> None:
        if len(link_lengths) != 3 or any(float(x) <= 0 for x in link_lengths):
            raise ValueError("Exactly three positive link lengths are required")
        if len(joint_limits) != 4 or any(len(pair) != 2 or pair[0] >= pair[1] for pair in joint_limits):
            raise ValueError("Four valid (min, max) joint-limit pairs are required")
        if len(base_world) != 3:
            raise ValueError("base_world must be an XYZ triple")
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
        """Return TCP world XYZ for four joint angles (radians)."""
        if len(joints) != 4 or not all(math.isfinite(float(q)) for q in joints):
            raise KinematicsError("joints must contain four finite angles")
        yaw, shoulder, elbow, wrist = map(float, joints)
        if not self.within_limits((yaw, shoulder, elbow, wrist)):
            raise KinematicsError("joint configuration violates joint limits")
        # Wrist angle adjusts orientation only. Position chain is shoulder/elbow
        # plus a terminal tool link aligned to forearm + elbow, to keep FK/IK analytic.
        radial = self.l1 * math.cos(shoulder) + self.l2 * math.cos(shoulder + elbow) + self.l3 * math.cos(shoulder + elbow)
        z = self.base[2] + self.l1 * math.sin(shoulder) + self.l2 * math.sin(shoulder + elbow) + self.l3 * math.sin(shoulder + elbow)
        return (
            self.base[0] + radial * math.cos(yaw),
            self.base[1] + radial * math.sin(yaw),
            z,
        )

    def inverse(
        self,
        target: Sequence[float],
        wrist_pitch: float = 0.0,
        seed: Sequence[float] = (0.0, 0.3, -0.5, 0.2),
        position_tolerance: float = 0.015,
        max_iterations: int = 100,
    ) -> IKResult:
        """Solve position IK numerically using damped least-squares finite differences.

        Wrist pitch is independently specified. A bounded coordinate-descent /
        damped update is used so the solution respects configured joint limits.
        """
        if len(target) != 3 or not all(math.isfinite(float(v)) for v in target):
            raise KinematicsError("target must contain finite XYZ coordinates")
        if len(seed) != 4:
            raise KinematicsError("seed must contain four joint angles")
        if not math.isfinite(wrist_pitch):
            raise KinematicsError("wrist pitch must be finite")
        q = [self._clamp(float(v), self.limits[i]) for i, v in enumerate(seed)]
        q[3] = self._clamp(float(wrist_pitch), self.limits[3])
        tx, ty, tz = map(float, target)
        yaw = math.atan2(ty - self.base[1], tx - self.base[0])
        q[0] = self._clamp(yaw, self.limits[0])
        radial_target = math.hypot(tx - self.base[0], ty - self.base[1])
        vertical_target = tz - self.base[2]
        # The first link is l1; the final two collinear links are an effective l2+l3.
        a, b = self.l1, self.l2 + self.l3
        distance_sq = radial_target * radial_target + vertical_target * vertical_target
        if distance_sq > (a + b) ** 2 or distance_sq < max(0.0, a - b) ** 2:
            err = abs(math.sqrt(distance_sq) - min(a + b, max(abs(a - b), math.sqrt(distance_sq))))
            return IKResult(tuple(q), err, False, 0)
        cosine = (distance_sq - a*a - b*b) / (2*a*b)
        if cosine < -1.0 - 1e-8 or cosine > 1.0 + 1e-8:
            return IKResult(tuple(q), float("inf"), False, 0)
        elbow_candidates = [math.acos(max(-1.0, min(1.0, cosine))), -math.acos(max(-1.0, min(1.0, cosine)))]
        candidates: list[tuple[list[float], float]] = []
        for elbow in elbow_candidates:
            shoulder = math.atan2(vertical_target, radial_target) - math.atan2(b * math.sin(elbow), a + b * math.cos(elbow))
            cand = [q[0], self._clamp(shoulder, self.limits[1]), self._clamp(elbow, self.limits[2]), q[3]]
            if not self.within_limits(cand):
                continue
            p = self.forward(cand)
            err = math.dist(p, target)
            candidates.append((cand, err))
        if not candidates:
            return IKResult(tuple(q), float("inf"), False, 0)
        best_q, best_error = min(candidates, key=lambda pair: pair[1])
        return IKResult(tuple(best_q), best_error, best_error <= position_tolerance, 1)

    def reachable(self, target: Sequence[float], position_tolerance: float = 0.015) -> bool:
        return self.inverse(target, position_tolerance=position_tolerance).reachable
