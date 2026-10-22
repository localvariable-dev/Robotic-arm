"""CoppeliaSim ZeroMQ remote API adapter.

Expected scene names are documented in config/simulation_config.json and the scene builder.
"""
from __future__ import annotations

import math
import time
from typing import Sequence

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:  # Importing pure modules/tests must not require CoppeliaSim client.
    RemoteAPIClient = None  # type: ignore[assignment,misc]


class SimulatorConnectionError(RuntimeError):
    pass


class MotionTimeoutError(TimeoutError):
    pass


class CoppeliaAdapter:
    def __init__(self, host: str = "localhost", port: int = 23000, timeout_s: float = 12.0) -> None:
        if RemoteAPIClient is None:
            raise SimulatorConnectionError("Install requirements.txt to use the CoppeliaSim remote API client")
        try:
            self.client = RemoteAPIClient(host=host, port=port)
            self.sim = self.client.require("sim")
        except Exception as exc:
            raise SimulatorConnectionError(f"Could not connect to CoppeliaSim at {host}:{port}: {exc}") from exc
        self.timeout_s = float(timeout_s)
        self.handles: dict[str, int] = {}

    def start(self) -> None:
        try:
            self.sim.startSimulation()
        except Exception as exc:
            # Some CoppeliaSim scenes may already be running.
            if "already" not in str(exc).lower():
                raise

    def stop(self) -> None:
        try:
            self.sim.stopSimulation()
        except Exception:
            pass

    def handle(self, name: str) -> int:
        if name not in self.handles:
            try:
                self.handles[name] = self.sim.getObject("/" + name)
            except Exception as exc:
                raise SimulatorConnectionError(f"Scene object {name!r} is missing: {exc}") from exc
        return self.handles[name]

    def _joint_handles(self) -> list[int]:
        return [self.handle(n) for n in ("joint_base", "joint_shoulder", "joint_elbow", "joint_wrist")]

    def _wait_joint_targets(self, targets: Sequence[float]) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            positions = [self.sim.getJointPosition(h) for h in self._joint_handles()]
            if all(abs((float(a)-float(b)+math.pi) % (2*math.pi)-math.pi) < 0.035 for a,b in zip(positions, targets)):
                return
            self.client.step()
        raise MotionTimeoutError(f"Timed out waiting for joint target {list(targets)}")

    def move_tcp(self, target_xyz: Sequence[float], wrist_pitch: float = 0.0) -> None:
        from src.kinematics.arm4dof import Arm4DOF, KinematicsError
        model = Arm4DOF()
        result = model.inverse(target_xyz, wrist_pitch=wrist_pitch)
        if not result.reachable:
            raise KinematicsError(f"Target {tuple(target_xyz)} is unreachable (error {result.position_error:.4f} m)")
        joints = self._joint_handles()
        for h, q in zip(joints, result.joints):
            self.sim.setJointTargetPosition(h, float(q))
        self._wait_joint_targets(result.joints)

    def open_gripper(self) -> None:
        self.sim.setJointTargetPosition(self.handle("gripper_left_joint"), 0.035)
        self.sim.setJointTargetPosition(self.handle("gripper_right_joint"), -0.035)
        self.client.step()

    def grasp(self, block_name: str) -> bool:
        left = self.handle("gripper_left_finger")
        right = self.handle("gripper_right_finger")
        block = self.handle(block_name)
        tip = self.handle("tool_center_point")
        tip_pos = self.sim.getObjectPosition(tip, -1)
        block_pos = self.sim.getObjectPosition(block, -1)
        distance = math.dist(tip_pos, block_pos)
        if distance > 0.09:
            self.open_gripper()
            return False
        self.sim.setJointTargetPosition(self.handle("gripper_left_joint"), 0.002)
        self.sim.setJointTargetPosition(self.handle("gripper_right_joint"), -0.002)
        self.client.step()
        # Parent the block to the TCP only after proximity success. Preserve its world pose.
        world_pos = self.sim.getObjectPosition(block, -1)
        world_ori = self.sim.getObjectOrientation(block, -1)
        self.sim.setObjectParent(block, tip, True)
        self.sim.setObjectPosition(block, -1, world_pos)
        self.sim.setObjectOrientation(block, -1, world_ori)
        return True

    def lift_held_block(self, target_xyz: Sequence[float]) -> None:
        self.move_tcp(target_xyz)

    def release(self, target_xyz: Sequence[float]) -> str:
        attached = []
        for name in ("block_red", "block_blue", "block_green", "block_yellow", "block_orange", "block_purple"):
            try:
                h = self.handle(name)
                if self.sim.getObjectParent(h) == self.handle("tool_center_point"):
                    attached.append(name)
            except Exception:
                continue
        if len(attached) != 1:
            raise RuntimeError(f"Expected exactly one attached block, found {attached}")
        name = attached[0]
        block = self.handle(name)
        self.sim.setObjectParent(block, self.handle("world"), True) if self._has_object("world") else self.sim.setObjectParent(block, -1, True)
        self.sim.setObjectPosition(block, -1, list(target_xyz))
        return name

    def _has_object(self, name: str) -> bool:
        try:
            self.handle(name)
            return True
        except SimulatorConnectionError:
            return False

    def verify_placed(self, block_name: str, target_xyz: Sequence[float], tolerance: float = 0.06) -> bool:
        block = self.handle(block_name)
        pos = self.sim.getObjectPosition(block, -1)
        return self.sim.getObjectParent(block) != self.handle("tool_center_point") and math.dist(pos, target_xyz) <= tolerance

    def verify_scene(self, expected_blocks: Sequence[str]) -> None:
        for name in (*expected_blocks, "tool_center_point", "gripper_left_joint", "gripper_right_joint"):
            self.handle(name)
