"""CoppeliaSim ZeroMQ remote API adapter for the Phase 1 arm."""
from __future__ import annotations

import math
import time
from typing import Sequence

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:  # Keep pure-Python tests independent of CoppeliaSim.
    RemoteAPIClient = None  # type: ignore[assignment,misc]


class SimulatorConnectionError(RuntimeError):
    pass


class MotionTimeoutError(TimeoutError):
    pass


class CoppeliaAdapter:
    def __init__(
        self, host: str = "localhost", port: int = 23000, timeout_s: float = 12.0,
        base_world: Sequence[float] = (0.0, 0.0, 1.03),
        tool_offset: Sequence[float] = (0.0, 0.0, 0.08),
    ) -> None:
        if RemoteAPIClient is None:
            raise SimulatorConnectionError("Install dependencies with: python -m pip install -r requirements.txt")
        try:
            self.client = RemoteAPIClient(host=host, port=port)
            self.client.setStepping(True)
            self.sim = self.client.require("sim")
        except Exception as exc:
            raise SimulatorConnectionError(f"Could not connect to CoppeliaSim at {host}:{port}: {exc}") from exc
        self.timeout_s = float(timeout_s)
        self.base_world = tuple(map(float, base_world))
        self.tool_offset = tuple(map(float, tool_offset))
        if len(self.base_world) != 3 or len(self.tool_offset) != 3:
            raise ValueError("base_world and tool_offset must be XYZ triples")
        self.handles: dict[str, int] = {}

    def start(self) -> None:
        try:
            self.sim.startSimulation()
            self.client.step()
        except Exception as exc:
            if "already" not in str(exc).lower():
                raise SimulatorConnectionError(f"Could not start simulation: {exc}") from exc

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

    def _arm_handles(self) -> list[int]:
        return [self.handle(n) for n in ("joint_base", "joint_shoulder", "joint_elbow", "joint_wrist")]

    def _wait_positions(self, handles: Sequence[int], targets: Sequence[float], label: str) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            positions = [float(self.sim.getJointPosition(h)) for h in handles]
            if all(abs((a - b + math.pi) % (2 * math.pi) - math.pi) < 0.04 for a, b in zip(positions, targets)):
                return
            self.client.step()
        raise MotionTimeoutError(f"Timed out waiting for {label}: targets={list(targets)}")

    def move_tcp(self, target_xyz: Sequence[float], wrist_pitch: float = 0.0) -> None:
        """Move TCP to target block-frame XYZ plus the configured vertical tool offset."""
        from src.kinematics.arm4dof import Arm4DOF, KinematicsError

        if len(target_xyz) != 3:
            raise ValueError("target_xyz must be an XYZ triple")
        tcp_target = tuple(float(v) + self.tool_offset[i] for i, v in enumerate(target_xyz))
        model = Arm4DOF(base_world=self.base_world)
        result = model.inverse(tcp_target, wrist_pitch=wrist_pitch)
        if not result.reachable:
            raise KinematicsError(
                f"TCP target {tcp_target} is unreachable (position error {result.position_error:.4f} m)"
            )
        handles = self._arm_handles()
        for h, q in zip(handles, result.joints):
            self.sim.setJointTargetPosition(h, float(q))
        self._wait_positions(handles, result.joints, "arm joints")

    def open_gripper(self) -> None:
        left, right = self.handle("gripper_left_joint"), self.handle("gripper_right_joint")
        self.sim.setJointTargetPosition(left, 0.0)
        self.sim.setJointTargetPosition(right, 0.0)
        self._wait_positions((left, right), (0.0, 0.0), "open gripper")

    def grasp(self, block_name: str) -> bool:
        block = self.handle(block_name)
        tcp = self.handle("tool_center_point")
        tcp_pos = self.sim.getObjectPosition(tcp, -1)
        block_pos = self.sim.getObjectPosition(block, -1)
        distance = math.dist(tcp_pos, block_pos)
        if distance > 0.09:
            self.open_gripper()
            return False

        left, right = self.handle("gripper_left_joint"), self.handle("gripper_right_joint")
        self.sim.setJointTargetPosition(left, 0.033)
        self.sim.setJointTargetPosition(right, -0.033)
        self._wait_positions((left, right), (0.033, -0.033), "close gripper")

        # Demonstration-grade attachment: attach only after the proximity/gripper command succeeds.
        world_pos = self.sim.getObjectPosition(block, -1)
        world_ori = self.sim.getObjectOrientation(block, -1)
        self.sim.setObjectParent(block, tcp, True)
        self.sim.setObjectPosition(block, -1, world_pos)
        self.sim.setObjectOrientation(block, -1, world_ori)
        return True

    def lift_held_block(self, target_xyz: Sequence[float]) -> None:
        self.move_tcp(target_xyz)

    def release(self, target_xyz: Sequence[float]) -> str:
        blocks = ("block_red", "block_blue", "block_green", "block_yellow", "block_orange", "block_purple")
        tcp = self.handle("tool_center_point")
        attached = []
        for name in blocks:
            try:
                h = self.handle(name)
                if self.sim.getObjectParent(h) == tcp:
                    attached.append(name)
            except Exception:
                continue
        if len(attached) != 1:
            raise RuntimeError(f"Expected exactly one attached block, found {attached}")
        name = attached[0]
        block = self.handle(name)
        self.sim.setObjectParent(block, -1, True)
        self.sim.setObjectPosition(block, -1, [float(v) for v in target_xyz])
        self.sim.setObjectInt32Param(block, self.sim.shapeintparam_static, 0)
        return name

    def verify_placed(self, block_name: str, target_xyz: Sequence[float], tolerance: float = 0.06) -> bool:
        block = self.handle(block_name)
        pos = self.sim.getObjectPosition(block, -1)
        tcp = self.handle("tool_center_point")
        return self.sim.getObjectParent(block) != tcp and math.dist(pos, target_xyz) <= tolerance

    def verify_scene(self, expected_blocks: Sequence[str]) -> None:
        for name in (*expected_blocks, "tool_center_point", "gripper_left_joint",
                     "gripper_right_joint", "joint_base", "joint_shoulder", "joint_elbow", "joint_wrist"):
            self.handle(name)
