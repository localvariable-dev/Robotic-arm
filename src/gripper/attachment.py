"""Small simulator-independent grasp/release contract used by tests and adapters."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence
import math


class GraspError(RuntimeError):
    pass


class AttachmentBackend(Protocol):
    def candidate_distance(self, block_name: str) -> float: ...
    def attach(self, block_name: str) -> None: ...
    def release(self, block_name: str, position: Sequence[float]) -> None: ...


@dataclass
class AttachmentGripper:
    backend: AttachmentBackend
    max_grasp_distance: float = 0.085
    held_block: str | None = None

    def open(self) -> None:
        if self.held_block is not None:
            raise GraspError("Cannot open gripper while a block is attached; release it first")

    def close_and_grasp(self, block_name: str) -> bool:
        if self.held_block is not None:
            raise GraspError(f"Already holding {self.held_block}")
        distance = self.backend.candidate_distance(block_name)
        if not math.isfinite(distance) or distance > self.max_grasp_distance:
            return False
        self.backend.attach(block_name)
        self.held_block = block_name
        return True

    def release(self, position: Sequence[float]) -> str:
        if self.held_block is None:
            raise GraspError("Cannot release: no block is attached")
        name = self.held_block
        self.backend.release(name, position)
        self.held_block = None
        return name
