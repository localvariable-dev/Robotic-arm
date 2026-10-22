"""Sequential pick/place orchestration over an injected simulator adapter."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence

from src.controller.state_machine import TaskState, TaskStateMachine


class PickPlaceAdapter(Protocol):
    def move_tcp(self, target_xyz: Sequence[float], wrist_pitch: float = 0.0) -> None: ...
    def open_gripper(self) -> None: ...
    def grasp(self, block_name: str) -> bool: ...
    def lift_held_block(self, target_xyz: Sequence[float]) -> None: ...
    def release(self, target_xyz: Sequence[float]) -> str: ...
    def verify_placed(self, block_name: str, target_xyz: Sequence[float], tolerance: float = 0.06) -> bool: ...


class TaskExecutionError(RuntimeError):
    pass


@dataclass
class TaskResult:
    placed: list[str] = field(default_factory=list)
    failed_block: str | None = None
    error: str | None = None
    state_history: list[str] = field(default_factory=list)

    @property
    def success(self) -> bool:
        return self.failed_block is None and self.error is None


class PickPlaceTask:
    def __init__(
        self,
        adapter: PickPlaceAdapter,
        blocks: Sequence[str],
        source_positions: Sequence[Sequence[float]],
        destination_positions: Sequence[Sequence[float]],
        approach_height: float = 0.12,
        safe_transfer_height: float = 0.20,
        place_height: float = 0.02,
    ) -> None:
        if not (len(blocks) == len(source_positions) == len(destination_positions)):
            raise ValueError("blocks and source/destination positions must have equal lengths")
        if len(set(blocks)) != len(blocks):
            raise ValueError("block names must be unique to prevent duplicate processing")
        if min(approach_height, safe_transfer_height, place_height) < 0:
            raise ValueError("height offsets must be non-negative")
        self.adapter = adapter
        self.blocks = tuple(blocks)
        self.sources = tuple(tuple(map(float, p)) for p in source_positions)
        self.destinations = tuple(tuple(map(float, p)) for p in destination_positions)
        if any(len(p) != 3 for p in (*self.sources, *self.destinations)):
            raise ValueError("all positions must be XYZ triples")
        self.approach_height = float(approach_height)
        self.safe_transfer_height = float(safe_transfer_height)
        self.place_height = float(place_height)
        self.processed: set[str] = set()
        self.machine = TaskStateMachine()

    @staticmethod
    def _offset(position: Sequence[float], dz: float) -> tuple[float, float, float]:
        return (float(position[0]), float(position[1]), float(position[2]) + dz)

    def run(self) -> TaskResult:
        result = TaskResult()
        try:
            if not self.blocks:
                self.machine.transition(TaskState.COMPLETE)
                result.state_history = [s.value for s in self.machine.history]
                return result
            for index, name in enumerate(self.blocks):
                if name in self.processed:
                    raise TaskExecutionError(f"Refusing to process {name} twice")
                source, destination = self.sources[index], self.destinations[index]
                self.machine.transition(TaskState.APPROACH)
                self.adapter.open_gripper()
                self.adapter.move_tcp(self._offset(source, self.approach_height))
                self.adapter.move_tcp(source)
                self.machine.transition(TaskState.PICK)
                if not self.adapter.grasp(name):
                    raise TaskExecutionError(f"Grasp verification failed for {name}")
                self.machine.transition(TaskState.LIFT)
                self.adapter.lift_held_block(self._offset(source, self.safe_transfer_height))
                self.machine.transition(TaskState.TRANSFER)
                self.adapter.move_tcp(self._offset(destination, self.safe_transfer_height))
                self.machine.transition(TaskState.PLACE)
                self.adapter.move_tcp(self._offset(destination, self.place_height))
                released = self.adapter.release(self._offset(destination, self.place_height))
                if released != name:
                    raise TaskExecutionError(f"Release mismatch: expected {name}, got {released}")
                if not self.adapter.verify_placed(name, self._offset(destination, self.place_height)):
                    raise TaskExecutionError(f"Placement verification failed for {name}")
                self.processed.add(name)
                result.placed.append(name)
                self.machine.transition(TaskState.RETRACT)
                self.adapter.move_tcp(self._offset(destination, self.safe_transfer_height))
            self.machine.transition(TaskState.COMPLETE)
        except Exception as exc:
            result.failed_block = self.blocks[len(result.placed)] if len(result.placed) < len(self.blocks) else None
            result.error = str(exc)
            if self.machine.state not in (TaskState.ERROR, TaskState.COMPLETE):
                self.machine.fail(str(exc))
        result.state_history = [s.value for s in self.machine.history]
        return result
