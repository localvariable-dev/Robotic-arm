"""Fail-safe state machine for a single pick-and-place task."""
from __future__ import annotations

from enum import Enum


class TaskState(str, Enum):
    IDLE = "IDLE"
    APPROACH = "APPROACH"
    PICK = "PICK"
    LIFT = "LIFT"
    TRANSFER = "TRANSFER"
    PLACE = "PLACE"
    RETRACT = "RETRACT"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


_ALLOWED = {
    TaskState.IDLE: {TaskState.APPROACH, TaskState.ERROR},
    TaskState.APPROACH: {TaskState.PICK, TaskState.ERROR},
    TaskState.PICK: {TaskState.LIFT, TaskState.ERROR},
    TaskState.LIFT: {TaskState.TRANSFER, TaskState.ERROR},
    TaskState.TRANSFER: {TaskState.PLACE, TaskState.ERROR},
    TaskState.PLACE: {TaskState.RETRACT, TaskState.ERROR},
    TaskState.RETRACT: {TaskState.APPROACH, TaskState.COMPLETE, TaskState.ERROR},
    TaskState.COMPLETE: set(),
    TaskState.ERROR: set(),
}


class StateTransitionError(RuntimeError):
    pass


class TaskStateMachine:
    def __init__(self) -> None:
        self.state = TaskState.IDLE
        self.history: list[TaskState] = [self.state]
        self.error: str | None = None

    def transition(self, next_state: TaskState) -> None:
        if next_state not in _ALLOWED[self.state]:
            raise StateTransitionError(f"Illegal state transition: {self.state.value} -> {next_state.value}")
        self.state = next_state
        self.history.append(next_state)

    def fail(self, reason: str) -> None:
        if self.state in (TaskState.COMPLETE, TaskState.ERROR):
            raise StateTransitionError(f"Cannot fail a terminal state ({self.state.value})")
        self.error = reason
        self.state = TaskState.ERROR
        self.history.append(self.state)
