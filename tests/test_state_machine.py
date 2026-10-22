import unittest

from src.controller.state_machine import TaskState, TaskStateMachine, StateTransitionError


class StateMachineTests(unittest.TestCase):
    def test_valid_sequence_and_terminal_state(self):
        m = TaskStateMachine()
        for s in (TaskState.APPROACH, TaskState.PICK, TaskState.LIFT, TaskState.TRANSFER,
                  TaskState.PLACE, TaskState.RETRACT, TaskState.COMPLETE):
            m.transition(s)
        self.assertEqual(m.state, TaskState.COMPLETE)
        with self.assertRaises(StateTransitionError):
            m.transition(TaskState.APPROACH)

    def test_illegal_transition_rejected(self):
        m = TaskStateMachine()
        with self.assertRaises(StateTransitionError):
            m.transition(TaskState.PLACE)

    def test_failure_is_terminal(self):
        m = TaskStateMachine()
        m.fail("test")
        self.assertEqual(m.state, TaskState.ERROR)
        self.assertEqual(m.error, "test")


if __name__ == "__main__":
    unittest.main()
