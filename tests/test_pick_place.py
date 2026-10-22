import unittest

from src.task_manager.pick_place import PickPlaceTask


class FakeAdapter:
    def __init__(self, fail_grasp=None):
        self.held = None
        self.placed = {}
        self.fail_grasp = fail_grasp
        self.moves = []

    def move_tcp(self, target_xyz, wrist_pitch=0.0):
        self.moves.append(tuple(target_xyz))

    def open_gripper(self):
        if self.held:
            raise AssertionError("should release before opening")

    def grasp(self, block_name):
        if block_name == self.fail_grasp:
            return False
        self.held = block_name
        return True

    def lift_held_block(self, target_xyz):
        self.move_tcp(target_xyz)

    def release(self, target_xyz):
        if not self.held:
            raise RuntimeError("nothing held")
        name, self.held = self.held, None
        self.placed[name] = tuple(target_xyz)
        return name

    def verify_placed(self, block_name, target_xyz, tolerance=0.06):
        return self.placed.get(block_name) == tuple(target_xyz)


class PickPlaceTests(unittest.TestCase):
    def make_task(self, adapter):
        names = ["red", "blue", "green", "yellow", "orange", "purple"]
        src = [(0.4, -0.2, 0.9+i*0.001) for i in range(6)]
        dst = [(0.4, 0.2, 0.9+i*0.001) for i in range(6)]
        return PickPlaceTask(adapter, names, src, dst)

    def test_all_six_sequentially(self):
        adapter = FakeAdapter()
        result = self.make_task(adapter).run()
        self.assertTrue(result.success, result.error)
        self.assertEqual(len(result.placed), 6)
        self.assertEqual(len(set(result.placed)), 6)

    def test_failed_grasp_stops_without_success(self):
        adapter = FakeAdapter(fail_grasp="green")
        result = self.make_task(adapter).run()
        self.assertFalse(result.success)
        self.assertEqual(result.failed_block, "green")
        self.assertEqual(result.placed, ["red", "blue"])

    def test_duplicate_block_names_rejected(self):
        adapter = FakeAdapter()
        with self.assertRaises(ValueError):
            PickPlaceTask(adapter, ["red", "red"], [(0,0,0)]*2, [(0,1,0)]*2)


if __name__ == "__main__":
    unittest.main()
