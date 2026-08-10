import importlib.util
import unittest


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "ROS 2 is not installed")
class RosImportTest(unittest.TestCase):
    def test_node_dependencies_import(self) -> None:
        from scratch_hci_bridge.node import ScratchHciBridge

        self.assertEqual(ScratchHciBridge.__name__, "ScratchHciBridge")


if __name__ == "__main__":
    unittest.main()
