import unittest
from behaviour_tree.behaviour_tree import BatteryStatus2bb, LaserScan2bb
import py_trees as pt


class TestBehaviorTree(unittest.TestCase):
    def test_battery_low(self):
        # Initialize BatteryStatus2bb
        battery = BatteryStatus2bb(threshold=20.0)

        # Mock blackboard data
        battery.blackboard.battery_level = 10.0  # Below threshold
        self.assertEqual(battery.update(), pt.common.Status.SUCCESS)

        # Verify the blackboard flag
        self.assertTrue(battery.blackboard.battery_low)

    def test_battery_normal(self):
        # Initialize BatteryStatus2bb
        battery = BatteryStatus2bb(threshold=20.0)

        # Mock blackboard data
        battery.blackboard.battery_level = 30.0  # Above threshold
        self.assertEqual(battery.update(), pt.common.Status.SUCCESS)

        # Verify the blackboard flag
        self.assertFalse(battery.blackboard.battery_low)

    def test_collision_warning(self):
        # Initialize LaserScan2bb
        laser = LaserScan2bb(safe_range=0.5)

        # Mock blackboard data
        laser.blackboard.laser_scan = [0.2, 0.3, 0.4]  # Within safe range
        self.assertEqual(laser.update(), pt.common.Status.SUCCESS)

        # Verify the blackboard flag
        self.assertTrue(laser.blackboard.collision_warning)

    def test_no_collision(self):
        # Initialize LaserScan2bb
        laser = LaserScan2bb(safe_range=0.5)

        # Mock blackboard data
        laser.blackboard.laser_scan = [0.6, 0.7, 0.8]  # Outside safe range
        self.assertEqual(laser.update(), pt.common.Status.SUCCESS)

        # Verify the blackboard flag
        self.assertFalse(laser.blackboard.collision_warning)


if __name__ == "__main__":
    unittest.main()
