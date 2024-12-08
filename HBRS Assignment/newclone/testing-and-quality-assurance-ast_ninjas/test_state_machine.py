import unittest
from state_machine.state_machine import MonitorBattery, MonitorCollision, StopMovement
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy

class TestStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_state_machine")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_battery_low(self):
        battery_state = MonitorBattery(self.node, battery_threshold=20.0)
        battery_state.battery_callback(Float32(data=15.0))
        self.assertEqual(battery_state.execute(None), 'low_battery')

    def test_battery_normal(self):
        battery_state = MonitorBattery(self.node, battery_threshold=20.0)
        battery_state.battery_callback(Float32(data=25.0))
        self.assertEqual(battery_state.execute(None), 'normal')

    def test_collision_warning(self):
        collision_state = MonitorCollision(self.node, distance_threshold=0.5)
        collision_state.scan_callback(LaserScan(ranges=[0.4, 0.6, 0.8]))
        self.assertEqual(collision_state.execute(None), 'collision_warning')

    def test_no_collision(self):
        collision_state = MonitorCollision(self.node, distance_threshold=0.5)
        collision_state.scan_callback(LaserScan(ranges=[1.0, 1.2, 1.5]))
        self.assertEqual(collision_state.execute(None), 'clear')

if _name_ == '_main_':
    unittest.main()
