import py_trees as pt
import py_trees_ros as ptr
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

# Blackboard Behavior: Monitor Battery
class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, topic_name="battery_level", name="BatteryStatus", threshold=20.0):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=Float32,
            blackboard_variables={"battery_level": "data"},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
        )
        self.threshold = threshold
        self.blackboard.register_key(key="battery_low", access=pt.common.Access.WRITE)

    def update(self):
        # Check if battery data is valid
        if self.blackboard.battery_level is None:
            self.node.get_logger().warn("Battery sensor not responding.")
            return pt.common.Status.FAILURE

        # Update battery_low flag
        self.blackboard.battery_low = self.blackboard.battery_level < self.threshold
        return pt.common.Status.SUCCESS


# Blackboard Behavior: Monitor Collision
class LaserScan2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, topic_name="scan", name="LaserScan2bb", safe_range=0.5):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=LaserScan,
            blackboard_variables={"laser_scan": "ranges"},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
        )
        self.safe_range = safe_range
        self.blackboard.register_key(key="collision_warning", access=pt.common.Access.WRITE)

    def update(self):
        # Check if laser data is valid
        if not self.blackboard.laser_scan:
            self.node.get_logger().warn("Laser scan not responding.")
            return pt.common.Status.FAILURE

        # Update collision_warning flag
        self.blackboard.collision_warning = any(
            distance < self.safe_range for distance in self.blackboard.laser_scan if distance > 0
        )
        return pt.common.Status.SUCCESS


# Rotate Behavior
class Rotate(pt.behaviour.Behaviour):
    def __init__(self, name="Rotate"):
        super().__init__(name)
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        return True

    def update(self):
        if self.blackboard.battery_low and not self.blackboard.collision_warning:
            twist_msg = Twist()
            twist_msg.angular.z = 0.5  # Rotate the robot
            self.publisher.publish(twist_msg)
            return pt.common.Status.RUNNING
        else:
            twist_msg = Twist()  # Stop rotation
            self.publisher.publish(twist_msg)
            return pt.common.Status.SUCCESS


# Stop Motion Behavior
class StopMotion(pt.behaviour.Behaviour):
    def __init__(self, name="StopMotion"):
        super().__init__(name)
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        return True

    def update(self):
        if self.blackboard.collision_warning:
            twist_msg = Twist()  # Stop the robot
            self.publisher.publish(twist_msg)
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


# Idle Behavior
class Idle(pt.behaviour.Behaviour):
    def __init__(self, name="Idle"):
        super().__init__(name)

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        return True

    def update(self):
        return pt.common.Status.RUNNING


# Create the Behavior Tree
def create_root() -> pt.behaviour.Behaviour:
    root = pt.composites.Parallel(name="Root", policy=pt.common.ParallelPolicy.SuccessOnAll())

    # Blackboard behaviors
    topics2bb = pt.composites.Sequence(name="Topics2BB")
    battery_status = BatteryStatus2bb()
    laser_scan = LaserScan2bb()
    topics2bb.add_children([battery_status, laser_scan])

    # Decision-making behaviors
    selector = pt.composites.Selector(name="Selector")
    rotate = Rotate()
    stop_motion = StopMotion()
    idle = Idle()
    selector.add_children([rotate, stop_motion, idle])

    root.add_children([topics2bb, selector])
    return root


# Main Function
def main():
    rclpy.init()

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root)

    try:
        tree.setup(timeout=15)
        tree.tick_tock(period_ms=100)
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
