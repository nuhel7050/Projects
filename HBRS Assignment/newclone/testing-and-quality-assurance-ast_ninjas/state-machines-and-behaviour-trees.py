#!/usr/bin/env python
# coding: utf-8

# #### Team members:
# * anuhel2s
# * mkhan2s

# # Robot Behaviour Management Using State Machines and Behaviour Trees
# 
# In this assignment, we will compare the implementation of behaviour trees and state machines to establish primary safety features for a robot; this includes situations such as the battery level falling below a certain threshold and avoiding potential collisions. In other words, we want to implement both a behaviour tree and a state machine to achieve the same functionality.
# 
# We particularly want the robot to behave as follows:
# * *The battery level falling below a given threshold*: The robot starts rotating in place until the level is above the threshold again (you can control the battery level by publishing to a topic).
# * *A collision is about to happen*: The robot stops moving and needs to be moved to a safe distance manually.
# 
# You will use the Robile simulation for testing your implementation. For your submission, you need to add your code to the appropriate cells below; however, note that, to actually test your implementation, you need to integrate the code in a local ROS package and perform all tests on your local machine.

# ## Robot Safety Functionalities Using a State Machine [45 points]
# 
# To implement a state machine, we will use the [SMACH framework](https://wiki.ros.org/smach/Tutorials). SMACH as such is ROS independent, but `executive_smach` provides a ROS integration, which we will be using in this assignment.
# 
# Your task here is to implement the necessary states and set up the state machine to achieve the desired robot functionality.

# In[ ]:


# Import necessary libraries
import rclpy
import smach
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Define the MonitorBattery state
class MonitorBattery(smach.State):
    def __init__(self, node, battery_threshold):
        smach.State.__init__(self, outcomes=['low_battery', 'normal'])
        self.node = node
        self.battery_threshold = battery_threshold
        self.battery_level = None

        # Subscribe to battery level updates
        self.subscription = self.node.create_subscription(
            Float32,
            'battery_level',
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        """Updates battery level based on messages."""
        self.battery_level = msg.data

    def execute(self, userdata):
        # Log once upon entry
        self.node.get_logger().info('Entering MonitorBattery state.')

        # Wait for a valid battery level before deciding on transition
        while self.battery_level is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        if self.battery_level < self.battery_threshold:
            self.node.get_logger().info('Battery level low, transitioning to RotateInPlace.')
            return 'low_battery'
        else:
            self.node.get_logger().info('Battery level normal, transitioning to MonitorCollision.')
            return 'normal'

# Define the RotateInPlace state
class RotateInPlace(smach.State):
    def __init__(self, node, battery_threshold):
        smach.State.__init__(self, outcomes=['stop_rotation'])
        self.node = node
        self.battery_threshold = battery_threshold
        self.battery_level = None
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to battery level topic
        self.subscription = self.node.create_subscription(
            Float32,
            'battery_level',
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        """Updates battery level based on messages."""
        self.battery_level = msg.data

    def execute(self, userdata):
        # Log once upon entry
        self.node.get_logger().info('Entering RotateInPlace state.')

        # Reset battery level to ensure fresh data on each execution
        self.battery_level = None

        # Wait until a valid battery level is received
        while self.battery_level is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        # Rotate if the battery is low
        if self.battery_level < self.battery_threshold:
            twist = Twist()
            twist.angular.z = 0.5
            self.publisher.publish(twist)
            self.node.get_logger().info('Rotating in place due to low battery.')

        # Monitor battery level and stop rotation if it is restored
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.battery_level >= self.battery_threshold:
                twist = Twist()  # Publish zero velocity to stop rotation
                self.publisher.publish(twist)
                self.node.get_logger().info('Battery level restored. Stopping rotation and transitioning to MonitorBattery.')
                return 'stop_rotation'

# Define the MonitorCollision state
class MonitorCollision(smach.State):
    def __init__(self, node, distance_threshold):
        smach.State.__init__(self, outcomes=['collision_warning', 'clear'])
        self.node = node
        self.distance_threshold = distance_threshold
        self.collision_warning = False

        # Subscribe to laser scan topic for obstacle detection
        self.subscription = self.node.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Check if ranges are received and process them
        if msg.ranges:
            min_distance = min(msg.ranges)

            # Set collision warning if an obstacle is within the threshold
            if min_distance < self.distance_threshold:
                self.collision_warning = True
            else:
                self.collision_warning = False
        else:
            self.node.get_logger().warn('No range data received in LaserScan message.')

    def execute(self, userdata):
        # Log once upon entry
        self.node.get_logger().info('Entering MonitorCollision state.')

        # Process incoming LaserScan messages
        rclpy.spin_once(self.node, timeout_sec=1.0)

        # Decide on transition based on collision_warning status
        if self.collision_warning:
            self.node.get_logger().info('Transitioning to StopMovement due to obstacle.')
            return 'collision_warning'
        else:
            self.node.get_logger().info('Path is clear. No obstacles detected, transitioning to MonitorBattery.')
            return 'clear'

# Define the StopMovement state
class StopMovement(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['stopped'])
        self.node = node
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def execute(self, userdata):
        # Log once upon entry
        self.node.get_logger().info('Entering StopMovement state.')

        # Publish a zero velocity command to stop the robot
        twist = Twist()  # Default Twist has all zeros, which stops movement
        self.publisher.publish(twist)
        self.node.get_logger().info('Robot is stopping due to collision warning.')

        # Allow some time for the stop command to take effect
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Transitioning back to MonitorCollision to continue monitoring.')
        return 'stopped'

# Define the main state machine setup
def main():
    rclpy.init()
    node = rclpy.create_node('robot_safety_state_machine')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    battery_threshold = 20.0
    distance_threshold = 0.5

    with sm:
        # Define states and transitions with logging for transitions
        smach.StateMachine.add('MONITOR_BATTERY', MonitorBattery(node, battery_threshold),
                               transitions={'low_battery': 'ROTATE_IN_PLACE',
                                            'normal': 'MONITOR_COLLISION'})

        smach.StateMachine.add('ROTATE_IN_PLACE', RotateInPlace(node, battery_threshold),
                               transitions={'stop_rotation': 'MONITOR_BATTERY'})

        smach.StateMachine.add('MONITOR_COLLISION', MonitorCollision(node, distance_threshold),
                               transitions={'collision_warning': 'STOP_MOVEMENT',
                                            'clear': 'MONITOR_BATTERY'})

        smach.StateMachine.add('STOP_MOVEMENT', StopMovement(node),
                               transitions={'stopped': 'MONITOR_COLLISION'})

    node.get_logger().info("Starting state machine execution.")
    outcome = sm.execute()
    node.get_logger().info(f"State machine finished with outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()

# Execute the state machine
if __name__ == '__main__':
    main()


# ## Robot Safety Functionalities Using a Behaviour Tree [45 points]
# 
# The majority of implementations of behaviour trees in robotics are using `BehaviorTree.CPP` in cpp and [py_trees](https://py-trees.readthedocs.io/en/devel/) in Python. [py_trees_ros](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.1.x/tutorials.html) is a wrapper for `py_trees` to integrate it with ROS, which we will use in this assignment.
# 
# Your task here is to implement the necessary behaviours and set up the behaviour tree to achieve the desired robot functionality.

# Implement the required behaviours in the cell below. [35 points]

# In[ ]:


import py_trees as pt
import py_trees_ros as ptr
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, topic_name="/battery_voltage", name="BatteryStatus", threshold=30.0):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=Float32,
            blackboard_variables={'battery_level': 'data'},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
        )
        self.threshold = threshold
        self.blackboard.register_key(key='battery_low', access=pt.common.Access.WRITE)

    def update(self):
        # Check if battery data is valid
        if self.blackboard.battery_level is None:
            self.node.get_logger().error("Battery sensor not responding.")
            return pt.common.Status.FAILURE  # Sensor failure

        # Update battery_low flag
        self.blackboard.battery_low = self.blackboard.battery_level < self.threshold
        return pt.common.Status.SUCCESS  # Sensor is working


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, topic_name="/scan", name="LaserScan2bb", safe_range=0.5):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=LaserScan,
            blackboard_variables={'laser_scan': 'ranges'},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
        )
        self.safe_range = safe_range
        self.blackboard.register_key(key='collision_warning', access=pt.common.Access.WRITE)

    def update(self):
        # Check if laser data is valid
        if not self.blackboard.laser_scan:
            self.node.get_logger().error("Laser sensor not responding.")
            return pt.common.Status.FAILURE  # Sensor failure

        # Update collision_warning flag
        self.blackboard.collision_warning = any(
            dist < self.safe_range for dist in self.blackboard.laser_scan if dist > 0
        )
        return pt.common.Status.SUCCESS  # Sensor is working


class Rotate(pt.behaviour.Behaviour):
    def __init__(self, name="Rotate"):
        super().__init__(name)
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
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


class StopMotion(pt.behaviour.Behaviour):
    def __init__(self, name="StopMotion"):
        super().__init__(name)
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        return True

    def update(self):
        if self.blackboard.collision_warning:
            twist_msg = Twist()
            self.publisher.publish(twist_msg)  # Stop the robot
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class Idle(pt.behaviour.Behaviour):
    def __init__(self, name="Idle"):
        super().__init__(name)
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, timeout):
        self.node = rclpy.create_node(self.name)
        return True

    def update(self):
        return pt.common.Status.RUNNING
    




# Now, set up and initialise your behaviour tree in the cell below. [10 points]

# In[ ]:


### Implement a behaviour tree using your previously implemented behaviours here

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys

def create_root() -> pt.behaviour.Behaviour:
    root = pt.composites.Parallel(name="Root", policy=pt.common.ParallelPolicy.SuccessOnAll())

    topics2bb = pt.composites.Sequence(name="Topics2BB")
    battery_status = BatteryStatus2bb()
    laser_scan = LaserScan2bb()
    topics2bb.add_children([battery_status, laser_scan])

    selector = pt.composites.Selector(name="Selector")
    rotate = Rotate()
    stop_motion = StopMotion()
    idle = Idle()
    selector.add_children([rotate, stop_motion, idle])

    root.add_children([topics2bb, selector])
    return root


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


# ## Setting up Your System for Testing
# 
# 1. Please setup Ubuntu 22.04, ROS2 humble, and the Robile simulation (if not already done) by following the [documentation](https://robile-amr.readthedocs.io/en/humble/).
# 
# 2. Clone the `executive_smach` repository in the src folder of your workspace, and also install the `py-trees-ros` package:
# ```
# cd ~/ros2_ws/src/  
# git clone -b ros2 https://github.com/ros/executive_smach.git  
# sudo apt-get install ros-humble-py-trees ros-humble-py-trees-ros ros-humble-py-trees-ros-interfaces xcb
# ```
# 
# From the workspace directory, build the workspace:
# ```
# cd ~/ros2_ws/
# colcon build --symlink-install
# ```
# 
# Now source the workspace setup file:
# ```
# source install/setup.bash
# ```
# 3. Create a new ROS2 python package and integrate your implementation in it
# 
# ## Testing Instructions
# 
# Use the following steps to test your implementation:
# - Run the Robile in simulation
# - After integrating your implementation in your local ROS workspace, launch your components to test the functionalities. Note that you need to test the state machine and behaviour tree implementations independently to verify that the robot behaves exactly the same in both cases.
# 
# **In your submission, please include screenshots to verify your implementation.**
# 
# Note that, as the battery percentage is not readily available in simulation, please publish the battery percentage values manually. For instance, if the topic `/battery_voltage` is used for monitoring the battery status, you should be able to publish a battery percentage value of your choice to verify your implementation, e.g.:
# ```  
# ros2 topic  pub /battery_voltage std_msgs/msg/Float32  data:\ 50.0\ 
# ```
# 
# Finally, behaviour tree visualization is not released on ROS2 humble yet, but the changes in the behaviour tree can be monitored by running the following command, which is helpful for debugging:
# ```
# py-trees-tree-watcher
# ```
# 
# The following is a sample visualisation when the robot is about to collide:
# 
# ![collision avoidance BT](figures/BT_watcher.png)
# 
# For getting a better intuition of behaviour trees, another sample visualisation of a similar task, but with a slightly different structure can be found below:
# 
# ![collison and battery low](figures/collision_battery.png)

# **Discuss any observations from your tests and include screenshots that verify your implementation in the cell below. [10 points]**

# Here are few of the observations that we found from the experiment:
# 1.Setting up and initializing ROS nodes is necessary in both approaches. With state machines, this was straightforward using the constructor, while behavior trees required a more complex setup.
# 2.Behavior trees respond more effectively to dynamic changes, while state machines relied on layered if-else conditions to manage changing conditions.
# 3.State machines made the debugging process easier.

# 
