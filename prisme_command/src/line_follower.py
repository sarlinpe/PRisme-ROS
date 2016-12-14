#!/usr/bin/env python
"""Line follower

ROS node for a line follower robot running in simulation using V-REP.
"""

import rospy
import message_filters
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Twist, Vector3

__author__ = "Karl Kangur"
__email__ = "karl.kangur@gmail.com"


def process_ir_under(ir_left, ir_right):
    """Follow the line using the IR sensors under the robot."""

    base_lin_speed = 0.12
    base_ang_speed = 0.4
    k_linear = 0.01
    k_angular = 0.015

    delta = ir_left.illuminance - ir_right.illuminance

    v_linear = base_lin_speed / (1 + k_linear*abs(delta))
    v_angular = -1 * base_ang_speed * k_angular * delta

    rospy.loginfo("Delta: %f,\tv_lin: %f, \tv_ang: %f", delta, v_linear, v_angular)

    # Never publish after the node has been shut down
    if not rospy.is_shutdown():
        # Actually publish the message
        set_speed(v_linear, v_angular)


def set_speed(linear, angular):
    # Set robot speed with a Twist topic
    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = linear
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular = Vector3()
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular

    # Actually publish the topic
    pub.publish(msg)


def stop_robot():
    rospy.loginfo("Stopping robot")
    set_speed(0, 0)


def initialize():
    """Initialize the sample node."""
    global pub
    namespace = "/prisme/"
    controller = "vel_controller/"

    # Provide a name for the node
    rospy.init_node("line_follower", anonymous=True)

    # Give some feedback in the terminal
    rospy.loginfo("Line follower node initialization")

    # Subscribe to and synchronise the infra-red sensors under the robot
    ir_under_left = message_filters.Subscriber(namespace+"ir_under_left", Illuminance)
    ir_under_right = message_filters.Subscriber(namespace+"ir_under_right", Illuminance)
    # Wait for all topics to arrive before calling the callback
    ts_ir_under = message_filters.ApproximateTimeSynchronizer([
        ir_under_left,
        ir_under_right], 1, 0.005)
    # Register the callback to be called when all sensor readings are ready
    ts_ir_under.registerCallback(process_ir_under)

    # Publish the linear and angular velocities so the robot can move
    pub = rospy.Publisher(namespace+controller+"cmd_vel", Twist, queue_size=1)

    # Register the callback for when the node is stopped
    rospy.on_shutdown(stop_robot)
    stop_robot()

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    initialize()
