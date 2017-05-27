#!/usr/bin/env python

import rospy
import message_filters
import random
import time
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Vector3
from math import pow


''' Node parameters '''
namespace = "/prisme/"
controller = "vel_controller/"

base_lin_speed = 0.12
base_ang_speed = 0.4

alpha = 0.5
k_linear = 200
k_angular = 100
max_distance = 0.1

recovery_thresh_delta = max_distance / 4
recovery_thresh_dist = max_distance / 2
recovery_time = 1


def process_ir_front(ir_left, ir_left_center, ir_right_center, ir_right):
    distances = [ir_left.range, ir_left_center.range, ir_right_center.range, ir_right.range]
    delta_center = distances[1] - distances[2]
    delta_side = distances[0] - distances[3]
    recovery = False

    ''' Control on linear and angular velocities '''
    delta = (1-alpha)*delta_side + alpha*delta_center
    v_linear = base_lin_speed / (1 + pow(k_linear*delta,2))
    v_angular = base_ang_speed * k_angular * delta

    ''' Recovery mode '''
    if abs(delta) < recovery_thresh_dist and min(distances) < recovery_thresh_delta:
        v_linear = 0
        v_angular = base_ang_speed
        recovery = True

    rospy.loginfo("Delta: %f,\tv_lin: %f, \tv_ang: %f", delta, v_linear, v_angular)
    if not rospy.is_shutdown():
        set_speed(v_linear, v_angular)
    if recovery:
        time.sleep(recovery_time * random.random())


def set_speed(linear, angular):
    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = linear
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular = Vector3()
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular

    pub.publish(msg)


def stop_robot():
    rospy.loginfo("Stopping robot")
    set_speed(0, 0)


def initialize():
    global pub

    rospy.init_node("obstacle_avoidance", anonymous=True)
    rospy.loginfo("Initialization: obstacles avoidance node.")

    # IR sensors require synchronisation
    ir_front_left = message_filters.Subscriber(namespace+"ir_front_left", Range)
    ir_front_right = message_filters.Subscriber(namespace+"ir_front_right", Range)
    ir_front_left_center = message_filters.Subscriber(namespace+"ir_front_left_center", Range)
    ir_front_right_center = message_filters.Subscriber(namespace+"ir_front_right_center", Range)
    # Wait for all topics to arrive before calling the callback
    ts_ir_front = message_filters.TimeSynchronizer([
        ir_front_left,
        ir_front_left_center,
        ir_front_right_center,
        ir_front_right], 1)
    # Register the callback to be called when all sensor readings are ready
    ts_ir_front.registerCallback(process_ir_front)

    pub = rospy.Publisher(namespace+controller+"cmd_vel", Twist, queue_size=1)
    rospy.on_shutdown(stop_robot)
    rospy.spin()


if __name__ == "__main__":
    initialize()
