#!/usr/bin/env python

import rospy
import message_filters
import random
import time
import struct
from sensor_msgs.msg import Range, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header
from math import cos, sin, radians


ir_front_radius = 0.06
ir_z_offset = 0.01725
ir_angle1 = radians(17.5)
ir_angle2 = radians(42.5)

def process_ir_front(ir_left, ir_left_center, ir_right_center, ir_right):
    rospy.loginfo("Computing PointCloud.")

    header = Header()
    header.seq = 0
    header.stamp = rospy.Time.now()
    header.frame_id = "base"

    pts = []
    distances = [ir_left.range, ir_left_center.range, ir_right_center.range, ir_right.range]
    angles = [ir_angle2, ir_angle1, -ir_angle1, -ir_angle2]
    for i in range(0,4):
        pt = []
        pt.append((distances[i]+ir_front_radius)*cos(angles[i]))
        pt.append((distances[i]+ir_front_radius)*sin(angles[i]))
        pt.append(ir_z_offset)
        pts.append(pt)

    msg = pcl2.create_cloud_xyz32(header, pts)

    pub.publish(msg)


def initialize():
    global pub
    namespace = "/prisme/"

    # Provide a name for the node
    rospy.init_node("ir_point_cloud", anonymous=True)

    # Give some feedback in the terminal
    rospy.loginfo("Conversion of IR ranges into a point cloud.")

    # Subscribe to and synchronise the infra-red sensors in front of the robot
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


    # Publish the linear and angular velocities so the robot can move
    pub = rospy.Publisher("ir_front_pointcloud", PointCloud2, queue_size=1) # add namespace again

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    initialize()
