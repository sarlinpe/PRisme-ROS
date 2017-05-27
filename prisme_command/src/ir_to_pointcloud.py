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


''' Node parameters '''
namespace = "/prisme/"
ir_front_radius = 0.06
ir_z_offset = 0.01725
ir_angle1 = radians(17.5)
ir_angle2 = radians(42.5)


def ir_to_pointcloud(ir_left, ir_left_center, ir_right_center, ir_right):
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

    rospy.init_node("ir_to_pointcloud", anonymous=True)
    rospy.loginfo("Initialization: IR to Pointcloud converter node.")

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
    ts_ir_front.registerCallback(ir_to_pointcloud)

    pub = rospy.Publisher(namespace+"ir_front_pointcloud", PointCloud2, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    initialize()
