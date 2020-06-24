#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
from datetime import datetime

import roslib
import rospkg
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

# Load waypoint path
r = rospkg.RosPack()
WPT_CSV_PATH = r.get_path('waypoint_recorder') + "/wpt_data/test.csv"

# Load wpt file
try:
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
except:
    print("Assign appropriate path of waypoint csv file!")

wpts_x = csv_data.values[:,0]
wpts_y = csv_data.values[:,1]
wpts_length = len(wpts_x)

# ROS init
rospy.init_node('wpt_loader')
marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)
listener = tf.TransformListener()
rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    # Publish wpt as MarkerArray
    msg_wpts = Marker()
    msg_wpts.header.frame_id= "/map"
    msg_wpts.header.stamp= rospy.Time.now()
    msg_wpts.ns= "spheres"
    msg_wpts.action= Marker.ADD
    msg_wpts.pose.orientation.w= 1.0

    msg_wpts.id = 0
    msg_wpts.type = Marker.SPHERE_LIST


    # POINTS markers use x and y scale for width/height respectively
    msg_wpts.scale.x = 0.1
    msg_wpts.scale.y = 0.1
    msg_wpts.scale.z = 0.1

    # Points are green
    msg_wpts.color.a = 1.0; # Don't forget to set the alpha!
    msg_wpts.color.r = 176/255.;
    msg_wpts.color.g = 60/255.;
    msg_wpts.color.b = 193/255.;

    msg_point_list = []

    for i in range(wpts_length):
        msg_point = Point()
        msg_point.x = wpts_x[i]
        msg_point.y = wpts_y[i]
        msg_point_list.append(msg_point)

    msg_wpts.points = msg_point_list
    marker_pub.publish(msg_wpts)
