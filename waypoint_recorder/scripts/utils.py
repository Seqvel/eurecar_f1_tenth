#!/usr/bin/env python
import numpy as np
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('map_tf_listener')
listener = tf.TransformListener()
rate = rospy.Rate(100.0)

while not rospy.is_shutdown():

    # Get current pose
    try:
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("No tf")
        continue

    x, y, _ = trans
    _, _, yaw = euler_from_quaternion(rot)
    
    print("x: {}, y: {}, yaw: {}".format(round(x, 3), round(y, 3), round(np.rad2deg(yaw), 3)))

    rate.sleep()