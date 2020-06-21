#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
from datetime import datetime

import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

# Params
WPTS_GAP = 0.1 # [m]

def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Waypoint recorder init
now = datetime.now()
# filename = str(now.year) + "_" + str(now.month) + "_" + str(now.day) + "_" + str(now.hour) + "_" + str(now.minute) + ".csv"
filename = "test.csv"
WPT_CSV_PATH = "/home/seong/catkin_ws/src/waypoint_recorder/wpt_data/" + filename
if os.path.isfile(WPT_CSV_PATH):
    wpt_csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
else:
    wpt_csv_data = np.array([[]])
wpt_id = 0

# Parameters
ego_x = 0
ego_y = 0
ego_yaw = 0

def callback_odom(msg):
    # Get current pose
    global ego_x, ego_y, ego_yaw
    ego_x = msg.pose.pose.position.x
    ego_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    q_list = [q.x, q.y, q.z, q.w]
    _, _, ego_yaw = euler_from_quaternion(q_list)

# ROS init
rospy.init_node('wpt_recorder')
sub_odometry = rospy.Subscriber('/car_1/ground_truth', Odometry, callback_odom)
rate = rospy.Rate(100.0)

while not rospy.is_shutdown():

    # Waypoint recorder
    wpt_payload = np.array([[ego_x, ego_y, wpt_id]])
    dist_driven = 0

    if wpt_csv_data.shape[1] == 0: # if csv data is empty
        wpt_csv_data = np.append(wpt_csv_data, wpt_payload, axis=1)
        # update data
        last_ego_pose = [ego_x, ego_y]
        wpt_id += 1
    else:                          # if csv data is not empty
        # Append wpt data whenever agent drive 0.1 m from before
        dist_driven = calc_dist(last_ego_pose[0], last_ego_pose[1], ego_x, ego_y)
        if dist_driven < WPTS_GAP:
            pass
        else:
            wpt_csv_data = np.append(wpt_csv_data, wpt_payload, axis=0)
            # update data
            last_ego_pose = [ego_x, ego_y]
            wpt_id += 1

            # Save as csv
            dataframe = pd.DataFrame(wpt_csv_data)
            dataframe.to_csv(WPT_CSV_PATH, header=False, index=False)

    print("ego_x: {}, ego_y: {}, ego_yaw: {}".format(round(ego_x, 3), round(ego_y, 3), round(np.rad2deg(ego_yaw), 3)))
    rate.sleep()

