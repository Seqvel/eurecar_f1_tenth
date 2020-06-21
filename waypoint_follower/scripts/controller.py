#!/usr/bin/env python
import numpy as np
import os
import pandas as pd

import roslib
import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import math
import tf
from tf.transformations import euler_from_quaternion

# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle (from 0 ~ +2pi to -pi ~ +pi)
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    # Translational transform
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    x_list = x_list - ego_x
    y_list = y_list - ego_y

    # Rotational transform
    rot_theta = -ego_yaw
    c_theta = np.cos(rot_theta)
    s_theta = np.sin(rot_theta)

    rot_mat = np.array([[c_theta, -s_theta],
                        [s_theta, c_theta]])

    output_xy_list = np.matmul(rot_mat, np.array([x_list, y_list]))
    output_x_list = output_xy_list[0,:]
    output_y_list = output_xy_list[1,:]

    return output_x_list, output_y_list

# Find nearest waypoint
def find_near_point(ego_x, ego_y, x_list, y_list):
    dist = np.zeros(len(x_list))
    for i in range(len(x_list)):
        dist[i] = calc_dist(x_list[i], y_list[i], ego_x, ego_y)
    
    near_ind = np.argmin(dist)
    near_dist = dist[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list):
    """
    TODO : Design Error calculation
        - Calculate errors to control
        - Crosstrack error(error_y), Heading angle error(error_yaw)
        - Calculate errors w.r.t. the nearest waypoint or the look-ahead waypoint if you want
        - Fill free to change below codes (Below codes is implemented about 'nearest waypoint')

    Reference paper
        - Automatic Steering Methods for Autonomous Automobile Path Tracking : https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

    """
    ##### TODO: Error calculation #####

    # Target waypoint (Nearest point or Look ahead point) ('%x_list.shape[0]' for loop driving)
    _, wpt_ind = find_near_point(ego_x, ego_y, wpts_x, wpts_y)
    target_wpt_ind = (wpt_ind)%x_list.shape[0]
    
    # Global to Local coordinate
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)
    
    # Calculate Crosstrack error
    error_y   = local_y_list[target_wpt_ind] # y value of the local path can be lateral error
    
    # Calculate Heading angle error
    error_yaw = math.atan2(local_y_list[(target_wpt_ind+1) % len(local_x_list)] - local_y_list[target_wpt_ind], \
                            local_x_list[(target_wpt_ind+1) % len(local_x_list)] - local_x_list[target_wpt_ind])
    error_yaw = normalize_angle(error_yaw)

    ###################################

    return error_y, error_yaw

# Controller
def steer_control(error_y, error_yaw):
    """
    TODO : Design Vehicle Lateral Control (Steering angle control)
        - Use crosstrack error(error_y), heading angle error(error_yaw), or more if you want
        - Can add derivative or integral of error values for PI, PID control
        - Min/Max angle is -30/+30 [deg]

    Reference paper
        - Automatic Steering Methods for Autonomous Automobile Path Tracking : https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

    """
    ##### TODO: Steer control #####
    kp_y = 0.2
    kp_yaw = 0.4
    
    steer = kp_y*error_y + kp_yaw*error_yaw
    #########################

    # Max/Min steer angle
    max_steer_l = np.deg2rad( 30)
    min_steer_r = np.deg2rad(-30)
    steer = max(min_steer_r, min(max_steer_l, steer))

    return steer

def velocity_profile(steer):
    """
    TODO : Design Vehicle Longitudinal Control (Velocity profile)
        - Constant velocity profile (simple case)
        - Variable velocity profile using some arguments (i.e. curvature of path)
    """
    
    ##### TODO: Velocity profile #####
    v_out = 1.0 # [m/s]

    ############################

    return v_out


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
rospy.init_node('wpt_foller')
listener = tf.TransformListener()
sub_odometry = rospy.Subscriber('/car_1/ground_truth', Odometry, callback_odom)
pub_command  = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=1)
rate = rospy.Rate(100.0)

# Load Waypoint
WPT_CSV_PATH = "/home/seong/catkin_ws/src/waypoint_recorder/wpt_data/test.csv"
csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
wpts_x = csv_data.values[:,0]
wpts_y = csv_data.values[:,1]
wpts_ind = csv_data.values[:,2]
wpts_length = wpts_x.shape[0]

while not rospy.is_shutdown():
    ##### TODO: Fill free to change below codes if you need #####

    # Error calculation
    error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y)

    # Control
    steer = steer_control(error_y, error_yaw)
    v_cmd = velocity_profile(steer)

    #############################################################

    # Publish command
    msg_cmd = AckermannDrive()
    msg_cmd = AckermannDrive()
    msg_cmd.speed = v_cmd
    msg_cmd.steering_angle = steer
    pub_command.publish(msg_cmd)

    print("Current Ego Pose. x: {}, y: {}, yaw: {}".format(\
        round(ego_x,3), round(ego_y,3), round(ego_yaw, 3)))

    rate.sleep()
