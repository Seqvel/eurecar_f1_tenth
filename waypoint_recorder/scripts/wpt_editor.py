#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
from scipy import interpolate

import matplotlib.pyplot as plt
from datetime import datetime
import time

import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

SAVE_DATA = True

WPT_CSV_LOAD_PATH = "/home/seong/jetson_ws/src/waypoint_recorder/wpt_data/2020_5_1_18_3.csv"
WPT_CSV_SAVE_PATH = "/home/seong/jetson_ws/src/waypoint_recorder/wpt_data/2020_5_1_18_3_edit.csv"

# ----- Load wpt file
csv_data = pd.read_csv(WPT_CSV_LOAD_PATH, sep=',', header=None)
wpts_data = csv_data.values
wpts_x = wpts_data[:,0]
wpts_y = wpts_data[:,1]
wpts_i = wpts_data[:,2]
wpts_length = len(wpts_x)
print("wpts_length :", wpts_length)

# ----- Spline Interpolation
N = 1000
wpts_x_ = np.r_[wpts_x, wpts_x[0]] # append the starting x,y coordinates
wpts_y_ = np.r_[wpts_y, wpts_y[0]]

tck, u = interpolate.splprep([wpts_x_, wpts_y_], s=1, per=True)
# evaluate the spline fits for N evenly spaced distance values
wpts_xi_, wpts_yi_ = interpolate.splev(np.linspace(0, 1, N), tck)

# ----- Curvature
tic = time.time()

dx_dt = np.gradient(wpts_xi_)
dy_dt = np.gradient(wpts_yi_)
ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt)

d2s_dt2 = np.gradient(ds_dt)
d2x_dt2 = np.gradient(dx_dt)
d2y_dt2 = np.gradient(dy_dt)

wpts_curv_ = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5

# ----- Update data
wpts_ii_ = np.linspace(0, N-1, N)

wpts_data_new = np.zeros([N, 4]) # [x, y, index, curvature]
wpts_data_new[:,0] = wpts_xi_
wpts_data_new[:,1] = wpts_yi_
wpts_data_new[:,2] = wpts_ii_
wpts_data_new[:,3] = wpts_curv_

print("wpts_data_new :", wpts_data_new.shape)

# Plot waypoints
plt.figure()
plt.plot(wpts_x, wpts_y, '.b')
plt.hold(True)
plt.plot(wpts_xi_, wpts_yi_, '.r')
plt.grid()
plt.axis('equal')

plt.figure()
plt.scatter(wpts_xi_, wpts_yi_, c=wpts_curv_, cmap='winter', edgecolors='none')
plt.colorbar()

plt.show()

if SAVE_DATA:
    # Save as csv
    dataframe = pd.DataFrame(wpts_data_new)
    dataframe.to_csv(WPT_CSV_SAVE_PATH, header=False, index=False)