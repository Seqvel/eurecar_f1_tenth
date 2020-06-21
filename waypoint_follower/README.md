# Waypoint Follower
Package for controlling the ego vehicle to follow the pre-built waypoint trajectory in the gazebo simulator.

## Requirements 
1. Python packages
```
pip2 install numpy==1.16.6 --user
pip2 install pandas==0.24.2 --user
pip2 install roslib==1.14.8 --user
```
2. ROS Melodic messages
```
sudo apt-get install ros-melodic-geometry-msgs
sudo apt-get install ros-melodic-visualization-msgs
sudo apt-get install ros-melodic-nav-msgs
sudo apt-get install ros-melodic-ackermann-msgs
```

## Explanation
### Waypoint follower (controller.py)
The task waypoint follower do are
- Load pre-built waypoint trajectory
- Subscribe the current ego vehicle position
- Calculate error values to control
- Calculate appropriate control command
- Publish the control command

```
rosrun waypoint_follower controller.py
```

## TODO
What you need to do is design a vehicle controller for driving the ego vehicle as fast as possible in the gazebo environment.

All interfaces between the simulator and this controller have already implemented. (i.e., Subscribing current ego position, Publish control command to the simulator)

To load pre-build waypoint trajectory, change below paramter
- WPT_CSV_PATH : the path for your saved waypoint trajectory

You can change these functions in controller.py
1. calc_error : This is for calculating errors. This function selects a control point by finding the nearest waypoint between the waypoint path and ego vehicle using the current ego x, y, and pre-build waypoint info. It calculates cross-track error (error_y) and heading angle error (error_yaw). You can change the way to select the control point (i.e., applying look-ahead distance) and add more errors if you want.
2. steer_control : This is for calculating appropriate steer control commands. It calculates steering angle commands using errors from the 'calc_error' function. Currently, a simple P Controller is implemented and all control parameters are zero. You need to tune this parameters or implement better controller for better performance.
3. velocity_profile : This is for calculating appropriate velocity commands. Currently, constant velocity profile is implemented, but you need to improve this for better performance.

Reference paper for designing controller
- Automatic Steering Methods for Autonomous Automobile Path Tracking : https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf