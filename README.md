# Eurecar F1tenth Project
This project is based on F1TENTH Project.
Please refer to the following site:

[https://f1tenth.org/](https://f1tenth.org/)

[https://github.com/f1tenth-dev/simulator](https://github.com/f1tenth-dev/simulator)

This project provides gazebo simulator and messages. 
Therefore, it is a proper environment to implement such as vehicle control, simple mapping, and localization algorithm. We hope you to try whatever you want to test in this simulator. We modified the origin f1tenth model for you to use an IMU sensor. Also, we added the oval shape easy map.

## ROS install guidance
For ROS-beginner, we provide you a link installing ROS.

We recomment you to install ROS-melodic distro.

you can follow the instruction here:
[ROS-melodic-installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

Also, you can follow the documentation to start ROS and set the environment to use ros-command comfortably.
See this page: [ROS/StartGuide](http://wiki.ros.org/ROS/StartGuide)

## F1tenth 
If you finish to install ROS-melodic, you need to do git clone:
```
user@ros-computer: cd ~/catkin_ws/src
user@ros-computer: git clone https://github.com/Leedk3/eurecar_f1_tenth.git
user@ros-computer: cd ~/catkin_ws && catkin_make
```

## F1tenth Gazebo 
We divided the origin f1tenth launch file into 2 steps: 
1. start gazebo
2. keyborad controller to move vehicle
```
user@ros-computer: roslaunch f1tenth-sim simulator.launch
user@ros-computer: roslaunch f1tenth-sim key_control.launch
```
You can modify the key_controller to add the motion or decrease the highest speed in the simulator.

If your vehicle does not move even though you followed the instruction in the **ros/f1tenth-dev**, here are some possibilities we have experienced and resolved it. When packages listed below were not installed, the vehicle did not move and the error occurred. 

1. effort controller
```
user@ros-computer: sudo apt-get install ros-melodic-effort-controllers
```
2. gazebo packages
```
user@ros-computer: sudo apt-get install ros-melodic-gazebo-dev
user@ros-computer: sudo apt-get install ros-melodic-gazebo-ros-control
user@ros-computer: sudo apt-get install ros-melodic-gazebo-ros
``` 

## Oval track
For using oval-shaped track, you need to copy some files in ~/.gazebo/models (If you don't have the directory, run `mkdir -p ~/.gazebo/models`)
```
user@ros-computer: cp -r /simulator/world/oval_track ~/.gazebo/models
```

![Alt text](/media/gazebo_oval_track.png "Oval Track")


The 2D map can be generated using a sensor model in the gazebo environment. This figure is an example of map building process in gazebo environment. The sensors used for map building are 2D Hokuyo LiDAR and 3DM-GX5-25 IMU sensors.

![Alt text](/media/mapping_in_gazebo.png.png "Mapping")

Here is the result of the generated 2D map using Google Cartographer in the track.

![Alt text](/media/rviz_google_cartographer.png "Cartographer")

As a 2D map building algorithm, Cartographer and Gmapping are mainly used. Please refer to the following site:

Google Cartographer: [https://google-cartographer-ros.readthedocs.io/en/latest/#](https://google-cartographer-ros.readthedocs.io/en/latest/#)

Gmapping: [http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping)

Note that when using a Cartographer, the vehicle's odometry may be necessary to increase the accuracy of map building performance. In the 2D map generated by these map building algorithm, we can estimate the position of the vehicle using LiDAR and IMU sensor data.
## Vehicle Controller
### 1.Waypoint Recording with manual driving (waypoint_recorder)

Please refer to the waypoint_recorder package.

To record,
```
user@ros-computer: rosrun waypoint_recorder recorder.py
```
To visualize the saved waypoint trajectory,
```
user@ros-computer: rosrun waypoint_recorder loader.py
```

### 2. Design the skeleton code to control the vehicle (waypoint_follower)
Please refer to the waypoint_follwer package.

To run the controller,
```
user@ros-computer: rosrun waypoint_follower controller.py
```

## Waypoint Follower
>Package for controlling the ego vehicle to follow the pre-built waypoint trajectory in the gazebo simulator.

### Requirements 
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

### Explanation
#### Waypoint follower (controller.py)
The task waypoint follower do are
- Load pre-built waypoint trajectory
- Subscribe the current ego vehicle position
- Calculate error values to control
- Calculate appropriate control command
- Publish the control command

```
rosrun waypoint_follower controller.py
```

### TODO
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

