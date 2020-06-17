# Eurecar F1tenth Project
This project is based on F1TENTH Project.
Please refer to the following site:

[https://f1tenth.org/](https://f1tenth.org/)

[https://github.com/f1tenth-dev/simulator](https://github.com/f1tenth-dev/simulator)

This project provides gazebo simulator and messeges. 
Therefore, it is proper environment to implement such as vehicle control, simple mapping and localization algorithm. We hope you to try whatever you want to test in this simulator. We modified the origin f1tenth model for you to use a IMU sensor. Also, we added the oval shape easy map. 

## ROS install guidance
For ROS-beginner, we provide you a link installing ROS.

We recomment you to install ROS-melodic distro.

you can follow the instruction here:
[ROS-melodic-installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

Also, you can follow the documentation to start ROS and set the environment to use ros-command comfortably.
See this page: [ROS/StartGuide](http://wiki.ros.org/ROS/StartGuide)

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


## Vehicle Controller(Skeleton Code) 
> You can use this form
>	> You can use this form
> > > You can use this form

```
put the bash command in here.
put the bash command in here.
```

1. You can use this numbering
2. You can use this numbering

* You can use this header
  * You can use this header
    * You can use this header

+ You can use this header
  + You can use this header
    + You can use this header

- You can use this header
  - You can use this header
    - You can use this header
 
