# ackermann_drive_teleop
ROS teleoperation scripts for robots with ackermann steering

for using in the f1tenth gazebo simulator, use following command
```
rosrun ackermann_drive_teleop keyop.py 0.5 0.8 car_1/command
```

##### ackermann_drive_keyop
+ Run the teleoperation script, with  
`rosrun ackermann_drive_teleop keyop.py`  
+ You can set max speed, steering angle and command topic by giving them as arguments, when running the script.  
eg.1 `rosrun ackermann_drive_teleop keyop.py 0.5`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; -> max_speed=max_steering_angle=0.5, command_topic=/ackermann_cmd  
eg.2 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max_speed=0.5, max_steering_angle=0.8, command_topic=/ackermann_cmd  
eg.3 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8 ack_cmd`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max=speed=0.5, max_steering_angle=0.8, command_topic=/ack_cmd  
+ Use the "up", "down" arrow keys to control speed, "left" and "right" arrow keys to control the steering angle,  
  space to brake and tab to reset the steering angle.  