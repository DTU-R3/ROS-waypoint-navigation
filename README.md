# ROS-waypoint-navigation
============================

This is the code to drive the robot through a series of waypoint defined by the user. The robot is supposed to have two position sensors on the left and right side respectively.

## How to use
Install pyproj package
```
sudo apt-get install python-pip
sudo pip install pyproj
```

## waypoint_convert.py 
The code receives waypoint information from user as well as the left and right sensor data. Afterwards, the robot postion and orientation are calculated and send to the waypoint_control node. The waypoint is published to the waypoint_control node as position relative to the robot base. 

## waypoint_control.py 
The code receives the relative position of the waypoint to the robot base. Besides, the 'waypoint/state' topic is defined to enable/disable the waypoint control. The code publishes the 'cmd_vel' topic to directly control the robot.

The control algorithm is simplied as 
```
vel.linear.x = k_rho * distance
vel.angular.z = k_alpha * angle 
```

The control parameters of the control algorithm can also be tuned as ROS topics
