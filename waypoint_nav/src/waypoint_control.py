#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Point, Pose2D, Twist
from std_msgs.msg import String, Float32

# Variables
global k_rho, k_alpha, state, substate, goal_set, distance, angle, goal, vel, vel_maxlin, vel_maxang
k_rho = 0.3
k_alpha = 0.8
state = "STOP"
substate = "STOP"
goal_set = False
distance = 0.0
angle = math.pi
goal = Point()
vel = Twist()
vel.linear.x = 0
vel.angular.z = 0
vel_maxlin = 1
vel_maxang = 2

# Callback functions
def goalCB(g):
  global goal_set, goal, substate
  goal = g
  goal_set = True
  substate = "STOP"
  print "Waypoint received"

def paraCB(p):
  global k_rho, k_alpha
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 2:
      k_rho = float(parts[0])
      k_alpha = float(parts[1])
      print "Parameter updated:"
      print "k_rho: " + str(k_rho)
      print "k_alpha: " + str(k_alpha) 
    else:
      print "2 parameter needed, only " + str(len(parts)) + " sent"

def stateCB(s):
  global state
  state = s.data
  print "Waypoint control state updated: " + state
  
def poseCB(p):
  global goal_set, distance, angle, goal
  if goal_set:
    dx = (goal.x-p.x)*math.cos(p.theta)+(goal.y-p.y)*math.sin(p.theta)
    dy = -(goal.x-p.x)*math.sin(p.theta)+(goal.y-p.y)*math.cos(p.theta)
    distance = math.sqrt( (dx)**2 + (dy)**2 )
    angle = math.atan2(dy,dx)

def linCB(l):
  global vel_maxlin
  vel_maxlin = l.data
  print "Max linear speed is set to: " + str(vel_maxlin)

def angCB(a):
  global vel_maxang
  vel_maxang = a.data
  print "Max angular speed is set to: " + str(vel_maxang)  

# Init ROS node
rospy.init_node('waypoint_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)

# Subscribers
state_sub = rospy.Subscriber('waypoint/state', String, stateCB)
pose_sub = rospy.Subscriber('robot_pose', Pose2D, poseCB)
goal_sub = rospy.Subscriber('waypoint/goal', Point, goalCB)
para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():

  if goal_set:
    if state == "RUNNING":
      if substate != "FORWARDING":
        substate = "TURNING"    
    
      if substate == "TURNING":
        vel.linear.x = 0
        vel.angular.z = k_alpha * angle
        if math.fabs(angle) < 0.2:
          substate = "FORWARDING"        
      elif substate == "FORWARDING":
      	if (angle > math.pi/2):
          vel.linear.x = -k_rho * distance
          vel.angular.z = k_alpha * (angle-math.pi)  
        elif (angle < -math.pi/2):
          vel.linear.x = -k_rho * distance
          vel.angular.z = k_alpha * (angle+math.pi)  
        else:
          vel.linear.x = k_rho * distance
          vel.angular.z = k_alpha * angle 
    
      if vel.linear.x > vel_maxlin:
        vel.linear.x = vel_maxlin
      if vel.angular.z > vel_maxang:
        vel.angular.z = vel_maxang
     
      vel_pub.publish(vel)
      
    elif state == "PARK":
      substate = "STOP"
      vel.linear.x = 0
      vel.angular.z = 0
      vel_pub.publish(vel)
      
    else:
      if prestate == "RUNNING":
        substate = "STOP"
        vel.linear.x = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
      else:
        substate = "IDLE"
        
    prestate = state
        
  state_pub.publish(substate)
  rate.sleep()

