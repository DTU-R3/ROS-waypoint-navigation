#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Float32
from pyproj import Proj

# Callback functions
def leftCB(l):
  global left_x,left_y
  left_x,left_y = projection(l.longitude, l.latitude)

def rightCB(r):
  global right_x,right_y
  right_x,right_y = projection(r.longitude, r.latitude)
 
def pointCB(p):
  goal = Point()
  goal.x,goal.y = projection(p.longitude, p.latitude)
  goal.z = 0
  waypoint_pub.publish(goal)

# Init ROS node
rospy.init_node('waypoint_convert')

# Publishers
waypoint_pub = rospy.Publisher('waypoint/goal', Point, queue_size = 10)
pose_pub = rospy.Publisher('robot_pose', Pose2D, queue_size = 10)
robot_gps_pub = rospy.Publisher('robot_gps_pose', NavSatFix, queue_size = 10)
robot_heading_pub = rospy.Publisher('robot_heading', Float32, queue_size = 10)

# Subscribers
leftGOT_sub = rospy.Subscriber('waypoint/left_sensor', NavSatFix, leftCB)
rightGOT_sub = rospy.Subscriber('waypoint/right_sensor', NavSatFix, rightCB)
point_sub = rospy.Subscriber('waypoint', NavSatFix, pointCB)

rate = rospy.Rate(1000)

# Variables
global left_x,left_y,right_x,right_y
left_x = 0.0
left_y = 0.0
right_x = 0.0
right_y = 0.0

projection = Proj(proj="utm", zone="34", ellps='WGS84')

while not rospy.is_shutdown():
  # Publish robot position in projection coordinate
  pose = Pose2D()
  gpspose = NavSatFix()
  pose.x = (left_x + right_x) / 2.0
  pose.y = (left_y + right_y) / 2.0
  pose.theta = math.atan2( (right_y-left_y), (right_x-left_x) ) + math.pi / 2.0
  if pose.theta > math.pi:
    pose.theta = pose.theta - 2.0 * math.pi
  pose_pub.publish(pose)
  
  # Publish robot position in GPS
  gpspose.longitude,gpspose.latitude = projection(pose.x,pose.y,inverse=True)
  robot_gps_pub.publish(gpspose)
  
  # Publish robot heading, 0-359 degrees, North: 0, East: 90
  heading_rad =-math.atan2( (right_y-left_y), (right_x-left_x) )
  if heading_rad < 0:
    heading_rad = heading_rad + 2.0 * math.pi
  heading = math.fabs(heading_rad) / math.pi * 180.0
  robot_heading_pub.publish(heading)
  
  rate.sleep()
  
