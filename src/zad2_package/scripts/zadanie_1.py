#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import turtlesim

from turtlesim.srv import SetPen
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute

from std_srvs.srv import *

import math
import time

points = [(10,10), (20,20), (40,10), (10,40), (40,40)]
iteration=0
is_reached=False

min_vel=0.1
error_epsilon=0.01





def turtlesim_pose_callback(data):
  global new_vel
  global points
  global iteration
  global is_reached

  pose = turtlesim.msg.Pose()
  pose.x = data.x
  pose.y = data.y
  pose.theta = data.theta

  nextPoint = turtlesim.msg.Pose()
  nextPoint.x = points[iteration][0]
  nextPoint.y = points[iteration][1]
  nextPoint.theta = 0

  print "Pozycja x: ",pose.x
  print "Pozycja y: ",pose.y
  print "Pozycja theta: ",pose.theta

  (linear, angular, error) = moveToNextPoint(pose, nextPoint)
  print "Angular=", angular

  new_vel.linear.x = linear
  new_vel.angular.z = angular

  print "Points: ", points[iteration]

  if(error<error_epsilon):
    is_reached=True
    iteration=iteration+1
    if (iteration>=len(points)):
      rospy.signal_shutdown("All points reached")
      sys.exit(1)



def moveToNextPoint(currentPoint, nextPoint):
  error=math.sqrt((nextPoint.x-currentPoint.x)**2 + (nextPoint.y-currentPoint.y)**2)

  angular=math.atan2(nextPoint.y-currentPoint.y,nextPoint.x-currentPoint.x)-currentPoint.theta


  while(angular<-math.pi):
    angular+=2*math.pi

  while(angular>math.pi):
    angular-=2*math.pi

  if(error>2):
    if(math.fabs(angular)>math.pi/6):
      linear=error*((math.pi -math.fabs(angular))/(10*math.pi))
    else:
      linear=error/2
  else:
    linear=error/2

  if(error<error_epsilon*20):
    angular=angular/2

  #angular=angular-angular/error

  return (linear, angular, error)

def initialiseServices():
  global spawn
  global reset
  global set_pen
  global teleport_absolute
  spawn = rospy.ServiceProxy('/spawn', Spawn)
  reset = rospy.ServiceProxy('/reset', Empty)
  set_pen=rospy.ServiceProxy('/turtle1/set_pen', SetPen)
  teleport_absolute=rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)


def resetBoard():
  reset()
  for point in points:
    print("Point=",point)
    drawCircle(point)

  set_pen(255,255,255,2, 0)
  new_vel.linear.x = 0
  new_vel.angular.z = 0

def drawCircle(point):
  set_pen(255,0,0,5, 1)
  teleport_absolute(point[0], point[1],0)
  set_pen(255,0,0,5, 0)
  new_vel.linear.x = 0.6
  new_vel.angular.z = 6.3
  pub.publish(new_vel)
  print "A"
  time.sleep(1)
  pub.publish(new_vel)
  time.sleep(1)
  print "B"


if __name__== "__main__":
  global new_vel
  global is_reached
  global pub
  new_vel = Twist()
  rospy.init_node('zadanie_1', anonymous=True)
  print("ready")

  pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
  time.sleep(0.5)
  # reset board
  initialiseServices()
  resetBoard()

  rospy.Subscriber( '/turtle1/pose' , turtlesim.msg.Pose, turtlesim_pose_callback)


  rate=rospy.Rate(10) # 10Hz
  while not rospy.is_shutdown():
    is_reached=False
    pub.publish(new_vel)#wyslanie predkosci zadanej
    rate.sleep()


  print("END")
