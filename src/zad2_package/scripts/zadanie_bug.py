#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import * # import ros service
import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
# initial_position_ = Point()
# initial_position_.x = rospy.get_param('initial_x')
# initial_position_.y = rospy.get_param('initial_y')
# initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = 2 #rospy.get_param('goal_pose_x')
desired_position_.y = 2 #rospy.get_param('goal_pose_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'Circumnavigate obstacle', 'Go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

#callbacks
def clbk_odom(msg):
	global position_, yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]

def clbk_laser(msg):
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}

def change_state(state):
	global state_, state_desc_
	global srv_client_wall_follower_, srv_client_go_to_point_
	global count_state_time_
	count_state_time_ = 0
	state_ = state
	log = "state changed: %s" % state_desc_[state]
	rospy.loginfo(log)
	if state_ == 0:
		srv_client_go_to_point_(True)
		srv_client_wall_follower_(False)
	if state_ == 1:
		srv_client_go_to_point_(False)
		srv_client_wall_follower_(True)
	if state_ == 2:
		srv_client_go_to_point_(False)
		srv_client_wall_follower_(True)

def calc_dist_points(point1, point2):
	dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
	return dist

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle








# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	print(scan)

# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global new_vel
	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([
		odom.pose.pose.orientation.x,
		odom.pose.pose.orientation.y,
		odom.pose.pose.orientation.z,
		odom.pose.pose.orientation.w])[2]
	print("Pozycja x: ",odom.pose.pose.position.x)
	print("Pozycja y: ",odom.pose.pose.position.y)
	print("Pozycja theta: ",pose.theta)
	new_vel.linear.x = 0.15
	new_vel.angular.z = 0.3

def main():
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_#, initial_position_, state_desc_
	global srv_client_go_to_point_, srv_client_wall_follower_
	global circumnavigate_closest_point_, circumnavigate_starting_point_
	global count_loop_, count_state_time_

	rospy.init_node('bug1', anonymous=True)

	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)

	rospy.wait_for_service('/go_to_point_switch')
	rospy.wait_for_service('/wall_follower_switch')
	rospy.wait_for_service('/gazebo/set_model_state')

	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch')
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch')
	srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state')

	# set robot position
	# model_state = ModelState()
	# model_state.model_name = 'waffle_pi'
	# model_state.pose.position.x = initial_position_.x
	# model_state.pose.position.y = initial_position_.y
	# srv_client_set_model_state(model_state)

	# initialize going to the point
	change_state(0)

	rate_hz = 10
	rate=rospy.Rate(rate_hz) # 10Hz

	global new_vel
	new_vel = Twist()
	# rospy.init_node('bug1', anonymous=True)
	print("ready")
	# rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	# rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		if regions_ == None:
			continue
		
		if state_ == 0:
			if regions_['front'] > 0.15 and regions_['front'] < 1:
				circumnavigate_closest_point_ = position_
				circumnavigate_starting_point_ = position_
				change_state(1)
		
		elif state_ == 1:
			# if current position is closer to the goal than the previous closest_point
			if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
				circumnavigate_closest_point_ = position_
			       
         # compare only after 5 seconds - need some time to get out of starting_point
         # if robot reaches (is close to) starting point
			
			if count_state_time_ > 5 and calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
				change_state(2)
				
		elif state_ == 2:
         # if robot reaches (is close to) closest point
			if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
				change_state(0)
				 
			count_loop_ = count_loop_ + 1
			if count_loop_ == 20:
				count_state_time_ = count_state_time_ + 1
				count_loop_ = 0


		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()


if __name__== "__main__":
	main()
	print("END")
