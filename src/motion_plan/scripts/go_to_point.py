#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import SetBool, SetBoolResponse

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_gp_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.05

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def change_state_gp(state):
    global state_gp_
    state_gp_ = state
    print('State changed to [%s]' % state_gp_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def euclidean_distance(position_, des_pos):
    return math.sqrt((des_pos.x - position_.x) ** 2 + (des_pos.y - position_.y) ** 2)

def linear_vel(position_, des_pos, constant=1.5):
    vel = constant * euclidean_distance(position_, des_pos)
    return vel if vel < max_vel else max_vel

def steering_angle(position_, yaw_, des_pos):
    ang = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x) - yaw_
    return ang

def angular_vel(position_, yaw_, des_pos, constant=6):
    angular = steering_angle(position_, yaw_, des_pos)
    a = steering_angle(position_, yaw_, des_pos)
    while a < -math.pi:
        angular += 2*math.pi
        print("angular2=",angular)
        break
    while a > math.pi:
        angular -= 2*math.pi
        print("angular3=",angular)
        break
    rot = constant * angular
    return rot if rot < max_rot else max_rot

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_gp_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    # ang = angular_vel(position_, -yaw_, des_pos, rot_mult)
    
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        if rospy.get_time() % 2 < 0.06:
            log = "math.fabs(err_yaw) = [%s]" % math.fabs(steering_angle(position_, yaw_, des_pos))
            rospy.loginfo(log)

        # Angular velocity in the z-axis.
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = -angular_vel(position_, yaw_, des_pos, rot_mult) #0.8 if err_yaw > 0 else -0.8
        
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        log = 'Yaw error1: [%s]' % err_yaw
        rospy.logerr(log)
        change_state_gp(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_gp_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    lin = linear_vel(position_, des_pos, vel_mult)
    
    twist_msg = Twist()
    if err_pos > dist_precision_:
        # Linear velocity in the x-axis.
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.linear.x = lin# if math.fabs(steering_angle(position_, yaw_, des_pos)) < math.pi/30 else lin/4

        # Angular velocity in the z-axis.
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = -angular_vel(position_, yaw_, des_pos, rot_mult) #0.8 if err_yaw > 0 else -0.8

        # twist_msg.linear.x = 0.5
        # twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print('Position error: [%s]' % err_pos)
        change_state_gp(2)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    exit()

def main():
    global pub, active_, point, distance_tolerance, max_vel, vel_mult, max_rot, rot_mult, refresh_rate

    # Initial first goal
    point = 0
    
    # Please, insert a number slightly greater than 0 (e.g. 0.01).
    distance_tolerance = 0.02
    
    # Maximum linear velocity:
    max_vel = 0.6
    
    # Linear velocity multiplier:
    vel_mult = 10
    
    # Maximum rotational velocity:
    max_rot = 1*math.pi
    
    # Rotation speed multiplier:
    rot_mult = 2
    
    # Rate of ublishing new velocities
    refresh_rate = 60
    
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(refresh_rate)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_gp_ == 0:
                fix_yaw(desired_position_)
            elif state_gp_ == 1:
                go_straight_ahead(desired_position_)
            elif state_gp_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    print("END")