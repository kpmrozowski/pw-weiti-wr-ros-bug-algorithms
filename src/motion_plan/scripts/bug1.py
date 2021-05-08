#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import SetBool, SetBoolResponse

import math

srv_client_go_to_point_ = True
srv_client_wall_follower_clockwise = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

# callbacks
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
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    # 360 / 5 = 72
    # tuple_rear_right =  msg.ranges[0:71]
    # tuple_front_right = msg.ranges[72:143]
    # tuple_front =       msg.ranges[144:215]
    # tuple_front_left =  msg.ranges[216:237]
    # tuple_rear_left =   msg.ranges[238:359]
    # array_rear_right =  [e for e in tuple_rear_right ]
    # array_front_right = [e for e in tuple_front_right]
    # array_front =       [e for e in tuple_front      ]
    # array_front_left =  [e for e in tuple_front_left ]
    # array_rear_left =   [e for e in tuple_rear_left  ]
    regions_ = {
        'rright': min(min(msg.ranges[180:251]), 3.5), 
        'right':  min(min(msg.ranges[252:323]), 3.5),
        'fright': min(min(msg.ranges[324:359]), 3.5),      
        'fleft':  min(min(msg.ranges[0:35]), 3.5),      
        'left':   min(min(msg.ranges[36:107]), 3.5), 
        'rleft':  min(min(msg.ranges[108:179]), 3.5)
    }

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_clockwise, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        srv_client_go_to_point_(True)
        srv_client_wall_follower_clockwise(False)
    if state_ == 1:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_clockwise(True)
    if state_ == 2:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_clockwise(True)

def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_clockwise
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_
    
    rospy.init_node('bug1')
    
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch_clockwise')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_clockwise = rospy.ServiceProxy('/wall_follower_switch_clockwise', SetBool)
    # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot position
    # model_state = ModelState()
    # model_state.model_name = 'turtlebot3_waffle_pi'
    # model_state.pose.position.x = initial_position_.x
    # model_state.pose.position.y = initial_position_.y
    # srv_client_set_model_state(model_state)
    
    # initialize going to the point
    change_state(0)
    
    rate_hz = 60
    rate = rospy.Rate(rate_hz)
    # srv_client_go_to_point_(True)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        if state_ == 0:
            if rospy.get_time() % 2 < 1.01/rate_hz:
                log = "state_ == 0"
                rospy.loginfo(log)
            if count_state_time_ > 4 and regions_['fright'] > 0.1 and regions_['fright'] < 1.4:
                # if rospy.get_time() % 2 < 0.06:
                log = "fright: [%s]" % regions_['fright']
                rospy.loginfo(log)
                rospy.logerr('fright')
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                change_state(1)
        
        elif state_ == 1:
            if rospy.get_time() % 2 < 1.01/rate_hz:
                dist = calc_dist_points(position_, circumnavigate_starting_point_)
                log = 'state_ == 1, dist '  + (lambda: '>', lambda: '<')[dist < 0.8]() + ' 0.8, dist = ' + str(dist)
                rospy.loginfo(log)
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
                circumnavigate_closest_point_ = position_
                
            # compare only after 4 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if count_state_time_ > 6 and \
               calc_dist_points(position_, circumnavigate_starting_point_) < 1.2:
                change_state(2)

        elif state_ == 2:
            dist = calc_dist_points(position_, circumnavigate_closest_point_)
            if rospy.get_time() % 2 < 1.01/rate_hz:
                log = 'state_ == 2, dist '  + (lambda: '>', lambda: '<')[dist < 0.2]() + ' 0.2, dist = ' + str(dist)
                rospy.loginfo(log)
            # if robot reaches (is close to) closest point
            if dist < 0.6:
                change_state(0)
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == rate_hz:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()
    

if __name__ == "__main__":
    main()
