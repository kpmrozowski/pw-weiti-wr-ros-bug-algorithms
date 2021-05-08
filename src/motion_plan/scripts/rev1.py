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
srv_client_wall_follower_counterclockwise = None
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
# regions_ = None
state_desc_ = ['Go to point', 'clockwise_circumnavigate obstacle', 'counterclockwise_circumnavigate obstacle']
state_ = 0
state_changes_count = 0
hit_point_ = Point()
hit_first_point_ = Point()
m_line_error = .05
def m_line_x_a():
    return (desired_position_.x - initial_position_.x) / (desired_position_.y - initial_position_.y)
def m_line_y_a():
    return (desired_position_.y - initial_position_.y) / (desired_position_.x - initial_position_.x)
def m_line_x(y_coord):
    return (y_coord - desired_position_.y) * (desired_position_.x - initial_position_.x) / (desired_position_.y - initial_position_.y) + desired_position_.x
def m_line_y(x_coord):
    return (x_coord - initial_position_.x) * (desired_position_.y - initial_position_.y) / (desired_position_.x - initial_position_.x) + initial_position_.y
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
dist_front = None
hit_points_ = []
# 0 - go to point
# 1 - clockwise_circumnavigate
# 2 - counterclockwise_circumnavigate

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
    global index_min, dist_min, dist_front#, regions_
    # regions_ = {
    #     'rright': min(min(msg.ranges[180:251]), 3.5), 
    #     'right':  min(min(msg.ranges[252:323]), 3.5),
    #     'fright': min(min(msg.ranges[324:359]), 3.5),      
    #     'fleft':  min(min(msg.ranges[0:35]), 3.5),      
    #     'left':   min(min(msg.ranges[36:107]), 3.5), 
    #     'rleft':  min(min(msg.ranges[108:179]), 3.5)
    # }
    index_min = msg.ranges.index(min(msg.ranges))
    dist_min = min(min(msg.ranges), 3.5)
    dist_front = msg.ranges[0]

def change_state(state):
    global state_, state_desc_, srv_client_wall_follower_counterclockwise, srv_client_wall_follower_clockwise, srv_client_go_to_point_, count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        srv_client_go_to_point_(True)
        srv_client_wall_follower_clockwise(False)
        srv_client_wall_follower_counterclockwise(False)
    if state_ == 1:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_clockwise(True)
        srv_client_wall_follower_counterclockwise(False)
    if state_ == 2:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_clockwise(False)
        srv_client_wall_follower_counterclockwise(True)

def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def steering_angle(position_, yaw_, des_pos):
    ang = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x) - yaw_
    return ang

def main():
    global index_min, dist_min, dist_front
    global position_, desired_position_, state_, yaw_, yaw_error_allowed_#, regions_
    global srv_client_go_to_point_, srv_client_wall_follower_clockwise, srv_client_wall_follower_counterclockwise
    global hit_point_, hit_first_point_, state_changes_count
    global count_loop_, count_state_time_, m_line_error
    
    rospy.init_node('alg1')
    
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch_clockwise')
    rospy.wait_for_service('/wall_follower_switch_counterclockwise')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_clockwise = rospy.ServiceProxy('/wall_follower_switch_clockwise', SetBool)
    srv_client_wall_follower_counterclockwise = rospy.ServiceProxy('/wall_follower_switch_counterclockwise', SetBool)
    
    # initialize going to the point
    change_state(0)
    
    rate_hz = 60
    rate = rospy.Rate(rate_hz)
    # srv_client_go_to_point_(True)
    while not rospy.is_shutdown():
        # if regions_ == None:
        #     continue
        
        if state_ == 0:
            cond_time = count_state_time_ > 1.
            if rospy.get_time() % 2 < 1.01/rate_hz:
                log = "state_ == 0"
                rospy.loginfo(log)
            if dist_front < 1.4 and cond_time:
                rospy.logerr('fright')
                if state_changes_count == 0:
                    hit_first_point_ = position_
                state_changes_count += 1
                hit_point_ = position_
                hit_points_.append(position_)
                if len(hit_points_) % 2 == 1:
                    change_state(1)
                else:
                    change_state(2)
        
        elif state_ == 1:
            if rospy.get_time() % 1 < 1.01/rate_hz:
                log = "state_ == 1"
                rospy.loginfo(log)
            # if current position is between target and initial position and current position is closer to the target than hitpoint than move towards target
            cond_slope = abs(m_line_x_a()) >= 1
            cond_between_y = \
                position_.y > min(desired_position_.y, initial_position_.y) + .4 and \
                position_.y < max(desired_position_.y, initial_position_.y) - .4
            cond_between_x = \
                position_.x > min(desired_position_.x, initial_position_.x) + .4 and \
                position_.x < max(desired_position_.x, initial_position_.x) - .4
            cond_closer_to_target = \
                calc_dist_points(position_, desired_position_) < \
                calc_dist_points(hit_point_, desired_position_)
            cond_m_line_y = abs(m_line_y(position_.x) - position_.y) < m_line_error
            cond_m_line_x = abs(m_line_x(position_.y) - position_.x) < m_line_error
            cond_time = count_state_time_ > 2.8
            log = 'cond_slope={}, cond_between_y={}, cond_between_x={}, cond_m_line_y={}, cond_m_line_x={}, cond_closer_to_target={}, cond_time={}'.format(cond_slope, cond_between_y, cond_between_x, cond_m_line_y, cond_m_line_x, cond_closer_to_target, cond_time)
            # if rospy.get_time() % 2 < 0.1:
            # rospy.logwarn(log)
            if desired_position_.x != initial_position_.x:
                # if calc_dist_points(position_, hit_point_) < 1.2 and count_state_time_ > 5:
                #     change_state(2)
                # rospy.loginfo('black=' + str(abs(steering_angle(position_, yaw_, desired_position_)/math.pi*180 - index_min)))
                if cond_slope:
                    if cond_closer_to_target and cond_m_line_y and (cond_between_y or cond_between_x) and dist_front > 2. and abs(steering_angle(position_, yaw_, desired_position_)/math.pi*180 - index_min) > 90 and cond_time:
                        change_state(0)
                else:
                    if cond_closer_to_target and cond_m_line_x and (cond_between_y or cond_between_x) and dist_front > 2. and abs(steering_angle(position_, yaw_, desired_position_)/math.pi*180 - index_min) > 90 and cond_time:
                        change_state(0)
            elif abs(desired_position_.x - position_.x) < m_line_error and position_.y > min(desired_position_.y, initial_position_.y)  + m_line_error and position_.y < max(desired_position_.y, initial_position_.y) - m_line_error and cond_time:
                change_state(0)
        elif state_ == 2:
            if rospy.get_time() % 2 < 1.01/rate_hz:
                log = "state_ == 2"
                rospy.loginfo(log)
            # if current position is between target and initial position and current position is closer to the target than hitpoint than move towards target
            cond_slope = abs(m_line_x_a()) >= 1
            cond_between_y = \
                position_.y > min(desired_position_.y, initial_position_.y) + .4 and \
                position_.y < max(desired_position_.y, initial_position_.y) - .4
            cond_between_x = \
                position_.x > min(desired_position_.x, initial_position_.x) + .4 and \
                position_.x < max(desired_position_.x, initial_position_.x) - .4
            cond_closer_to_target = \
                calc_dist_points(position_, desired_position_) < \
                calc_dist_points(hit_point_, desired_position_)
            cond_m_line_y = abs(m_line_y(position_.x) - position_.y) < m_line_error
            cond_m_line_x = abs(m_line_x(position_.y) - position_.x) < m_line_error
            cond_time = count_state_time_ > 2.8
            log = 'cond_slope={}, cond_between_y={}, cond_between_x={}, cond_m_line_y={}, cond_m_line_x={}, cond_closer_to_target={}, cond_time={}'.format(cond_slope, cond_between_y, cond_between_x, cond_m_line_y, cond_m_line_x, cond_closer_to_target, cond_time)
            # if rospy.get_time() % 2 < 0.1:
            # rospy.logwarn(log)
            if desired_position_.x != initial_position_.x:
                # if calc_dist_points(position_, hit_point_) < 1.2 and count_state_time_ > 5:
                #     change_state(1)
                if cond_slope:
                    if cond_closer_to_target and cond_m_line_y and (cond_between_y or cond_between_x) and dist_front > 2. and abs(steering_angle(position_, yaw_, desired_position_)/math.pi*180 - index_min) > 90 and cond_time:
                        change_state(0)
                else:
                    if cond_closer_to_target and cond_m_line_x and (cond_between_y or cond_between_x) and dist_front > 2. and abs(steering_angle(position_, yaw_, desired_position_)/math.pi*180 - index_min) > 90 and cond_time:
                        change_state(0)
            elif abs(desired_position_.x - position_.x) < m_line_error and position_.y > min(desired_position_.y, initial_position_.y)  + m_line_error and position_.y < max(desired_position_.y, initial_position_.y) - m_line_error and cond_time:
                change_state(0)
            else:
                pass
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == rate_hz:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    exit()
    print("END")
