#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import SetBool, SetBoolResponse
import numpy as np

import math

active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'rright': 0,
        'fright': 0,
        'fleft': 0,
        'left': 0,
}
state_fw = 0
state_fw_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
# parameters
yaw_precision_ = math.pi / 45 # +/- 4 degree allowed
# Maximum rotational velocity:
max_rot_fw = .001*math.pi
# Rotation speed multiplier:
rot_mult_fw = .001


def wall_follower_switch_clockwise(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global index_min, dist_min, dist_front#, regions_
    # 720 / 5 = 144
    # regions_ = {
    #     'rright': min(min(msg.ranges[180:251]), 3.5), 
    #     'right':  min(min(msg.ranges[252:323]), 3.5),
    #     'fright': min(min(msg.ranges[324:359]), 3.5),      
    #     'fleft':  min(min(msg.ranges[0:35]), 3.5),      
    #     'left':   min(min(msg.ranges[36:107]), 3.5), 
    #     'rleft':  min(min(msg.ranges[108:179]), 3.5)
    # }
    # index_min = min(range(len(msg.ranges)), key=msg.ranges.__getitem__)
    index_min = msg.ranges.index(min(msg.ranges))
    dist_min = min(min(msg.ranges), 3.5)
    dist_front = msg.ranges[0]
    
    take_action()

def change_state_fw(state):
    global state_fw, state_fw_dict_
    if state is not state_fw:
        log = 'Wall follower - [{}] - {}'.format(state, state_fw_dict_[state])
        rospy.loginfo(log)
        state_fw = state

def take_action():
    global index_min, dist_min, state_fw_description, dist_err, d
    
    state_fw_description = ''
    
    d = .5
    dist_err = .5
    deg_fw_err = 20
    yaw_tl_err_ = 10
    # # znajdz minimum, zlokalizuj w ktorej strefie, 
    # if (index_min < 80 and index_min > 0) or (index_min < 359 and index_min > 315):
    #     # state_fw_description = 'turn_left, index_min={}, dist_min={}'.format(index_min, dist_min)
    #     # if rospy.get_time() % 2 < 0.1:
    #         # rospy.logwarn(state_fw_description)
    #     change_state_fw(1)
    # else:
    #     # state_fw_description = 'follow_the_wall, index_min={}, dist_min={}'.format(index_min, dist_min)
    #     # if rospy.get_time() % 2 < 0.1:
    #         # rospy.logwarn(state_fw_description)
    change_state_fw(2)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.4
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = -1.
    return msg

def follow_the_wall():
    global index_min, dist_min, dist_front, dist_err, d
    msg = Twist()
    speed = 2
    cond_count = 3
    ang_speed = np.zeros(cond_count)
    weights = np.array([3.,2.,6.])
    msg.linear.x = .25 * speed
    if dist_min > 1.05*dist_err:
        ang_speed[0] = .5 * speed
    elif dist_min < .95*dist_err:
        ang_speed[0] = -.4 * speed
    else:
        ang_speed[0] = 0
    if index_min < 270-2:
        ang_speed[1] = .6 * speed
    elif index_min < 270+2:
        ang_speed[1] = -.3 * speed
    else:
        ang_speed[0] = 0
    if index_min < 270-10:
        ang_speed[1] = .8 * speed
    elif index_min > 270+10:
        ang_speed[1] = -.4 * speed
    if dist_front < 1.25:
        ang_speed[2] = -.4 * speed * 1.5
    msg.angular.z = np.dot(ang_speed, weights)/np.dot(weights, weights) ** 0.5
    

    return msg

def main():
    global pub_, active_, state_fw
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rospy.Service('wall_follower_switch_clockwise', SetBool, wall_follower_switch_clockwise)
    
    rate_hz = 60
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        log = "reading_laser"
        rospy.loginfo(log)
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_fw == 0:
            msg = find_wall()
            if rospy.get_time() % 2 < 40./rate_hz:
                log = "state_fw == 0"
                rospy.loginfo(log)
        elif state_fw == 1:
            msg = turn_left()
            if rospy.get_time() % 2 < 40./rate_hz:
                log = "state_fw == 1"
                rospy.loginfo(log)
        elif state_fw == 2:
            msg = follow_the_wall()
            if rospy.get_time() % 2 < 40./rate_hz:
                log = "state_fw == 2"
                rospy.loginfo(log)
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()