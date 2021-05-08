#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def clbk_laser(msg):
    # 720 / 5 = 144
    # tuple_rear_right =  msg.ranges[0:143]
    # tuple_front_right = msg.ranges[144:287]
    # tuple_front =       msg.ranges[430:431]
    # tuple_front_left =  msg.ranges[432:575]
    # tuple_rear_left =   msg.ranges[576:719]
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
    
    take_action(regions_)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['rright'] > 1 and regions['fleft'] > 1 and regions['rright'] > 1:
        state_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif regions['rright'] < 1 and regions['fleft'] > 1 and regions['rright'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['rright'] > 1 and regions['fleft'] > 1 and regions['rright'] < 1:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['rright'] > 1 and regions['fleft'] < 1 and regions['rright'] > 1:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['rright'] < 1 and regions['fleft'] > 1 and regions['rright'] < 1:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['rright'] < 1 and regions['fleft'] < 1 and regions['rright'] > 1:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['rright'] < 1 and regions['fleft'] < 1 and regions['rright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['rright'] > 1 and regions['fleft'] < 1 and regions['rright'] < 1:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub
    
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
