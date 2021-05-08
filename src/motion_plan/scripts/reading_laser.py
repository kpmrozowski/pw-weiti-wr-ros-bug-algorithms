#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 720 / 5 = 144
    tuple_rear_right =  msg.ranges[0:143]
    tuple_front_right = msg.ranges[144:287]
    tuple_front =       msg.ranges[430:431]
    tuple_front_left =  msg.ranges[432:575]
    tuple_rear_left =   msg.ranges[576:719]
    array_rear_right =  [e for e, in tuple_rear_right ]
    array_front_right = [e for e, in tuple_front_right]
    array_front =       [e for e, in tuple_front      ]
    array_front_left =  [e for e, in tuple_front_left ]
    array_rear_left =   [e for e, in tuple_rear_left  ]
    regions = [
        min(min(array_rear_right.append(0)), 10), 
        min(min(array_front_right.append(0)), 10),
        min(min(array_front.append(0)), 10),      
        min(min(array_front_left.append(0)), 10), 
        min(min(array_rear_left.append(0)), 10)]  
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
