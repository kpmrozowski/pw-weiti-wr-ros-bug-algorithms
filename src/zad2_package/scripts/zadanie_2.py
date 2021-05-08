#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
# import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import turtlesim
from math import pow, atan2, sqrt, pi
import time

class TurtleBot:
    def __init__(self):
        global points, distance_tolerance, point, point_old, max_vel, vel_mult, \
                max_rot, rot_mult, smallest_distances, trigger, start

        # Load points matrix
        with open('/home/kmro/wr_ws/src/zad2_package/points.txt') as f:
                w, h = [int(x) for x in next(f).split()]
                points = [[int(x) for x in line.split()] for line in f]
        print(points)
        print('points[0][1] = ', points[0][1])

        # Initial first goal
        point = 0

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.02

        # Maximum linear velocity:
        max_vel = 40

        # Linear velocity multiplier:
        vel_mult = 10

        # Maximum rotational velocity:
        max_rot = 40*pi

        # Rotation speed multiplier:
        rot_mult = 20
        
        # Rate of ublishing new velocities
        refresh_rate = 60

        smallest_distances = []
        trigger = True
        point_old = -1

        start = time.time()

        rospy.init_node('wr_zad', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
                                                  
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(refresh_rate)

    def update_pose(self, data):
        """Callback function which is called when a new message
        of type Pose is received by the subscriber."""
        self.pose = data
            
        self.pose.y = round(self.pose.y, 6)

    def euclidean_distance(self, goal_pose):
        return sqrt((goal_pose.x - self.pose.x) ** 2 +
                    (goal_pose.y - self.pose.y) ** 2)

    def linear_vel(self, goal_pose, constant=1.5):
        vel = constant * self.euclidean_distance(goal_pose)
        return vel if vel < max_vel else max_vel

    def steering_angle(self, goal_pose):
        ang = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta
        return ang

    def angular_vel(self, goal_pose, constant=6):
        angular = self.steering_angle(goal_pose)
        a = self.steering_angle(goal_pose)
        while a < -pi:
            angular += 2*pi
            print("angular2=",angular)
            break
        while a > pi:
            angular -= 2*pi
            print("angular3=",angular)
            break
        rot = constant * angular
        return rot if rot < max_rot else max_rot

    def move2goal(self):
        """Moves the turtle to the goal."""
        
        global points, point, point_old, distance_tolerance, trigger, start

        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = points[point][0] # float(input("Set your x goal: "))
        goal_pose.y = points[point][1] # float(input("Set your y goal: "))

        vel_msg = Twist()

        data = [['nameservers','panel'], ['nameservers','panel']]

        file_name2 = "/home/kmro/wr_ws/src/zad2_package/short_distances/distances-p%d" % point
        short_distances = open(file_name2, "w")
        
        file_name1 = "/home/kmro/wr_ws/src/zad2_package/distances/distances-p%d" % point
        all_distances_file = open(file_name1, "w")

        val = "dx%d\t\t" % (point-1) + "dy%d\t\t" % (point-1) + "dx%d\t\t" % point + "dy%d\n" % point 
        short_distances.write(str(val))

        val = "dx\t\tdy"
        for i in range(22):
            val = val + "\t\tdx%d\t\t" % i + "dy%d" % i 
        all_distances_file.write(str(val))

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose, vel_mult)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose, rot_mult)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Print results to files
            if point_old != point:
                print("point = ", point)
                point_old = point
            if point > 0:
                val = "{:.3f}\t".format(points[point-1][0] - self.pose.x)
                short_distances.write(str(val))
                # print(val, end=' ')
                val = "{:.3f}\t".format(points[point-1][1] - self.pose.y)
                short_distances.write(str(val))
                # print(val, end=' ')
                val = "{:.3f}\t".format(points[point][0] - self.pose.x)
                short_distances.write(str(val))
                # print(val, end=' ')
                val = "{:.3f}\t".format(points[point][1] - self.pose.y)
                short_distances.write(str(val))
                # print(val, end=' ')
                if trigger == True:
                    smallest_distances.append(((points[point-1][0] - self.pose.x)**2 + (points[point-1][1] - self.pose.y)**2)**0.5)
                    trigger = False
            short_distances.write("\n")

            val = "{:.3f}\t".format(goal_pose.x - self.pose.x)
            all_distances_file.write(str(val))
            val = "{:.3f}\t".format(goal_pose.y - self.pose.y)
            all_distances_file.write(str(val))
            for i in range(1,len(points)):
                val = "{:.3f}\t".format(points[i-1][0] - self.pose.x)
                all_distances_file.write(str(val))
                # print(val, end=' ')
                val = "{:.3f}\t".format(points[i-1][1] - self.pose.y)
                all_distances_file.write(str(val))
                # print(val, end=' ')
            all_distances_file.write("\n")

            # Publish at the desired rate.
            self.rate.sleep()
        
        short_distances.close()
        all_distances_file.close()

        # If it was not the last goal, then move to the second one
        if point < len(points) - 1:
            trigger = True
            point = point + 1
            goal_pose.x = points[point][0]
            goal_pose.y = points[point][1]
            vel_msg.linear.x = self.linear_vel(goal_pose, vel_mult)
            vel_msg.angular.z = self.angular_vel(goal_pose, rot_mult)
            self.move2goal()
        # Stopping our robot after the movement is over.
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            suma = 0
            i = 0
            for j in smallest_distances:
                print("p%d: " % i , "%.3f error" % j)
                i = i + 1
            print("error_sum(22) = %.3f" % sum(smallest_distances))
            end = time.time()
            print("Elapsed time: ", end - start)
            exit()
                
        point = point + 1
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()
   
if __name__== "__main__":
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
    print("END")