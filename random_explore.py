#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random as rdm

global angular_multiplier

angular_multiplier = 0

def callback(data):
    global angular_multiplier
    laser_ranges = data.ranges
    min_distances = dist_from_obs(laser_ranges)
    obs_data, l_block, f_block, r_block = detect_obs(min_distances)
    angular_multiplier = avoid_obs(obs_data, l_block, f_block, r_block)

def get_laser_data():
    global angular_multiplier
    pub = rospy.Publisher('cmd_vel',Twist, queue_size = 100)
    rospy.init_node('Mover',anonymous = True)
    rate = rospy.Rate(10)
    rospy.Subscriber("base_scan", LaserScan, callback)
    while not rospy.is_shutdown():
        move_info = Twist()
        move_info.angular.z = 2*angular_multiplier
        if angular_multiplier == 0:
            move_info.linear.x = 0.3
        else:
            move_info.linear.x = 0
        pub.publish(move_info)
        rate.sleep()

def dist_from_obs(data):
    #takes in some data figures out if needs to move away
    data_length = len(data)
    individual_length = data_length/3

    left_scans = data[data_length-individual_length:data_length]
    forward_scans = data[individual_length:(data_length-individual_length)]
    right_scans = data[0 : individual_length]

    min_l_dist = min(left_scans)
    min_f_dist = min(forward_scans)
    min_r_dist = min(right_scans)

    return min_l_dist, min_f_dist, min_r_dist

def detect_obs(mins):
    min_l, min_f, min_r = mins

    f_blocked= False
    l_blocked= False
    r_blocked= False
    area_open= True

    if min_f <= 1:
        area_open = False
        f_blocked = True
    if min_l <= 0.2:
        area_open = False
        l_blocked = True
    if min_r <= 0.2:
        area_open = False
        r_blocked = True

    return area_open, l_blocked, f_blocked, r_blocked

def avoid_obs(obs_data, l_blocked, f_blocked, r_blocked):
    # -1 for rotate left
    #1 for rotate right
    #0 for forward

    if l_blocked == True and f_blocked == True and r_blocked == True:
        rotate = -1
    elif l_blocked == True and f_blocked == True:
        rotate = 1
    elif f_blocked == True and r_blocked == True:
        rotate = -1
    elif f_blocked == True:
        rotate = -1
    elif r_blocked == True:
        rotate = -1
    elif l_blocked == True:
        rotate = 1
    else:
        rotate = 0

    return rotate


if __name__ == '__main__':
    try:
        get_laser_data()
    except rospy.ROSInterruptException:
        pass
