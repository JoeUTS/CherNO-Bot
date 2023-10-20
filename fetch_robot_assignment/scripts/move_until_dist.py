#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# can rename this to whatever I want my function to be
def move_until_dist(laser):
    # Fetch robot base_laser is 220deg with values every 1/3deg.
    # total values:   660
    # mid value:      330
    # max range:      25m
    # min range:      0.05m

    # Move fast until 1m away
    if (laser.ranges[330] > 1):
        move_msg.linear.x = 0.5
        move_msg.angular.z = 0
    # Move backwards if less than 10cm away
    if (laser.ranges[330] < 0.15):
        move_msg.linear.x = -0.5
        move_msg.angular.z = 0
    # stop if 25cm away
    elif (laser.ranges[330] < 0.25):
        move_msg.linear.x = 0

        # rotate to face table then close
        if (laser.ranges[465] > 1.1*laser.ranges[195]):
            move_msg.angular.z = 1
        elif (laser.ranges[195] > 1.1*laser.ranges[465]):
            move_msg.angular.z = -1
        else:
            move_msg.angular.z = 0
            rospy.signal_shutdown("I have entered an argument so this works")
    # move slow under 1m distance
    else:
        move_msg.linear.x = 0.25
        move_msg.angular.z = 0

    pub_cmd_vel.publish(move_msg)

# initilise the node and set the communication rate?
rospy.init_node('move_until_distance')

# Publish the message to move the robot
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

move_msg = Twist()


# Subscribe to the laser ranges and run the function with it
rospy.Subscriber('/base_scan', LaserScan, move_until_dist)

# loop the code until shutdown
rospy.spin()


