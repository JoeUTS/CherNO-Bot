#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# can rename this to whatever I want my function to be
def forward_for_time(event):
    print "Stopping"
    # this is shutting down this node?
    rospy.signal_shutdown("I have entered an argument so this works")

# initilise the node and set the communication rate
rospy.init_node('move_robot_forward')
rate = rospy.Rate(5)
# set a timer which calls a function when activated
rospy.Timer(rospy.Duration(1), forward_for_time)

# Publish the message to move the robot
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_msg = Twist()
move_msg.linear.x = 0.5

# will run this loop until shutdown
while not rospy.is_shutdown():
    pub_cmd_vel.publish(move_msg)
    rate.sleep()
