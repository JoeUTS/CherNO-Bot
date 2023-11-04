#!/usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

# what does anonymous do?
rospy.init_node('move_to_ready_pose', anonymous=True)

move_group = MoveGroupInterface("head", "base_link")

joints = ["head_pan_joint", "head_tilt_joint"]

pose = [0.0, -1.57]

while not rospy.is_shutdown():

    result = move_group.moveToPose(joints, pose, 0.02)

    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Traj successfully executed!")
        else:
            rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
    else:
        rospy.logger("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()
