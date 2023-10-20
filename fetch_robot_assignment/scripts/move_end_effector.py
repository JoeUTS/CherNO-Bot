#!/usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# what does anonymous do?
rospy.init_node('move_to_ready_pose', anonymous=True)

move_group = MoveGroupInterface("arm", "base_link")

gripper_frame = 'gripper_link'

gripper_poses = [Pose(point(0.809, 0.0, 1.358), Quaternion(0.01, 0.0, 0.0, 1.0)),
                 Pose(point(0.809, 0.0, 1.136), Quaternion(0.01, 0.0, 0.0, 1.0))]

gripper_pose_stamped = poseStamped()
gripper_pose_stamped.header.frame_id = 'base_link'

while not rospy.is_shutdown():
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()

        gripper_pose_stamped.pose = pose

        redult = move_group.moveToPose(gripper_pose_stamped, gripper_frame)

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Traj successfully executed!")
            else:
                rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
        else:
            rospy.logger("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()
