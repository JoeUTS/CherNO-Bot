#!/usr/bin/env python

# I dont remember which libraries I have downloaded and which are ROS
# Libraries needed:
# -moveit_python
# -OpenCV
# -CV_bridge
# -numpy

import rospy
import cv2
import numpy as np
import tf
import math
import actionlib
from moveit_python import MoveGroupInterface
from cv_bridge import CvBridge
from moveit_msgs.msg import MoveItErrorCodes
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# There are better ways to do this. This is the easy way.

# Classes for moving head and torso
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Functions!!!
# Moves to home pose (tucked in arm)
def pose_home():
    # Let everyone know that this is running
    rospy.loginfo("Moving to home pose")

    # Defining what we want to move
    move_group = MoveGroupInterface("arm", "base_link")
    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [1.32, 1.4, -0.2, 1.7, 0.0, 1.57, 0.0]
    
    # Move the arm. Argument:(joints, joint angle(rad), tollerance(rad?))
    move_arm = move_group.moveToJointPosition(joints, pose, 0.02)

    # Verification
    ArmErrorChecking(move_arm, move_group)

# Moves to ready pose (arm up and to the side)``
def pose_ready():
    # Just the same as the above function
    move_group = MoveGroupInterface("arm", "base_link")
    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [1.58, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]

    # Move arm
    move_arm = move_group.moveToJointPosition(joints, pose, 0.02)

    # Verification
    ArmErrorChecking(move_arm, move_group)

# Set the laser values as a global array to access later
def LaserValues(laser):
    global laser_array
    laser_array = laser.ranges

# Lx matrix for visual servoing
def FuncLx(x, y, Z):
    Lx = np.zeros((2, 6))

    Lx[0, 0] = -1.0 / Z
    Lx[0, 1] = 0
    Lx[0, 2] = x / Z
    Lx[0, 3] = x * y
    Lx[0, 4] = -(1 + x**2)
    Lx[0, 5] = y

    Lx[1, 0] = 0
    Lx[1, 1] = -1.0 / Z
    Lx[1, 2] = y / Z
    Lx[1, 3] = 1 + y**2
    Lx[1, 4] = -x * y
    Lx[1, 5] = -x

    return Lx

# Read image and convert to CV2 compatable data
def ImageConvert(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the image if waitKey == 0
    cv2.imshow("Received Image", cv_image)
    cv2.waitKey(1)

    return cv_image

# Finds the corners for a coloured cube
def corners_detector(data, colour):
    cv_image = data

    # get hue saturation value data
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 0 = red
    # 1 = green
    if colour == 0:
        print("Finding red corners")
        # Find the red
        # Needs two sets of values due to red being split on the hue scale
        # array arguments: (hue, saturation, value)
        lower_colour = np.array([0, 100, 125])
        upper_colour = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_colour, upper_colour)
        
        lower_colour = np.array([160, 100, 125])
        upper_colour = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_colour, upper_colour)
        
        mask = mask1 + mask2
    else:
        print("Finding green corners")
        # Find the Green
        lower_colour = np.array([35, 100, 125])
        upper_colour = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_colour, upper_colour)

    cv2.imshow("Received Image", mask)
    cv2.waitKey(1)

    # Find contours of the object
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        corners = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]

        # Print the detected corners
        print("Detected corners:", corners)
        
        for corner in corners:
            #colour values are:(blue, green, red)
            cv2.circle(cv_image, corner, 2, (255, 0, 0), -1)

        cv2.imshow('Detected Corners', cv_image)
        cv2.waitKey(1)
        
        return corners

# find the velocities
def CornersToTransform(img, corners, slot):
    # Extracting the region of interest based on the corners
    # ... [previous code] ...

    # Compute Lx and Vc as in MATLAB script

    # Use intrinsic properties
    global intrinsics
    focalX = intrinsics.K[0]
    focalY = intrinsics.K[4]
    px = intrinsics.K[2]
    py = intrinsics.K[5]
    lamb = 1
    # Fetch depth values unobtainable as per Manufacturer. Cheating instead
    # We will get the transform of the camera (head) and take the known global transform of the rod and the slot
    global headTransformGlobal
    slotXYZ = [1.3,-0.2,0.738]
    rodXYZ = [1.5,0.2,0.755]
    if slot == 0:
        cartesianDistances = [slotXYZ[0]-headTransformGlobal[0,0], slotXYZ[1]-headTransformGlobal[0,1], slotXYZ[2]-headTransformGlobal[0,2]]
        Z = math.sqrt((cartesianDistances[0]**2) + (cartesianDistances[1]**2) + (cartesianDistances[2]**2))
    else:
        cartesianDistances = [rodXYZ[0]-headTransformGlobal[0,0], rodXYZ[1]-headTransformGlobal[0,1], rodXYZ[2]-headTransformGlobal[0,2]]
        Z = math.sqrt((cartesianDistances[0]**2) + (cartesianDistances[1]**2) + (cartesianDistances[2]**2))

    # Target and opbserved points
    Target = np.array([
        [446, 946],
        [446, 446],
        [946, 946],
        [946, 446]
    ])
    Obs = np.array(corners)

    # n is based on the size of the target array
    n = Target.shape[0]
    
    # Generate 
    xy = np.zeros((n,2))
    Obsxy = np.zeros((n,2))
    for i in range(n):
        xy[i,0] = (Target[i,0] - px) / focalX
        xy[i,1] = (Target[i,1] - py) / focalY
        Obsxy[i,0] = (Obs[i,0] - px) / focalX
        Obsxy[i,1] = (Obs[i,1] - py) / focalY
    
    # Calculate Lx values for each point and flip the array to be horizontal cause maths
    Lx = [FuncLx(xy[i, 0], xy[i, 1], Z) for i in range(n)]
    Lx = np.vstack(Lx)

    # Calculate error and drop to 1,n array
    eIntermediary = Obsxy - xy
    e = eIntermediary.T.reshape(-1)

    # Pseudoinverse the feature jacobian and calculate velocities
    Lx2 = np.linalg.pinv(Lx.T.dot(Lx)).dot(Lx.T)
    Vc = -lamb * Lx2.dot(e)
    
    print("Vc:", Vc)
    return -Vc

# Pick and place
def EndEffectorIK(transform):
    # Convert RPY to Quaternion
    quaternion = quaternion_from_euler(transform[3],transform[4],transform[5])
    print('quaternions: %s', quaternion)

    move_group = MoveGroupInterface("arm", "base_link")
    gripper_frame = 'gripper_link'

    # Set up message to move arm
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = Pose(Point(transform[0], transform[1], transform[2]), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

    # Move the arm
    result = move_group.moveToPose(gripper_pose_stamped, gripper_frame)

    # Verification
    ArmErrorChecking(result, move_group)

# Error check arm movements
def ArmErrorChecking(result, move_group):
    # Get verification that arm has moved
    if result:
        # Did we succeed?
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Movement Successful")
        else:
            # Error state 4 == control failed
            rospy.logerr("Error! state: %s", move_group.get_move_action().get_state())
    else:
        # No responce is bad
        rospy.logger("Panic, no responce. Is the fetch_MoveIt_config running?")

def FindTransform(target_frame, source_frame):
    transformResult = np.zeros((2,4))
    now = rospy.Time.now()
    # arguments(target_frame, source_frame, time, timeout(seconds))
    listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(5))
    # arguments(target_frame, source_fram, time)
    transform = listener.lookupTransform(target_frame, source_frame, now)
    # Data is being weird. Cant extract single values from transform array
    # Doing weird work around
    intermediary = transform[0]
    for i in range(3):
        transformResult[0,i] = intermediary[i]
    intermediary = transform[1]
    for i in range(4):
        transformResult[1,i] = intermediary[i]
    print('headTransformGlobal: %s', transformResult)
    return transformResult

# Initiate Node
rospy.init_node('main', anonymous=True)

# Main function
rospy.loginfo("Main Program Started")

# Get into starting position
#pose_ready()
pose_home()

# Move forward until in position
rospy.loginfo("Moving")

# set up global varaible for the laser ranges
# LiDAR on the fetch has 660 points
global laserArray
laser_array = [0]*660

inPosition = 0

while inPosition == 0:
    # Read the 2D LiDAR and run the function to control the speed
    # Having this here means it updates during the loop and then stops when not needed, saving procesing power
    rospy.Subscriber('/base_scan', LaserScan, LaserValues)

    # Declaring our move message
    move_msg = Twist()

    # Fetch Robot LiDAR info:
    # Fetch robot base_laser is 220deg with values every 1/3deg.
    # total values:   660
    # mid value:      330
    # max range:      25m
    # min range:      0.05m

    # Move fast until 1m away
    if (laser_array[330] > 1):
        move_msg.linear.x = 0.5
        move_msg.angular.z = 0

    # Move backwards if less than 10cm away
    if (laser_array[330] < 0.15):
        move_msg.linear.x = -0.5
        move_msg.angular.z = 0

    # stop if 25cm away
    elif (laser_array[330] < 0.25):
        move_msg.linear.x = 0

        # rotate to face table then close loop
        if (laser_array[465] > 1.1*laser_array[195]):
            move_msg.angular.z = 1
        elif (laser_array[195] > 1.1*laser_array[465]):
            move_msg.angular.z = -1
        else:
            move_msg.angular.z = 0
            inPosition = 1

    # move slow under 1m distance
    else:
        move_msg.linear.x = 0.25
        move_msg.angular.z = 0

    # Publish the message to move the robot
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_cmd_vel.publish(move_msg)

rospy.loginfo("In Position")

# NEED SOMETHING HERE TO MOVE THE HEAD TO FACE DOWN
head_action = PointHeadClient()
torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
head_action.look_at(0.4, 0.0, 0.7, "/base_link")
torso_action.move_to([0.2, ])

# Visual Servoing
bridge = CvBridge()

# Get transforms
listener = tf.TransformListener()

# Get saved intrinsic perameters
rospy.loginfo("Loading intrinsics")
global intrinsics
intrinsics = rospy.wait_for_message("/head_camera/rgb/camera_info", CameraInfo, timeout=None)
rospy.loginfo("Intrinsics loaded")

# Get the image + Convert to compatable data for CV2
rospy.loginfo("Converting Image for CV2")
rawImage = rospy.wait_for_message("/head_camera/rgb/image_raw", Image, timeout=None)
convertedImage = ImageConvert(rawImage)
rospy.loginfo("Image converted for CV2")

# Find head transform in relation to the global transform
# Treating /odom as global transform as Fetch robot loads @ [0,0,0]
# Will use this to cheat the distance as Fetch RGB-D depth values dont work in Gazebo
global headTransformGlobal
headTransformGlobal = np.zeros((2,4))
headTransformGlobal = FindTransform("/odom", "/head_camera_rgb_frame")
baseTransform = np.zeros((2,4))
baseTransform = FindTransform("base_link", "/head_camera_rgb_frame")
baseEuler = euler_from_quaternion([baseTransform[1,0], baseTransform[1,1], baseTransform[1,2], baseTransform[1,3]])


# Find the objects
rospy.loginfo("Finding rod")
redCorners = corners_detector(convertedImage, 0)
rodTransform = CornersToTransform(convertedImage, redCorners, 0)
rospy.loginfo("Rod found")

# Convert
for i in range(3):
    rodTransform[i] = rodTransform[i] + baseTransform[0,i]
    rodTransform[3+i] = rodTransform[3+i] + baseEuler[i]

print('rodTransform: %s', rodTransform)

rospy.loginfo("Finding slot")
greenCorners = corners_detector(convertedImage, 1)
slotTransform = CornersToTransform(convertedImage, greenCorners, 1)
rospy.loginfo("Slot found")

test = [0.3, 0.3, 1.3, 0, 0, 0]

# pick and place
# Move to rod
#EndEffectorIK(rodTransform)
EndEffectorIK(test)

# Close gripper

# Move to intermediary pose

# Move to slot (likely will need to move to an intermediary position above the slot first)
#EndEffectorIK(slotTransform)

# Open gripper

# Move to intermediary pose



