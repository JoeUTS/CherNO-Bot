#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image_pub = rospy.Publisher("/saved_image", Image, queue_size=10)

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

def image_read(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Display the image
    cv2.imshow("Received Image", cv_image)
    cv2.waitKey(1)

    # Save the image
    img_name = "saved_image.jpg"
    cv2.imwrite(img_name, cv_image)
    rospy.loginfo("Saved image to %s", img_name)

    # Publish the saved image
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

    red_object_corners_detector(cv_image)

def red_object_corners_detector(data):
    #try:
    #    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError as e:
    #    print(e)
    #    return
    cv_image = data
    # Filter for red color
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([160, 100, 100])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    
    mask = mask1 + mask2

    # Find contours of the red object
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        corners = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]

        # Print the detected corners
        print("Detected corners:", corners)
        
        for corner in corners:
            cv2.circle(cv_image, corner, 5, (0, 255, 0), -1)

        cv2.imshow('Detected Corners', cv_image)
        cv2.waitKey(1)
        harris_corner_detection(cv_image, corners)

def harris_corner_detection(img, corners):
    # Extracting the region of interest based on the corners
    # ... [previous code] ...

    # Compute Lx and Vc as in MATLAB script
    f = 400.0
    p = img.shape[0] / 2  # Assuming image height for p, can change to width if needed
    Z = 3.0
    l = 0.1  # lambda 

    Target = np.array([
        [446, 946],
        [446, 446],
        [946, 946],
        [946, 446]
    ])
    Obs = np.array([
        [313.5, 547.7],
        [599.0, 140.2],
        [720.9, 833.3],
        [1006.4, 425.7]
    ])
    
    xy = (Target - p) / f
    Obsxy = (Obs - p) / f

    n = Target.shape[0]
    Lx_list = [FuncLx(xy[i, 0], xy[i, 1], Z) for i in range(n)]
    Lx = np.vstack(Lx_list)

    e2 = Obsxy - xy
    e = e2.T.reshape(-1)
    de = -e * l

    Lx2 = np.linalg.pinv(Lx.T.dot(Lx)).dot(Lx.T)
    Vc = -l * Lx2.dot(e)
    
    print("Vc:", Vc)

if __name__ == '__main__':
    rospy.init_node('image_processing_node', anonymous=True)
    rate = rospy.Rate(5)
    #rospy.spin()
    #cv2.destroyAllWindows()

while not rospy.is_shutdown():
    # You can modify the callback according to which function you want to prioritize
    # Currently, both functions will process the image, but `image_read` will process it first.
    image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_read)
    #image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, red_object_corners_detector)
    rate.sleep()

if cbf == 0:
    cv2.destroyAllWindows()