#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import *
from math import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose

import rospy
import roslib


RL_GREEN = np.array((2.29, 1.14))
RL_RED =  np.array((3.55, 3.03))
RL_BLUE =  np.array((4.18, 1.77))
RL_PURPLE =  np.array((2.29, 2.4))

first_yaw = -10
font = cv2.FONT_HERSHEY_SIMPLEX


# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column
# this function is taken from the link in the assignment
# but it was slightly modified
def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = dot(transpose(AA), BB)

    U, S, Vt = linalg.svd(H)

    R = dot(Vt.T, U.T)

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       # not sure this does in 2D what it is supposed to do in 3D so commented out
       # did not cause problems so far either
       #Vt[1,:] *= -1
       #R = dot(Vt.T, U.T)

    scale = linalg.norm((centroid_B - RL_RED), 2) / linalg.norm((centroid_A - red_baloon), 2)
    t = dot(-R, centroid_A.T) + centroid_B.T
    return R, t, scale    
    

def pub_imgs(img):
    ros_img = bridge.cv2_to_imgmsg(img)
    img_pub.publish(ros_img)

# this function sets red, green, blue, and purple baloon vars respectively
def findBaloons((cx,cy),(h,s,v)):
    global red_baloon
    global green_baloon
    global blue_baloon
    global purple_baloon
    global res_bgr
    
    # filter out black and colorless spots
    if (h,s,v) == (0,0,0) or s < 115:
        return

    #cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)

    # set proper var if hue conditions are met
    if h > 170 or h < 10:
        red_baloon = (cx,cy)
        print "red = ", red_baloon
        cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)
        cv2.putText(res_bgr, "red", (cx,cy+5), font, 0.25, (0,0,255), 1)
    if h > 65 and h < 75:
        green_baloon = (cx,cy)
        print "green = ", green_baloon
        cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)
        cv2.putText(res_bgr, "green", (cx,cy+5), font, 0.25, (0,255,0), 1)
    if h > 115 and h < 125:
        blue_baloon = (cx,cy)
        print "blue = ", blue_baloon
        cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)
        cv2.putText(res_bgr, "blue", (cx,cy+5), font, 0.25, (255,0,0), 1)
    if h > 125 and h < 140:
        purple_baloon = (cx,cy)
        print "purple = ", purple_baloon
        cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)
        cv2.putText(res_bgr, "purple", (cx,cy+5), font, 0.25, (255,0,255), 1)

# function given in assignment
def yaw_to_quaternion(yaw):
    return Quaternion(0,0,math.sin(yaw / 2) , math.cos(yaw / 2))


def handle_new_image(img_bgr):
    global res_bgr
    global first_yaw

    # convert bgr image to YUV color space to get a mask
    img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)

    lower_gray_yuv = np.array([50,0,0])
    upper_gray_yuv = np.array([200,255,255])
    mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)

    # using the mask find contours in the image
    _, contours, _ = cv2.findContours(mask_yuv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # create an HSV image with the mask already used on it
    res_bgr = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)
    img_hsv = cv2.cvtColor(res_bgr, cv2.COLOR_BGR2HSV)

    # find center points for all contours and check if they match a color
    for cont in contours:
        M = cv2.moments(cont)
        if M['m00'] <= 0:
            continue
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        findBaloons((cx,cy), img_hsv[cy][cx])

    # save baloon coordinates in an array
    img_coords = [np.array(red_baloon), np.array(green_baloon) ,np.array(blue_baloon), np.array(purple_baloon) ]
    rl_coords = [RL_RED, RL_GREEN ,RL_BLUE, RL_PURPLE]

    # get rotation matrix, translation vector and scale
    R, t, scale = rigid_transform_3D(np.array(img_coords),np.array(rl_coords))

    # yaw is atan(sin/cos) for 360°
    yaw = math.atan2(R[1][0], R[0][0])
    if first_yaw == -10:
        first_yaw = yaw
    # subtract first yaw to get an accurate idea of where is forward
    yaw -= first_yaw
    # some border cases
    if yaw > math.pi:
        yaw -= 2* math.pi
    elif yaw < -math.pi:
        yaw += 2*math.pi
        
    # center point of image is position
    img_c_x = img_hsv.shape[1] / 2
    img_c_y = img_hsv.shape[0] / 2
    
    # calculate rl position using rotation matrix, translation vector and scale
    loc_vec = np.array([img_c_x,img_c_y])
    rl_loc = (dot(R,loc_vec) + t) * scale
    
    cv2.putText(res_bgr, "position = " + str(rl_loc), (loc_vec[0], loc_vec[1] - 5), font, 0.25, (255,255,255), 1)
    cv2.putText(res_bgr, "yaw = " + str(yaw), (loc_vec[0], loc_vec[1] + 5), font, 0.25, (255,255,255), 1)
    
    pub_imgs(res_bgr)
    publ_odom(rl_loc, yaw)



def publ_odom(rl_loc, yaw):
    print "pub"
    global odom_pub
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.pose.pose = Pose(Point(rl_loc[0], rl_loc[1], 0) , yaw_to_quaternion(yaw) )
    odom_pub.publish(msg)


#callback function
def camCallback(data):
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    handle_new_image(cv_img)

def init():
    global img_pub
    global bridge
    global odom_pub

    rospy.init_node('foobar', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber("/usb_cam/image_rect_color", Image, camCallback, queue_size=1)
    odom_pub = rospy.Publisher('/my_odom', Odometry)
    img_pub = rospy.Publisher("/mask/yuv",Image,queue_size=1)


if __name__ == '__main__':
    try:
        init()
        print "init"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
