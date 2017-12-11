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

RL_GREEN = (2.29, 1.14)
RL_RED = (3.55, 3.03)
RL_BLUE = (4.18, 1.77)
RL_PURPLE = (2.29, 2.4)


IMG_RED = (0,0)
IMG_BLUE = (0,0)
IMG_GREEN = (0,0)
IMG_PURBLE = (0,0)



from numpy import *
from math import sqrt

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    print t

    return R, t

def pub_imgs(img_yuv):
    global img_pub_yuv
    global img_pub_hsv
    global img_pub_bgr


    ros_img_yuv = bridge.cv2_to_imgmsg(img_yuv)
    img_pub_yuv.publish(ros_img_yuv)

def findBaloons((cx,cy),(h,s,v)):
    global red_baloon
    global green_baloon
    global blue_baloon
    global purple_baloon

    print (h,s,v)

def getBln(h):
    if h > 170 or h < 10:
        return "red"
    if h > 65 and h < 75:
        return "green"
    if h > 115 and h < 125:
        return "blue"
    if h > 125 and h < 140:
        return "purple"



def handle_new_image(img_bgr):
    global IMG_RED
    global IMG_BLUE
    global IMG_GREEN
    global IMG_PURBLE

    img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)


    lower_gray_yuv = np.array([50,0,0])
    upper_gray_yuv = np.array([200,255,255])
    mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)

    _, contours, _ = cv2.findContours(mask_yuv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    res_yuv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)
    img_hsv = cv2.cvtColor(res_yuv, cv2.COLOR_BGR2HSV)

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[0:10]

    comps = []
    for cont in contours:
        M = cv2.moments(cont)
        area = cv2.contourArea(cont,True)
        if area == 0:
            continue
        l = cv2.arcLength(cont,True)
        comps.append((l**2 / area , cont))
    comps_sorted = sorted(comps,reverse = True, key=lambda tup: tup[0])[0:4]


    for (cmp, cont) in comps_sorted:
        #print np.mean(bs)
        M = cv2.moments(cont)
        if M["m00"] <= 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        H = img_hsv[cY][cX][0]
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img_bgr, getBln(H) , (cX,cY+5), font, 0.25, (255,255,255), 1)



        if getBln(H) == "red":
            IMG_RED = (cx,cy)
        if getBln(H) == "blue":
            IMG_BLUE = (cx,cy)
        if getBln(H) == "green":
            IMG_GREEN = (cx,cy)
        if getBln(H) == "purple":
            IMG_PURBLE = (cx,cy)

    cv2.imwrite("img_bgr.png",img_bgr)



    #print "foo"
    #pub_imgs(res_yuv)


def img_to_rl(IMG_BLNS,RL_BLNS):


#callback function
def camCallback(data):
    #print "new image"
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.imwrite('rgb.png', cv_img)
    handle_new_image(cv_img)

def init():
    global img_pub_hsv
    global img_pub_bgr
    global img_pub_yuv
    global bridge

    rospy.init_node('foobar', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber("/usb_cam/image_rect_color", Image, camCallback, queue_size=1)
    #img_pub_bgr = rospy.Publisher("/mask/bgr",Image,queue_size=1)
    #img_pub_hsv = rospy.Publisher("/mask/hsv",Image,queue_size=1)
    img_pub_yuv = rospy.Publisher("/mask/yuv",Image,queue_size=1)


if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
