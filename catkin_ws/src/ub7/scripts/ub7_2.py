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



def pub_imgs(img_yuv):
    global img_pub_yuv
    global img_pub_hsv
    global img_pub_bgr
    
    #cv2.imshow('bgr', img_bgr)
    #cv2.imshow('hsv', img_hsv)
    #cv2.imshow('yuv', img_yuv)

    #ros_img_bgr = bridge.cv2_to_imgmsg(img_bgr)
    #img_pub_bgr.publish(ros_img_bgr)
    #ros_img_hsv = bridge.cv2_to_imgmsg(img_hsv)
    #img_pub_hsv.publish(ros_img_hsv)
    ros_img_yuv = bridge.cv2_to_imgmsg(img_yuv)
    img_pub_yuv.publish(ros_img_yuv)
    
def findBaloons((cx,cy),(h,s,v)):
    global red_baloon
    global green_baloon
    global blue_baloon
    global purple_baloon
    
    print (h,s,v)
    
    if h > 340 and h < 20:
        red_baloon = (cx,cy)
        print red_baloon
    if h > 100 and h < 140:
        green_baloon = (cx,cy)
        print green_baloon
    if h > 220 and h < 260:
        blue_baloon = (cx,cy)
        print blue_baloon
    if h > 280 and h < 320:
        purple_baloon = (cx,cy)
        print purple_baloon
    
    

def handle_new_image(img_bgr):
    
    
    #img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)

    #lower_gray_bgr = np.array([200,200,200])
    #upper_gray_bgr = np.array([255,255,255])

    #mask_bgr = cv2.inRange(img_bgr, lower_gray_bgr, upper_gray_bgr)
    #res_bgr = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_bgr)

    #lower_gray_hsv = np.array([0,0,200])
    #upper_gray_hsv = np.array([255,30,255])

    #mask_hsv = cv2.inRange(img_hsv, lower_gray_hsv, upper_gray_hsv)
    #res_hsv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_hsv)

    lower_gray_yuv = np.array([50,0,0])
    upper_gray_yuv = np.array([200,255,255])
    mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)
    
    _, contours, _ = cv2.findContours(mask_yuv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    res_yuv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)
    img_hsv = cv2.cvtColor(res_yuv, cv2.COLOR_BGR2HSV)
    
    for cont in contours:
        M = cv2.moments(cont)
        if M['m00'] <= 0:
            continue
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        findBaloons((cx,cy),img_hsv[cx][cy])
        #print img_hsv[cx][cy]
    

    pub_imgs(res_yuv)


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
