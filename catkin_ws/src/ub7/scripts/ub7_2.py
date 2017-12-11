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



def pub_imgs(img):
    
    #cv2.imshow('yuv', img_yuv)
    
    #img_pub_hsv.publish(ros_img_hsv)
    ros_img = bridge.cv2_to_imgmsg(img)
    img_pub_yuv.publish(ros_img)
    
def findBaloons((cx,cy),(h,s,v)):
    global red_baloon
    global green_baloon
    global blue_baloon
    global purple_baloon
    global res_bgr
    
    #print (cx,cy)
    #print (h,s,v)
    
    if (h,s,v) == (0,0,0) or s < 125:
        return
    #print (cx, cy)
    #print (h,s,v)
    #print "---"
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(res_bgr, str((h,s,v)), (cx,cy), font, 0.25, (255,255,255), 1)
    
    
    if h > 170 or h < 10:
        red_baloon = (cx,cy)
        print "red = ", red_baloon
        cv2.putText(res_bgr, "red", (cx,cy+5), font, 0.25, (0,0,255), 1)
    if h > 65 and h < 75:
        green_baloon = (cx,cy)
        print "green = ", green_baloon
        cv2.putText(res_bgr, "green", (cx,cy+5), font, 0.25, (0,255,0), 1)
    if h > 115 and h < 125:
        blue_baloon = (cx,cy)
        print "blue = ", blue_baloon
        cv2.putText(res_bgr, "blue", (cx,cy+5), font, 0.25, (255,0,0), 1)
    if h > 125 and h < 140:
        purple_baloon = (cx,cy)
        print "purple = ", purple_baloon
        cv2.putText(res_bgr, "purple", (cx,cy+5), font, 0.25, (255,0,255), 1)
    
    

def handle_new_image(img_bgr):
    global res_bgr
    
    img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)


    lower_gray_yuv = np.array([50,0,0])
    upper_gray_yuv = np.array([200,255,255])
    mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)
    
    _, contours, _ = cv2.findContours(mask_yuv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    res_bgr = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)
    img_hsv = cv2.cvtColor(res_bgr, cv2.COLOR_BGR2HSV)
    
    for cont in contours:
        M = cv2.moments(cont)
        if M['m00'] <= 0:
            continue
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        findBaloons((cx,cy), img_hsv[cy][cx])
        #print img_hsv[cx][cy]
        
    #cv2.imwrite('res.png',res_bgr)
    #cv2.imwrite('hsv_res.png',img_hsv)

    print '-----------------------------done--------------------------------'
    pub_imgs(res_bgr)


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
    img_pub_yuv = rospy.Publisher("/mask/yuv",Image,queue_size=1)


if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
