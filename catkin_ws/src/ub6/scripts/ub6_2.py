#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import cv2

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

IMAGE_PATH = '../../../../ub6/'

img_bgr = cv2.imread(IMAGE_PATH + 'cam_car121.png')
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)

lower_gray_bgr = np.array([200,200,200])
upper_gray_bgr = np.array([255,255,255])
mask_bgr = cv2.inRange(img_bgr, lower_gray_bgr, upper_gray_bgr)
res_bgr = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_bgr)

lower_gray_hsv = np.array([0,0,200])
upper_gray_hsv = np.array([255,30,255])
mask_hsv = cv2.inRange(img_hsv, lower_gray_hsv, upper_gray_hsv)
res_hsv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_hsv)

lower_gray_yuv = np.array([200,0,0])
upper_gray_yuv = np.array([255,255,255])
mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)
res_yuv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)

#cv2.imshow('bgr',img_bgr)
#cv2.imshow('hsv', img_hsv)
#cv2.imshow('yuv', img_yuv)

cv2.imshow('mask_bgr',mask_bgr)
cv2.imshow('mask_hsv',mask_hsv)
cv2.imshow('mask_yuv',mask_yuv)

#cv2.imshow('res_bgr',res_bgr)
#cv2.imshow('res_hsv',res_hsv)
#cv2.imshow('res_yuv',res_yuv)

cv2.waitKey(0)
cv2.destroyAllWindows()

#cv2.imwrite(IMAGE_PATH + 'res_bgr.png', res_bgr)
#cv2.imwrite(IMAGE_PATH + 'res_hsv.png', res_hsv)
#cv2.imwrite(IMAGE_PATH + 'res_yuv.png', res_yuv)
