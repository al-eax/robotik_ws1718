#!/usr/bin/env python
import rospy
import cv_bridge
import signal
import sys
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
from std_msgs.msg import Float32

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def init():
    global bridge
    rospy.init_node('foo', anonymous=True)
    rospy.Subscriber("/app/camera/color/image_raw",Image,cameraRawCallback)#subscribe to cars camera
    bridge = CvBridge() #create a opencv <->ros bridge to convert images


def calcVR_VT(cv_bin_image):
    model_points = np.array([(60,0,0), (60,20,0), (30,0,0), (30,20,0), (0,0,0) , (0,20,0)], dtype = "double")

    fx = 614.1699
    fy = fx
    cx = 329.9491
    cy = 237.2788

    camera_matrix = np.matrix(      [[fx,0,cx],
                                    [0,fy,cy],
                                    [0,0,1]], dtype = "double")
    k1 = 0.1115
    k2 = -0.1089
    t1 = 0
    t2 = 0

    distortion_params = np.array([[k1],[k2],[t1],[t2]], dtype = "double")

    image_points = np.array(getPoints(cv_binary_image),  dtype = "double")
    (retval, rvec, tvec) = cv2.solvePnP(model_points,image_points,camera_matrix,distortion_params)

    return (rvec, tvec)

#callback function to catch images from cars camera
def cameraRawCallback(data):
    print "got a new image"
    global cv_input_image
    global bridge
    cv_input_image = bridge.imgmsg_to_cv2(data, "bgr8") #convert ros image to opencv image
    cv_gray_image = cv2.cvtColor(cv_input_image, cv2.COLOR_BGR2GRAY) #convert to gray scale
    _,cv_binary_image = cv2.threshold(cv_gray_image,220,255,cv2.THRESH_BINARY) #binarize image into black and white

    (r,t) = calcVR_VT(cv_binary_image)

    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_gray.png",cv_gray_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_rgb.png",cv_input_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_binary.png",cv_binary_image)

    cv_detected_img = cv_input_image.copy()



def pubImages(img_gray, img_bin):
    global bridge
    gray_scale_pub = rospy.Publisher("/gray_scale_img",Image,queue_size = 10)
    bin_pub = rospy.Publisher("/binary_img",Image,queue_size = 10)

    ros_image_gray = bridge.cv2_to_imgmsg(img_gray, "mono8") #mono8 = 8bit,1 channel = grayscale
    ros_image_bin = bridge.cv2_to_imgmsg(img_bin, "mono8")

    gray_scale_pub.publish(ros_image_gray)
    bin_pub.publish(ros_image_bin)

def get2HighestIndex(l):
    m1 = max(l)
    m2 = max(x for x in l if x < m1)
    i1 = l.index(m1)
    i2 = l.index(m2)
    return (i1,i2)


def get2LowestIndex(l):
    m1 = min(l)
    m2 = min(x for x in l if x > m1)
    i1 = l.index(m1)
    i2 = l.index(m2)
    return (i1,i2)

def get2MidIndex(l):
    (i1,i2) = get2HighestIndex(l)
    (i3,i4) = get2LowestIndex(l)
    r = []
    for i in range(6):
        if i != i1 and i != i2 and i != i3 and i != i4:
            r.append(i)
    return (r[0],r[1])



def getPoints(bin_image):
    image, contours, hierarchy = cv2.findContours(bin_image,1, 2)#find contours in image
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[0:6] #take the 6 biggest contours
    points = []
    print len(contours)
    xs = []
    ys = []
    for contour in contours:
        #http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
        M = cv2.moments(contour)
        if M['m00'] != 0:#calculate center of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            xs.append(cx)
            ys.append(cy)
            points.append((cx,cy))

    (u1,u2) = get2LowestIndex(ys)
    if xs[u1] < xs[u2]:
        upper_left = u1
        upper_right =  u2
    else:
        upper_right = u1
        upper_left = u2

    (l1,l2) = get2HighestIndex(ys)
    if xs[l1] < xs[l2]:
        lower_left = l1
        lower_right = l2
    else:
        lower_right = l1
        lower_left = l2

    (m1,m2) = get2MidIndex(ys)
    print (m1,m2)
    if xs[m1] < xs[m2]:
        mid_left = m1
        mid_right = m2
    else:
        mid_left = m2
        mid_right = m1

    sorted_points = [points[upper_left], points[upper_right], points[mid_left], points[mid_right],
            points[lower_left], points[lower_right]]
    print sorted_points
    return sorted_points

if __name__ == '__main__':
    try:

        init()
        rospy.spin()
        '''
        img = cv2.imread("/home/alex/repos/robotik_ws1718/ub3/cam_image_rgb.png")
        cv_gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to gray scale
        _,cv_binary_image = cv2.threshold(cv_gray_image,220,255,cv2.THRESH_BINARY)#+cv2.THRESH_OTSU) #binarize image into black and white
        points = getPoints(cv_binary_image)
        '''



    except rospy.ROSInterruptException:
        pass
