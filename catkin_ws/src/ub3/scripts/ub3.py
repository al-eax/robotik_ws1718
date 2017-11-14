#!/usr/bin/env python
import rospy
import cv_bridge
import signal
import sys
import cv2
import numpy as np
import math

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
    rospy.init_node('foo2', anonymous=True)
    #/app/camera/rgb/image_color 104
    rospy.Subscriber("/app/camera/rgb/image_color",Image,cameraRawCallback)#subscribe to cars camera
    bridge = CvBridge() #create a opencv <->ros bridge to convert images

#
def drawAndSaveCoordsOnImg(cv_img, points):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv_img_tmp = cv_img.copy()
    for p in points:
        cv2.circle(cv_img_tmp,p, 1, (0,0,255), 1)
        cv2.putText(cv_img_tmp,str(p),p, font, 0.3,(0,255,0),1,cv2.LINE_AA)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_detected.png",cv_img_tmp)

#calculate the rvec,
def calcVR_VT(cv_bin_image):
    #3d world object coordinates, left handed coordsystem
    model_points = np.array([(0,0,0), (20,0,0), (0,20,0), (20,20,0), (0,40,0) , (40,40,0)], dtype = "double")

    #values from assignment
    fx = 614.1699
    fy = fx
    cx = 329.9491
    cy = 237.2788

    camera_matrix = np.matrix(      [[fx,0,cx],
                                    [0,fy,cy],
                                    [0,0,1]], dtype = "double")
    #values from assignment
    k1 = 0.1115
    k2 = -0.1089
    t1 = 0
    t2 = 0

    distortion_params = np.array([[k1],[k2],[t1],[t2]], dtype = "double")

    points = getPoints(cv_bin_image)# get 2d image object coortinates
    print "white points:" , points
    image_points = np.array(points,  dtype = "double")

    #get
    (_, rvec, tvec) = cv2.solvePnP(model_points,image_points,camera_matrix,distortion_params)
    return (rvec, tvec, points)

# rad to deg
def deg(a):
    return a * 180/math.pi

def processImage(cv_input_image):
    print "==========================[ new image ]====================================="
    cv_gray_image = cv2.cvtColor(cv_input_image, cv2.COLOR_BGR2GRAY) #convert to gray scale
    _, cv_binary_image = cv2.threshold(cv_gray_image,220,255,cv2.THRESH_BINARY) #binarize image into black and white

    (r,t, points) = calcVR_VT(cv_binary_image)

    print "rvec" ,r
    print "tvec" , t

    rotM,_ = cv2.Rodrigues(r)

    inv_rmat = rotM.T
    inv_tvec = -inv_rmat * t

    print  "inverse of Homogeneous", inv_tvec

    yaw = math.atan2(inv_rmat[1][0] ,inv_rmat[0][0])
    pitch = math.atan2(-inv_rmat[2][0], math.sqrt(inv_rmat[2][1]**2 + inv_rmat[2][2]**2))
    roll = math.atan2(inv_rmat[2][1], inv_rmat[2][2])

    print deg(yaw), deg(pitch) ,deg(roll)

    #save the images
    drawAndSaveCoordsOnImg(cv_input_image,points)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_gray.png",cv_gray_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_rgb.png",cv_input_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_binary.png",cv_binary_image)

#callback function to catch images from cars camera
def cameraRawCallback(data):
    global cv_input_image
    global bridge
    print "got a new image"
    cv_input_image = bridge.imgmsg_to_cv2(data, "bgr8") #convert ros image to opencv image
    processImage(cv_input_image)


def pubImages(img_gray, img_bin):
    global bridge
    gray_scale_pub = rospy.Publisher("/gray_scale_img",Image,queue_size = 10)
    bin_pub = rospy.Publisher("/binary_img",Image,queue_size = 10)

    ros_image_gray = bridge.cv2_to_imgmsg(img_gray, "mono8") #mono8 = 8bit,1 channel = grayscale
    ros_image_bin = bridge.cv2_to_imgmsg(img_bin, "mono8")

    gray_scale_pub.publish(ros_image_gray)
    bin_pub.publish(ros_image_bin)

#get the highest two element indices from a list
# [20,1,2,3,10,15] -> (0,4)
def get2HighestIndices(l):
    m1 = max(l)
    m2 = max(x for x in l if x < m1)
    i1 = l.index(m1)
    i2 = l.index(m2)
    return (i1,i2)

#get the lowest two element indices from a list
# [20,1,3,4,10,15] -> (1,2)
def get2LowestIndices(l):
    m1 = min(l)
    m2 = min(x for x in l if x > m1)
    i1 = l.index(m1)
    i2 = l.index(m2)
    return (i1,i2)

#dirty way, sorry
# get the two mid element indices from list with length of 6
# [20,1,3,4,10,15] -> (3,4)
def get2MidIndices(l):
    (i1,i2) = get2HighestIndices(l)
    (i3,i4) = get2LowestIndices(l)
    r = []
    for i in range(len(l)):
        if i != i1 and i != i2 and i != i3 and i != i4:
            r.append(i)
    return (r[0],r[1])

# returns the 2d image coordinates of 6 points from the binary image
def getPoints(bin_image):
    image, contours, hierarchy = cv2.findContours(bin_image,1, 2)#find contours in the image
    if len(contours) < 6:#check if we found 6 points
        return None

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[0:6] #take the 6 biggest contours
    points = [] #list to store center points of contours
    xs = [] #list to store x-values from contours
    ys = [] #list to store y-values from contours
    for contour in contours:
        #http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
        M = cv2.moments(contour)
        if M['m00'] != 0: #calculate center of contour with image moments
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            xs.append(cx)
            ys.append(cy)
            points.append((cx,cy))

    #now locate the 6 points on the image:
    (u1,u2) = get2HighestIndices(ys)
    if xs[u1] < xs[u2]:
        upper_left = u1
        upper_right =  u2
    else:
        upper_right = u1
        upper_left = u2

    (m1,m2) = get2MidIndices(ys)
    if xs[m1] < xs[m2]:
        mid_left = m1
        mid_right = m2
    else:
        mid_left = m2
        mid_right = m1

    (l1,l2) = get2LowestIndices(ys)
    if xs[l1] < xs[l2]:
        lower_left = l1
        lower_right = l2
    else:
        lower_right = l1
        lower_left = l2

    #put the points in an order and return them:
    sorted_points = [points[upper_left], points[upper_right], points[mid_left], points[mid_right],
            points[lower_left], points[lower_right]]
    return sorted_points

if __name__ == '__main__':
    try:

        processImage(cv2.imread("/home/alex/repos/robotik_ws1718/ub3/tmp/cam_image_gray.png"))
        #init()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass
