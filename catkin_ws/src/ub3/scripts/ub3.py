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
    #/app/camera/color/image_raw
    rospy.Subscriber("/app/camera/color/image_raw",Image,cameraRawCallback)#subscribe to cars camera
    bridge = CvBridge() #create a opencv <->ros bridge to convert images

#callback function to catch images from cars camera
def cameraRawCallback(data):
    print "got a new image"
    global cv_input_image
    global bridge
    cv_input_image = bridge.imgmsg_to_cv2(data, "bgr8") #convert ros image to opencv image
    cv_gray_image = cv2.cvtColor(cv_input_image, cv2.COLOR_BGR2GRAY) #convert to gray scale
    _,cv_binary_image = cv2.threshold(cv_gray_image,220,255,cv2.THRESH_BINARY)#+cv2.THRESH_OTSU) #binarize image into black and white

    image_2d_points = getPoints(cv_binary_image)
    print image_2d_points

    realworld_3d_points = [()]

    fx = 614.1699
    fy = fx
    cx = 329.9491
    cy = 237.2788

    intrinsic_params = np.asarray([[fx,0,cx],[0,fy,cy],[0,0,1]])

    k1 = 0.1115
    k2 = -0.1089
    t1 = 0
    t2 = 0

    #distortion_params = cv2.CreateMat(3, 3, cv2.CV_32FC3)
    distortion_params = np.asarray([k1,k2,t1,t2])

    (retval, rvec, tvec) = cv2.solvePnP(realworld_3d_points,image_2d_points,intrinsic_params,distortion_params)

    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_gray.png",cv_gray_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_rgb.png",cv_input_image)
    cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_binary.png",cv_binary_image)
    cv_detected_img = cv_input_image.copy()
    i = 1
    for center in image_2d_points:
        cv2.circle(cv_detected_img,center, 6,(0,0,255),3) #draw a red circle on the detected white square
        cv2.imwrite("/home/alex/repos/robotik_ws1718/ub3/cam_image_detected" + str(i) ".png",cv_detected_img)


def pubImages(img_gray, img_bin):
    global bridge
    gray_scale_pub = rospy.Publisher("/gray_scale_img",Image,queue_size = 10)
    bin_pub = rospy.Publisher("/binary_img",Image,queue_size = 10)

    ros_image_gray = bridge.cv2_to_imgmsg(img_gray, "mono8") #mono8 = 8bit,1 channel = grayscale
    ros_image_bin = bridge.cv2_to_imgmsg(img_bin, "mono8")

    gray_scale_pub.publish(ros_image_gray)
    bin_pub.publish(ros_image_bin)



def getPoints(bin_image):
    image, contours, hierarchy = cv2.findContours(bin_image,1, 2)#find contours in image
    #contours = sorted(contours, key = cv2.contourArea, reverse = True)[0:6] #take the 6 biggest contours
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
            points.append((cx,cy))
            xs.append(cx)
            ys.append(cy)
    #[(473, 335), (283, 235), (433, 222), (636, 292), (638, 63), (316, 185)]
    #get first two points/objects, smallest Y-Koordinate

    upper_left = []
    upper_right = []
    middl_left = []
    middl_right = []
    lower_left = []
    lower_right = []

    return points

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
