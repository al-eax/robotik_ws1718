#!/usr/bin/env python
import rospy
import cv_bridge
import signal
import sys
import cv2

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
    rospy.init_node('camera', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw",Image,cameraRawCallback)#subscribe to cars camera
    bridge = CvBridge() #create a opencv <->ros bridge to convert images

#callback function to catch images from cars camera
def cameraRawCallback(data):
    global cv_input_image
    global bridge
    cv_input_image = bridge.imgmsg_to_cv2(data, "bgr8") #convert ros image to opencv image
    #cv2.imwrite("/home/alex/foo.png",cv_input_image)
    pubGrayScaleImage(cv_input_image)

#convert an image to grayscale and publish it
def pubGrayScaleImage(cv_img):
    global bridge
    gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)#convert to gray scale
    gray_scale_pub = rospy.Publisher("/gray_scale_img",Image,queue_size = 10)
    ros_image = bridge.cv2_to_imgmsg(gray_image, "mono8") # mono8 = 8bit,1 channel = grayscale
    gray_scale_pub.publish(ros_image)
    #cv2.imwrite("/home/alex/foo_gray.png",gray_image)
    pubBinImage(gray_image)


def pubBinImage(cv_image):
    global bridge
    _,binary_image = cv2.threshold(cv_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #binarize image into black and white
    bin_pub = rospy.Publisher("/binary_img",Image,queue_size = 10)
    ros_image = bridge.cv2_to_imgmsg(binary_image, "mono8")
    bin_pub.publish(ros_image)
    #cv2.imwrite("/home/alex/foo_bin.png",binary_image)
    points = getPoints(binary_image)

def getPoints(bin_image):
    image, contours, hierarchy = cv2.findContours(bin_image,1, 2)#find contours in image
    points = []
    print len(contours)
    for contour in contours:
        #http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
        M = cv2.moments(contour)
        if M['m00'] != 0:#calculate center of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            points.append((cx,cy))
    return points

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
