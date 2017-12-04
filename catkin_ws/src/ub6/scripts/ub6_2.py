#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import cv2
from sklearn import linear_model, datasets
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#macht die oberen 20% des Bildes schwarz,
def cut_image(img):
    w = img.shape[1]
    img[0 : int(0.2 * w) , : ] = 0
    return img

def do_ransac_on_contur(contur):
    #source http://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html
    ransac = linear_model.RANSACRegressor()

    #load all x and y coordinates in seperated lists (required by ransac)
    X = []
    Y = []
    for pxl in contur:
        X.append([pxl[0][1]])
        Y.append([pxl[0][0]])

    ransac.fit(X, Y) #build ransac classifier
    b = ransac.estimator_.intercept_ #intersection
    m = ransac.estimator_.coef_ #gradient
    return (b,m)

#cluster two lines
def get_two_line_segments(img):
    im2, contours, hierarchy = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key = cv2.contourArea, reverse = True) #sort the segments by pixel count
    if len(contours) < 2:
        return None
    return (contours[0],contours[1]) #return the two biggest segments


# calculate and return the start and end points by evaluating f(x=0) and f(x=image_width)
def mb_to_tupel(b,m,width):
    def f(x,b,m): return x*m + b
    x_1 = 0
    x_2 = width
    y_1 = f(x_1,b,m)
    y_2 = f(x_2,b,m)
    return ((y_1,x_1) , (y_2,x_2))

#suche beide Linien in dem binarisierten Bild
#gibt Start- und Endpunkt beider Linien als Tupel zurueck
def detect_lines(bin_img):
    img = cut_image(bin_img) #remove noise
    line_segments = get_two_line_segments(img) #find line segments

    if line_segments is None: #less than 2 segments found?
        return None

    #get line parameters for both segments:
    b1,m1 = do_ransac_on_contur(line_segments[0])
    b2,m2 = do_ransac_on_contur(line_segments[1])

    #publish m,b for both lines:
    pub_mb(m1,b1,m2,b2)

    #get start and end points:
    L1 = mb_to_tupel(b1,m1, img.shape[1])
    L2 = mb_to_tupel(b2,m2, img.shape[1])

    return (L1,L2) #return start and end points for bost lines

#zeichnet beide Linien im Bild ein
def draw_lines(img,L1,L2):
    img = img.copy()
    cv2.line(img, L1[0] , L1[1],(0,255,0),5)# bgr
    cv2.line(img, L2[0] , L2[1],(0,255,0),5)
    return img

def pub_imgs(img_bgr,img_hsv, img_yuv):
    global img_pub_yuv
    global img_pub_hsv
    global img_pub_bgr

    bridge = CvBridge()
    ros_img_hsv = bridge.cv2_to_imgmsg(img_hsv)
    img_pub_hsv.publish(ros_img_hsv)
    ros_img_bgr = bridge.cv2_to_imgmsg(img_bgr)
    img_pub_bgr.publish(ros_img_bgr)
    ros_img_yuv = bridge.cv2_to_imgmsg(img_yuv)
    img_pub_yuv.publish(ros_img_yuv)

def handle_new_image(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    img_yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)

    lower_gray_bgr = np.array([200,200,200])
    upper_gray_bgr = np.array([255,255,255])

    mask_bgr = cv2.inRange(img_bgr, lower_gray_bgr, upper_gray_bgr)
    #res_bgr = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_bgr)

    lower_gray_hsv = np.array([0,0,200])
    upper_gray_hsv = np.array([255,30,255])

    mask_hsv = cv2.inRange(img_hsv, lower_gray_hsv, upper_gray_hsv)
    #res_hsv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_hsv)

    lower_gray_yuv = np.array([200,0,0])
    upper_gray_yuv = np.array([255,255,255])
    mask_yuv = cv2.inRange(img_yuv, lower_gray_yuv, upper_gray_yuv)
    #res_yuv = cv2.bitwise_and(img_bgr, img_bgr, mask = mask_yuv)

    #lets use the mask as binady image:
    bgr_1_channel = mask_bgr
    hsv_1_channel = mask_hsv
    yuv_1_channel = mask_yuv

    pub_imgs(bgr_1_channel,hsv_1_channel,yuv_1_channel)

    L1, L2 = detect_lines(yuv_bin) #get start and end points for lines in yuv img
    img = draw_lines(img_bgr,L1,L2) #draw lines on original image
    pub_img_lines(img)

#publish line parameters:
def pub_mb(m1,b1,m2,b2):
    global mb_pub
    #source: https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].size = 4
    mat.layout.dim[0].stride = 4
    mat.layout.data_offset = 0
    mat.data = [m1,b1,m2,b2]

    mb_pub.publish(mat)

#publish the bgr image the drawn lines
def pub_img_lines(img_lines):
    global img_pub_lines
    bridge = CvBridge()
    ros_img = bridge.cv2_to_imgmsg(img_lines, "bgr8")
    cv2.imwrite("foo.png",img_lines)
    img_pub_lines.publish(ros_img)

#callback function
def camCallback(data):
    bridge = CvBridge()
    print "new image"
    cv_img = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.imwrite("bar.png",cv_img)
    handle_new_image(cv_img)

def init():
    global img_pub_hsv
    global img_pub_bgr
    global img_pub_yuv
    global mb_pub
    global img_pub_lines

    rospy.init_node('foobar', anonymous=True)
    rospy.Subscriber("/app/camera/rgb/image_color", Image, camCallback, queue_size=1)
    img_pub_hsv = rospy.Publisher("/line_img/hsv",Image,queue_size=1)
    img_pub_bgr = rospy.Publisher("/line_img/bgr",Image,queue_size=1)
    img_pub_yuv = rospy.Publisher("/line_img/yuv",Image,queue_size=1)

    img_pub_lines = rospy.Publisher("/line_img/lines",Image,queue_size=1)
    mb_pub = rospy.Publisher("/mb",Float32MultiArray,queue_size=1)


if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
