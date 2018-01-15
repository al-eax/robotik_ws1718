#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from balloon_detector import BalloonDetector
import numpy as np
import numpy.linalg as la
import signal
import math
import time
from math import sqrt, cos, sin,asin, acos, atan2
import random
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
from std_msgs.msg import Int16
import os
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance

resolution = 10
map_size_x=600 #cm
map_size_y=400 #cm

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        pub_speed.publish(0)
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def quaternion_to_yaw(q):
    return acos(q.w) * 2 * np.sign(q.z)

def yaw_to_quaternion(yaw):
    return Quaternion(0,0,sin(yaw / 2) , cos(yaw / 2))

def angle(x,y):
    return atan2(y,x)

def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def norm_rad(yaw):
    if yaw > math.pi:
        return yaw - 2 * math.pi
    elif yaw < -math.pi:
        return yaw + 2 * math.pi
    else:
        return yaw

# https://newtonexcelbach.com/2014/03/01/the-angle-between-two-vectors-python-version/
def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)

# usb cam callback
first_location = False

speed = 0
def callback(data):
    global first_location
    global speed
    img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    pub_odom(img)
    if first_location == False:
    #detect balloons
        first_location = True


        x, y = detector.calculate_best_position(img)
        yaw = detector.calculate_angle()

        print "x,y,yaw" , (x , y, yaw)

        x_index=np.int(x*resolution)
        y_index=np.int(y*resolution)

        if (x_index < 0):
            x_index = 0
        if (x_index > ((map_size_x/resolution)-1)):
            x_index = (map_size_x/resolution)-1

        if (y_index < 0):
            y_index = 0
        if (y_index > ((map_size_y/resolution)-1)):
            y_index = (map_size_y/resolution)-1

        x3, y3 = force_map[x_index,y_index,:]
        f_x = np.cos(yaw)*x3 + np.sin(yaw)*y3
        print(f_x)

        f_y = -np.sin(yaw)*x3 + np.cos(yaw)*y3
        Kp = 4.0
        steering=Kp*np.arctan(f_y/(2.5*f_x))

        if (f_x > 0):
            speed = -150
        else:
            speed = 150
            if (f_y > 0):
                steering = -np.pi/2
            if (f_y < 0):
                steering = np.pi/2

        if (steering > (np.pi)/2):
            steering = (np.pi)/2

        if (steering < -(np.pi)/2):
            steering = -(np.pi)/2

        steering = 90 + steering * (180/np.pi)

        print "steering: ", steering

        #print "speed: ", speed
        pub_steering.publish(Int16(steering))
    else:
        #print speed
        pub_speed.publish(Int16(speed))


def pub_odom(img):

    pose_covar = PoseWithCovariance(Pose(Point(0, 0, 0), Quaternion()), None)
    odom = Odometry(Header(frame_id='odom'), 'base_link', pose_covar, None)
    xy = detector.calculate_best_position(img)
    #for bln in detector.balloon_positions:
        #print bln

    # Don't publish a pose if location can't be reliably determined
    if xy is None:
        print("No location found")
        return

    yaw_angle = detector.calculate_angle()

    # publish odometry message
    header = odom.header
    #header.seq = data.header.seq
    #header.stamp = data.header.stamp

    pose = odom.pose.pose
    pos = pose.position
    pos.x, pos.y = xy

    quaternion = pose.orientation
    quaternion.z, quaternion.w = math.sin(yaw_angle / 2), math.cos(yaw_angle / 2)
    odom_pub.publish(odom)

def main():
    global bridge
    global detector
    global pub_steering
    global pub_speed
    global force_map
    global odom_pub

    shell_script = "sshpass -p 'elfmeter' ssh root@192.168.43.102 'v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_auto=1; v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_absolute=5'"
    os.system(shell_script)

    force_map = np.load("matrixDynamic_lane2.npy")

    rospy.init_node('my_tenth_node', anonymous=True)
    bridge = CvBridge()

    detector = BalloonDetector()

    pub_steering = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)
    pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
    odom_pub = rospy.Publisher("/assignment6/odom", Odometry, queue_size=200)
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
