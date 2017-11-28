#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import math
import time

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from sensor_msgs.msg import LaserScan

CALIBRATED_ZERO_ANGLE = 99

KP = 0.1
KD = 1
t = []
heading_arr = []

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        pubSpeed(0)
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def init():
    global steering_pub
    global speed_pub
    global s_pub
    global theta_pub
    rospy.init_node('im_a_funny_node_in_a_funny_ros_package', anonymous=True)
    speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    steering_pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=1)
    s_pub = rospy.Publisher("/steering",Int16,queue_size=1)
    theta_pub = rospy.Publisher("/theta",Int16,queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=1)
    rospy.sleep(1)


def remap(angle,stepping):
    stepping = stepping * 180.0 / math.pi
    return angle * stepping

#calculate the index in scan data.ranges for the given angles
def mapAngleToArray(angle,stepping):
    stepping = stepping * 180.0 / math.pi
    return int(round(angle / stepping))

#calculate theta by formula from slides
def getTheta(dl2, dr2, alpha,do2):
    if math.isinf(dl2) or math.isinf(dr2):
        return 0,0 ,0
    p = 0.4
    l = 0.5
    s = 0.2
    alpha = alpha * math.pi / 180.0 #deg to rad
    t = math.sqrt(dr2**2 + dl2**2 - 2 * dl2 * dr2 * math.cos(alpha))
    phi2 = math.asin(dr2* math.sin(alpha) / t)
    do2 =  math.sin(phi2) * dl2 #dist to wall
    thetal2 = math.asin(do2 / dl2)
    theta = thetal2 - alpha
    cy = do2 + math.sin(theta) * s
    thetaStar = math.atan2(p - cy, l) * 180 / math.pi
    theta = theta * 180 / math.pi - 90

    return thetaStar, theta, do2

def scanCallback(data):
    global leftAngle
    global rightAngle
    stepping = data.angle_increment

    leftAngle = 100
    rightAngle = 80

     # get dl2, dr2 for the given angles and get the precise angles they are at
    dl2 = data.ranges[mapAngleToArray(leftAngle, stepping)]
    leftAngle = remap(leftAngle, stepping)
    dr2 = data.ranges[mapAngleToArray(rightAngle, stepping)]
    rightAngle = remap(rightAngle, stepping)
    # get the angle between dl2 and dr2
    alpha = leftAngle - rightAngle
    thetaStar, theta ,do2 = getTheta(dl2, dr2, alpha,0)
    if do2 == 0:
        return

    deltaHeading = thetaStar - theta
    heading_arr.append(deltaHeading) # array to calculate derivative and plot heading later
     # array to calculate derivative and plot time later
    t.append(time.time())

    # PD controller calculation from the assignment
    derivative = 0
    if len(heading_arr) > 1:
        derivative = (heading_arr[-1] - heading_arr[-2]) / (t[-1] - t[-2])
    u = - KP * deltaHeading - KD * derivative + CALIBRATED_ZERO_ANGLE

    print "dl2, do2, dr2 =", dl2 , do2, dr2
    print "Steering to: ", thetaStar, " from: ", theta
    print "u = ", u
    print ""

    pubSteering(u)
    s_pub.publish(u) #publish for qt_plot
    theta_pub.publish(theta) #publish for qt_plot

#set car speed
def pubSpeed(i):
    global speed_pub
    msg = Int16()
    msg.data = i
    speed_pub.publish(msg)

#set steering angle
def pubSteering(a):
    global steering_pub
    msg = Int16()
    msg.data = a
    steering_pub.publish(msg)

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
