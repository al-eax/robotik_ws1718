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

CALIBRATED_ZERO_ANGLE = 90
alpha = 20
leftAngle = 280
rightAngle = 260
KP = 15
KD = 8
t = []
heading_arr = []

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# initialize ros and subscribers and publishers
def init():
    global steering_pub
    global speed_pub

    rospy.init_node('im_a_funny_node_in_a_funny_ros_package', anonymous=True)
    speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    steering_pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=1)
    rospy.sleep(1)

# method to get the correct scan/ranges array mapping for a given angle
def mapAngleToArray(angle,stepping):
    stepping = stepping * 180 / math.pi
    return int(round(angle / stepping))

# method to get the precise angle for a given scan/ranges array mapping
def mappingToAngle(mapping,stepping):
    stepping = stepping * 180 / math.pi
    return mapping * stepping
    
    
# a few calculations to get theta* and theta
def getTheta(dl2, dr2, alpha):
    p = 0.4
    l = 0.5
    s = 0.2
    
    t = math.sqrt(dr2**2 + dl2**2 - 2 * dl2 * dr2 * math.cos(alpha))
    phi2 = math.asin(dr2 * math.sin(alpha) / t)
    do2 = math.sin(phi2) * dl2
    thetal2 = math.asin(do2 / dl2)
    theta = thetal2 - alpha
    cy = do2 + math.sin(theta) * s
    thetaStar = math.atan2(p - cy, l) * 180 / math.pi
    theta = theta * 180 / math.pi - 90
    return thetaStar, theta

# callback of the /scan topic we subscribed to
def scanCallback(data):
    
    stepping = data.angle_increment
    
    # get dl2, dr2 for the given angles and get the precise angles they are at
    dl2 = data.ranges[mapAngleToArray(leftAngle, stepping)]
    leftAngle = mappingToAngle(mapAngleToArray(leftAngle, stepping), stepping)
    dr2 = data.ranges[mapAngleToArray(rightAngle, stepping)]
    rightAngle = mappingToAngle(mapAngleToArray(rightAngle, stepping), stepping)
    # get the angle between dl2 and dr2
    alpha = leftAngle - rightAngle
    
    thetaStar, theta = getTheta(dl2, dr2, alpha)
    deltaHeading = thetaStar - theta
    # array to calculate derivative and plot heading later
    heading_arr.append(deltaHeading)
    # array to calculate derivative and plot time later
    t.append(time.time())
    
    # PD controller calculation from the assignment
    derivative = 0
    if len(heading_arr > 1):
        derivative = (heading_arr[-1] - heading_arr[-2]) / (t[-1] - t[-2])
    u = KP * deltaHeading + KD * derivative + CALIBRATED_ZERO_ANGLE
    
    # logging
    print "Steering to: ", thetaStar, " from: ", theta
    print "u = ", u
    
    #pubSteering(u)
    
    
    
    
               
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
        #pubSpeed(-300)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
