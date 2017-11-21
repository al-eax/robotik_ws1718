#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import math

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def init():
    global steering_pub
    global speed_pub
    global odom_sub

    rospy.init_node('im_a_funny_node_in_a_funny_ros_package', anonymous=True)
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback) #subscribe to odometry for position information
    speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
    steering_pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=10)
    rospy.sleep(1)

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

errors = []
KP = 0.2
KD = 0.8
CALIBRATED_ZERO_ANGLE = 90


def odomCallback(data):
    current_y = data.pose.pose.position.y
    do_PDC(current_y, 0.2)

def do_PDC(current_y, desired_y):
    global errors
    error_fac  = 0
    if len(errors) > 1:
        error_fac = (errors[-1] - errors[-2]) #sub last two elements

    u = KP *(desired_y - current_y) + KD * error_fac
    print "u=" , KP , "*(", desired_y , "-", current_y , ") ", "+" , KD , "*" , error_fac, "=",u
    errors.append((desired_y - current_y)**2)

    #print "u =", u
    #print "current_y =", current_y
    #print "desired_y =", desired_y
    sme = np.sum(errors) / len(errors)
    print "sme",sme

    pubSteering(u)


if __name__ == '__main__':
    try:
        init()
        pubSpeed(-1000)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
