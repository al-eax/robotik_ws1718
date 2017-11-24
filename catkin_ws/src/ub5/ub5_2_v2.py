#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
    odom_sub.unregister()
    plt.plot(t, odomy_arr)
    plt.xlabel('callback #')
    plt.ylabel('y')
    plt.grid(True)
    plt.show()
    
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

odomy_arr = []
t = []

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

KP = 0.6
KD = 0.4
CALIBRATED_ZERO_ANGLE = 80


def odomCallback(data):
    current_y = data.pose.pose.position.y
    odomy_arr.append(data.pose.pose.position.y)
    t.append(time.time())
    do_PDC(current_y, 0.2)
    

def do_PDC(current_y, desired_y):
    derivative = 0
    if len(odomy_arr) > 2:
        derivative = (odomy_arr[-1]-odomy_arr[-3]) / (t[-1]-t[-3]) #sub last two elements

    u = KP *(desired_y - current_y) + KD * (0 - derivative) + CALIBRATED_ZERO_ANGLE
    print "u=" , KP , "*(", desired_y - current_y , ") ", "+" , KD , "*" , -derivative, "=",u

    pubSteering(u)


if __name__ == '__main__':
    try:
        init()
        pubSpeed(-500)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
