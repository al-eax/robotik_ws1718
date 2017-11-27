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
