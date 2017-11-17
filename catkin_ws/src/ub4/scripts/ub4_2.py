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

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def pubSpeed(i):
    print "set speed " , i
    pub = rospy.Publisher('manual_control/speed', Int16, queue_size=10)
    msg = Int16()
    msg.data = i
    pub.publish(msg)

#set steering angle
def pubSteering(a):
    pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=10)
    msg = Int16()
    msg.data = a
    pub.publish(msg)

angles = [0,30,90,120,150,179]
ANGLE = 0

print_lidar = False

def measure():
    pubSteering(ANGLE)
    rospy.sleep(5)
    print "foo"
    pubSpeed(100)
    rospy.sleep(5)
    pubSpeed(0)
    print "Abstand d2 messen"
    print_lidar = True

def scanCallback(data):
    #print data
    if print_lidar:
        print "10:350", data.ranges[10] , data.ranges[350]


def init():
    rospy.init_node('kjdsfhkdsjfh', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)
    measure()
    rospy.spin()

init()
