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

#set steering angle
def pubSteering(a):
    pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=10)
    rospy.sleep(2)
    msg = Int16()
    msg.data = a
    pub.publish(msg)

angles = [0,30,90,120,150,179]
current_angle = 3

print_lidar = False

# 0deg :  1.18624997139 1.40125000477
# 30deg : 1.14999997616 1.30375003815
# 90deg : 1.14600002766 1.16425001621
# 120deg :

def measure():
    #d1 = 1m bis zur hinderen achse
    global print_lidar
    pubSteering(angles[current_angle])
    rospy.sleep(5)
    pubSpeed(100)
    rospy.sleep(3)
    pubSpeed(0)
    print "Abstand d2 messen"
    print_lidar = True

def scanCallback(data):
    global print_lidar
    #print data
    if print_lidar:
        print "a 10:350", data.ranges[10] , data.ranges[350]
        print_lidar = False

def pubSpeed(i):
    pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    rospy.sleep(1)
    msg = Int16()
    msg.data = i
    pub.publish(i)

rospy.init_node('kjdsfhkdsjfh', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)
measure()
rospy.spin()
