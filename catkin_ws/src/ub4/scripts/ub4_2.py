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

current_angle = 5

print_lidar = False

#angle: 0
# dl1,dr1:
# 10:350 0.910000026226 0.902000010014
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.37399995327 1.64900004864

#angle: 30
# dl1,dr1:
# 10:350 0.910000026226 0.907999992371
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.38499999046 1.59399998188

#angle: 90
# dl1,dr1:
# 10:350 0.916999995708 0.916000008583
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.38100004196 1.40299999714

#angle: 120
# dl1,dr1:
# 10:350 0.907000005245 0.916000008583
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.41100001335 1.36600005627

#angle: 150
# dl1,dr1:
# 10:350 0.921000003815 0.919000029564
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.47800004482 1.33299994469

#angle: 179
# dl1,dr1:
# 10:350 0.916000008583 0.907000005245
#Abstand d2 messen
# dl2 , dr2:
# 10:350 1.65499997139 1.38699996471


def t(dl2,dr2):
    t = dl2 +dr2+ (2*dl2+dr2* np.cos(20))
    return( t)

def phi2(dr2,T):
    phi2= T/np.sin(20) * (1/dr2)
    return phi2

def theta_o2(dl2, dr2):
    T = t(dl2,dr2)
    phi = phi2(dr2,T)
    num = np.sin((phi/dl2))
    theta = np.arccos(num/dl2)
    return theta - 10


def create_mapping():
    d2 = [(1.37399995327, 1.64900004864),
        (1.38499999046, 1.59399998188),
        (1.38100004196, 1.40299999714),
        (1.41100001335, 1.36600005627),
        (1.47800004482, 1.33299994469),
        (1.65499997139, 1.38699996471)
        ]
    o2_list = []
    for (dl2, dr2) in d2:
        o2 = theta_o2(dl2,dr2)
        o2_list.append(o2)
    return o2_list



def do_map(angle):
    o2 = create_mapping()



def measure():
    #d1 = 1m bis zur hinderen achse
    global print_lidar
    print "angle:" ,angles[current_angle]
    print_lidar = True
    pubSteering(angles[current_angle])
    rospy.sleep(5)
    pubSpeed(100)
    rospy.sleep(3)
    pubSpeed(0)
    print_lidar = True
    pubSteering(100)

def scanCallback(data):
    global print_lidar
    #print data
    if print_lidar:
        print " 10:350", data.ranges[10] , data.ranges[350]
        print_lidar = False

def pubSpeed(i):
    pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    rospy.sleep(1)
    msg = Int16()
    msg.data = i
    pub.publish(i)


print create_mapping()
rospy.init_node('kjdsfhkdsjfh', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)
rospy.spin()
