#!/usr/bin/env python
import rospy
from math import *
import signal
import sys

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry




# Robotik ws17/18
# Assignment 2
# Alexander Hinze-Huettl - 4578322
# hinze.alex[at]gmail.com
#
# this script uses odometry and yaw to calculate the side length and the curve-angle.
#


current_yaw = 0 #holds the current yaw value updated by yawCallback
current_position = (0,0,0) #holds the current position updated by odomCallback

STEERING_STRAIGHT = 80 #seems to be the angle to drive a straight line
STEERING_LEFT = 40
STEERING_RIGHT = 110

MOTOR_SPEED = -900
MOROT_SPEED_SLOW = -150
DIST_TO_DRIVE = 4
CURVE_ANGLE = 65 #90


# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

#calculate euclidian dist from two otometry values
def dist(A,B):
    (x1,y1,z1) = A
    (x2,y2,z2) = B
    return sqrt( (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

def init():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odomCallback) #subscribe to odometry for position information
    rospy.Subscriber("/model_car/yaw", Float32, yawCallback) #subscribe to yaw view angle to calculate curve angle

#set car speed
def pubSpeed(i):
    pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
    msg = Int16()
    msg.data = i
    pub.publish(msg)

#set steering angle
def pubSteering(a):
    pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=10)
    msg = Int16()
    msg.data = a
    pub.publish(msg)

#update current position by subscribed odometry
def odomCallback(data):
    global current_position
    current_position = (data.pose.pose.position.x, #odom to tripel
                        data.pose.pose.position.y,
                        data.pose.pose.position.z)

#update current yaw angle and make it from {-180deg, 180deg} to {0,360deg}
def yawCallback(data):
    global current_yaw
    if data.data < 0:
        current_yaw = data.data + 360
    else:
        current_yaw = data.data

#calculate the min diff between two angles:
# angleDiff(30 , 80) -> 50
# angleDiff(350 , 10) -> 20
def angleDiff(a1,a2):
    abs_angle =  abs(a1 - a2)
    if abs_angle > 180:
        return abs_angle - 180
    else:
        return abs_angle


def driveDist():
    print "go straight"
    first_position = current_position
    while dist(first_position, current_position) < DIST_TO_DRIVE:
        pubSpeed(MOTOR_SPEED)
        pubSteering(STEERING_STRAIGHT)
    print "dist ", dist(first_position, current_position)
    pubSpeed(0)

def driveCurve():
    print "turn left"
    first_yaw = current_yaw
    while angleDiff(first_yaw,current_yaw)  < CURVE_ANGLE: #drive a curve until yaw angle has changed enought
        pubSpeed(MOROT_SPEED_SLOW)
        pubSteering(STEERING_RIGHT)
    print "angle ", angleDiff(first_yaw,current_yaw)
    pubSpeed(0)
    pubSteering(STEERING_STRAIGHT)

def drive():
    for i in range(4): #do 4 iterations
        driveDist()
        driveCurve()
    print "DONE!"

if __name__ == '__main__':
    try:
        init()
        print "juhuuu"
        print "sleep 1 sec..."
        rospy.sleep(1) #sleep 1 sec to get valid odometry and yaw values from callbacks
        print "go!"
        drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
