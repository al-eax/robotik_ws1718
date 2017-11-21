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

KP = 0.6
CALIBRATED_ZERO_ANGLE = 90
errors = []

def init():
    global steering_pub
    global speed_pub
    global yaw_sub
    rospy.init_node('im_a_funny_node_in_a_funny_ros_package', anonymous=True)
    yaw_sub = rospy.Subscriber("/model_car/yaw", Float32 , yawCallback, queue_size=10)
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

def yawCallback(data):
    global ITERATIONS
    global yaw_sub

    current_yaw = data.data

    do_PDC(current_yaw, 90)

    if len(errors) == 100:
        yaw_sub.unregister()
        pubSpeed(0)
        sme = np.sum(errors) / len(errors)
        print "DONE"
        print "mean squared error", sme
        

def do_PDC(current_yaw, desired_yaw):
    global errors
    u = KP *(desired_yaw - current_yaw) + CALIBRATED_ZERO_ANGLE
    errors.append( (desired_yaw - current_yaw)**2)
    print "(" ,len(errors) , ")" "u = " , KP , " *  (", desired_yaw , " - " , current_yaw , ") + " , CALIBRATED_ZERO_ANGLE , " = " , u
    pubSteering(u)

if __name__ == '__main__':
    try:
        init()
        pubSpeed(-500)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
