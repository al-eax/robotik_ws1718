#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import math
import matplotlib.pyplot as plt

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):

    # when the script is terminated manually do the following
    # print the mean squared error to the terminal
    sme = np.sum(errors) / len(errors)
    print "DONE"
    print "mean squared error", sme
    
    # plot the yaw over time using matplotlib
    t = range(len(errors))
    plt.plot(t, heading_array)
    plt.xlabel('callback #')
    plt.ylabel('yaw')
    plt.grid(True)
    plt.show()
    
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

KP = 15
CALIBRATED_ZERO_ANGLE = 81
errors = []
heading_array = []

# initialize ros and subscribers and publishers
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

# callback for the /model_car/yaw topic we subscribed to
def yawCallback(data):
    global yaw_sub
    global heading_array

    current_yaw = data.data
    # append current yaw to an array to plot it later
    heading_array.append(data.data)

    pubSpeed(-200)

    do_PDC(current_yaw, 79)

# here the steering u is calculated using the formula of the lecture
def do_PDC(current_yaw, desired_yaw):
    global errors
    u = KP *(desired_yaw - current_yaw) + CALIBRATED_ZERO_ANGLE
    # squared error is saved in an array
    errors.append( (desired_yaw - current_yaw)**2)
    print "(" ,len(errors) , ")" "u = " , KP , " *  (", desired_yaw , " - " , current_yaw , ") + " , CALIBRATED_ZERO_ANGLE , " = " , u

    # set some boundaries
    if u < 0:
        u = 0
    if u > 179:
        u = 179

    pubSteering(u)

if __name__ == '__main__':
    try:
        init()
        pubSpeed(-200)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
