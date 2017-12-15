#!/usr/bin/env python
import rospy
from math import sqrt, cos, sin,asin, acos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose

positions = []
predicted_positions = []
timestamps = []

def odomCallback(data):
    timestamps.append(rospy.Time.now())rospy.Time.now()
    positions.append(data.pose.pose)

def dist(p1,p2):
    return sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

def calcVelocity():
    #TODO make sure, veloccity becomes negative while driving backwards
    p1 = positions[-1]
    p2 = positions[-2]
    t1 = timestamps[-1]
    t2 = timestamps[-2]

    t = t2 - t1
    d = dist(p1.position,p2.position)

    return d/t

def quaternion_to_yaw(q):
    # https://answers.unity.com/questions/416169/finding-pitchrollyaw-from-quaternions.html
    yaw = math.asin(2*q[0]*q[1] + 2*q[2]*q[3]);
    #yaw = acos(q.w)
    return yaw

def predict():
    delta_t = timestamps[-2] - timestamps[-1]
    v_x = calcVelocity()
    theta = quaternion_to_yaw(positions[-1].orientation)
    last_theta = quaternion_to_yaw(positions[-2].orientation)
    v = 1 #TODO was ist v?

    delta_x = v * cos(theta) * v_x * delta_t
    delta_y = v * sin(theta) * v_x * delta_t
    delta_theta = theta - last_theta

    (x_tp1, y_tp1, theta_tp1) = predicted_positions[-1]
    x_tp = x_tp1 + delta_x
    y_tp = y_tp1 + delta_y
    theta_tp = theta_tp1 + delta_theta

    predicted_positions.append( (x_tp, y_tp, theta_tp) )


def update_position():


def main():
    global odom_pub
    rospy.Subscriber("/my_odom", Odometry, odomCallback, queue_size=1)
if __name__ == '__main__':
    main()
