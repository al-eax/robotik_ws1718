#!/usr/bin/env python
import rospy
import numpy as np
from math import sqrt, cos, sin,asin, acos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose

import matplotlib
import matplotlib.pyplot as plt

import time

gps = []
odoms = []
predicts = []
updates = []

#def quaternion_to_yaw2(q):
    #siny = +2.0 * (q.w * q.z + q.x * q.y)
	#cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
	#yaw = atan2(siny, cosy);

def quaternion_to_yaw(q):
    return acos(q.w) * 2 * np.sign(q.z)

def yaw_to_quaternion(yaw):
    return Quaternion(0,0,sin(yaw / 2) , cos(yaw / 2))

def calc_velocity():
    #TODO Beim rueckwerts fahren, neg geschwindigkeit
    (x1, y1, _, t1) = odoms[-1] # aktuelles
    (x2, y2, _, t2) = odoms[-2] # letztes


	A = np.array([x1,y1]) #aktuelle
	B = np.array([x2,y2]) #letzte
	
	#relative:
	A = A - A
	B = B - A
	
	E = np.array([1,0])#einheitsvektor
	yaw = angle(B,E)
	
	real_yaw = 0#
	if yaw <= real_yaw - 10 or yaw >= real_yaw + 10:
		pass
		#fahren vorsaerts
	else:
		#fahren rueckwerts

    d = dist(x1,y1,x2,y2)
    return d / float(t1-t2)


def angle(A,B):
	return acos( (A[0] * B[0] + A[1] * B[1])  /  np.linalg.norm(A,2) * np.linalg.norm(B,2) )



def predict():
    if len(gps) < 2:
        return
    if len(predicts) < 1:
        predicts.append(gps[-1][0:3])
    (x_tp1, y_tp1, theta_tp1) = predicts[-1]
    v = calc_velocity()

    (_,_,theta_1,t_1) = gps[-1]
    (_,_,theta_2,t_2) = gps[-2]
    delta_t = t_1 - t_2

    delta_x = v * cos(theta_1) * delta_t
    delta_y = v * sin(theta_1) * delta_t
    delta_theta = theta_1 - theta_2

    print v, delta_x , delta_y

    x_tp = x_tp1 + delta_x
    y_tp = y_tp1 + delta_y
    theta_tp = theta_tp1 + delta_theta

    predicts.append( (x_tp, y_tp, theta_tp) )


def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def gps_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)
    t = time.time()
    gps.append((x,y,yaw ,t))

    predict()
    _filter()

def odom_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)
    t = time.time()
    odoms.append((x,y,yaw,t))


def create_odom_obj(x,y,yaw):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.pose.pose = Pose(Point(x, y, 0) , yaw_to_quaternion(yaw))
    return msg

def _filter():
    global pub_update
    global pub_odom
    global pub_gps

    if len(gps) < 1 or len(predicts) < 1:
        return
    k = 0
    (gps_x, gps_y, gps_theta,_) = gps[-1]
    (pred_x, pred_y, pred_theta) = predicts[-1]

    updated_position_x = k * gps_x + (1-k) * pred_x
    updated_position_y = k * gps_y + (1-k) * pred_y
    updated_position_theta = k * gps_theta + (1-k) * pred_theta

    updates.append( (updated_position_x,updated_position_y,updated_position_theta) )

    print "odom", odoms[-1]
    print "gps ", gps[-1]
    print "upd",updates[-1]
    print "pred" , predicts[-1]
    print "==================================="

    #republish all odoms to have the same time space:
    pub_update.publish(create_odom_obj(updated_position_x,updated_position_y,updated_position_theta))
    pub_odom.publish(create_odom_obj(odoms[-1][0],odoms[-1][1],odoms[-1][2]))
    pub_gps.publish(create_odom_obj(gps[-1][0],gps[-1][1],gps[-1][2]))

def main():
    global pub_update
    global pub_odom
    global pub_gps
    rospy.init_node('my_little_nody', anonymous=True)
    rospy.Subscriber("/my_odom", Odometry, gps_callback, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    pub_update = rospy.Publisher('/update_now', Odometry, queue_size=1)
    pub_odom = rospy.Publisher('/odom_now', Odometry, queue_size=1)
    pub_gps = rospy.Publisher('/gps_now', Odometry, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
