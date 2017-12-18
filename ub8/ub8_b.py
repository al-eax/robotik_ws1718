import rospy
import numpy as np
from math import sqrt, cos, sin,asin, acos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose

import matplotlib
import matplotlib.pyplot as plt

gps = []
odoms = []
predicts = []
updates = []

def gps_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)
    t = rospy.Time.now().nsecs
    gps.append((x,y,yaw ,t))
    if len(gps) > 2 and len(odom) > 2:
        filter_loop()

def odom_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)
    t = rospy.Time.now().nsecs
    odoms.append((x,y,yaw,t))


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


def calc_velocity():
    #TODO Beim rueckwerts fahren, neg geschwindigkeit
    if len(odoms) < 2:
        return
    (x1, y1, _, t1) = odoms[-1]
    (x2, y2, _, t2) = odoms[-2]
    return ((x1-x2) / float(t2-t1),(y1-y2) / float(t2-t1) )

init = False

#https://vimeo.com/87854542
#bzw https://vimeo.com/87854540
#bzw https://github.com/balzer82/Kalman/blob/master/Kalman-Filter-CV.ipynb?create=1

def filter_loop():
    global init

    (vx,vy) = calc_velocity()
    X = np.array([0,0,vx,vy,theta]) #vx,vy = geschwindigkeit

    if not init:
        init = True

        P = np.diag([1000.0, 1000.0, 1000.0, 1000.0]) #

        dt = 0.1 # Time Step between Filter Steps muss angepasst werden
        #delta_x = v * cos(theta_1) * delta_t
        #x = x + t * v
        A = np.matrix([[1.0, 0.0, dt * cos(t), 0.0],
                      [0.0, 1.0, 0.0, dt * sin(t)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])

        ra = 10.0**2
        R = np.matrix([[ra, 0.0],
                      [0.0, ra]])

        I = np.eye(4)
        #H ist die Messmatrix, x ist der State. Alle werte in x, die gemessen werden (z.B. theta, geschwindigkeit), werden in H mit einer 1 versehen.


        pass


if __name__ == '__main__':
    main()
