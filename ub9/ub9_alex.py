#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from math import sqrt, cos, sin,asin, acos, atan2
import random
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
import tf2_ros


# size of arena: 10x10
# divide by 2 to get a number between -5 and 5
# so it is centered on (0,0)
arena_x = 10.0/2.0
arena_y = 10.0/2.0
PARTICLE_NUM = 3


balloons = [('purple',2.255, 2.425),
            ('red',3.515, 3.04),
            ('blue',4.14, 1.79),
            ('green',2.265, 1.165)]

def quaternion_to_yaw(q):
    return acos(q.w) * 2 * np.sign(q.z)

def yaw_to_quaternion(yaw):
    return Quaternion(0,0,sin(yaw / 2) , cos(yaw / 2))

def angle(x,y):
    return atan2(y,x)

def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle_between_vecs(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def create_ros_pose_array_object():
    global pose_array
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.poses = pose_array
    return msg

def get_pose(x,y,yaw):
    return Pose(Point(x, y, 0), yaw_to_quaternion(yaw))

def initialize_pose_array():
    global pose_array
    pose_array = []
    for i in range(PARTICLE_NUM):
        x = random.uniform(-arena_x, arena_x)
        y = random.uniform(-arena_y, arena_y)
        yaw = random.uniform(-math.pi, math.pi)
        pose_array.append(get_pose(x,y,yaw))

def rad_to_deg(a):
    return a * 180.0/math.pi

def deg_to_rad(a):
    return a * math.pi/180.0

def norm_deg(a):
    if a < 0:
        a += 360
    return a % 360

def norm_weights(weights):
    s = 0
    for w in weights:
        s += w
    for i in range(len(weights)):
        weights[i] = weights[i] / s
    return weights

def calc_weight(ptcl, car_pose):
    #TODO yaw mit einberechnen.
    # nicht den winkel zwischen ballon und car berechnen, sondern den winkel zwischen ballon und yaw
    # ziehe Auto in mitelpunkt, berechne Winkel zu allen baloons,
    (p_x,p_y,p_yaw_deg) = ptcl
    (c_x,c_y,c_yaw_deg) = car_pose

    final_weight = 1
    std = 50

    #mit yaw
    for (bln_color, bln_x, bln_y) in [ balloons[0] ]:
        rel_car_x = bln_x - c_y
        rel_car_y = bln_y - c_y
        car_angle = angle(rel_car_x,rel_car_y)

        rel_ptcl_x = bln_x - p_x
        rel_ptcl_y = bln_y - p_y
        ptcl_angle = angle(rel_ptcl_x,rel_ptcl_y)

        car_angle_deg = norm_deg(rad_to_deg(car_angle) - c_yaw_deg)
        ptcl_angle_deg = norm_deg(rad_to_deg(ptcl_angle) - p_yaw_deg)

        #print "angles " , norm_deg(rad_to_deg(car_angle)) , "   " , norm_deg(rad_to_deg(ptcl_angle))

        w = math.exp( (car_angle_deg - ptcl_angle_deg)**2 / std)
        final_weight = w * final_weight
    """
    #ohne yaw
    for (bln_color, bln_x, bln_y) in balloons:
        car_angle = angle_between_vecs( [c_y,c_x] , [bln_x, bln_y])
        ptcl_angle = angle_between_vecs( [p_x,c_y] , [bln_x, bln_y])
        weight = math.exp( (car_angle - ptcl_angle)**2 / std)
        final_weight = final_weight * weight
    """
    #print "weight" , final_weight
    return final_weight
    #print "angles: ", rad_to_deg(car_angle) , " : " , rad_to_deg(ptcl_angle)


def odom_callback(data):
    global old_x
    global old_y
    global old_yaw_deg
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw_deg = rad_to_deg(quaternion_to_yaw(data.pose.pose.orientation))


    # to move each particle by the motion vector of the car,
    # every particle must have the same yaw as the car, no?
    try:
        delta_x = x - old_x
        delta_y = y - old_y
        delta_yaw_deg = yaw_deg - old_yaw_deg

    # NameError occurs if old_x and old_y are not defined yet
    except NameError:
        old_x = x
        old_y = y
        old_yaw_deg = yaw_deg
        return

    d = dist(x,y,old_x,old_y)

    weights = []

    for i in range(len(pose_array)):
        paricle_old_x = pose_array[i].position.x
        paricle_old_y = pose_array[i].position.y
        paricle_old_yaw_deg = rad_to_deg(quaternion_to_yaw(pose_array[i].orientation))

        paricle_new_yaw_deg = paricle_old_yaw_deg + delta_yaw_deg

        V_x = cos(deg_to_rad(paricle_new_yaw_deg))
        V_y = sin(deg_to_rad(paricle_new_yaw_deg))
        
        V_x *= d
        V_y *= d
        print V_x , V_y
        paricle_new_x = paricle_old_x + V_x
        paricle_new_y = paricle_old_y + V_y

        pose_array[i].position.x = paricle_new_x
        pose_array[i].position.y = paricle_new_y
        pose_array[i].orientation = yaw_to_quaternion(deg_to_rad(paricle_new_yaw_deg))

        weight = calc_weight( (paricle_new_x,paricle_new_y,paricle_new_yaw_deg) , (x,y,yaw_deg))
        weights.append(weight)

        #print "paricle_new_yaw_deg" ,paricle_new_yaw_deg
    old_x = x
    old_y = y
    old_yaw_deg = yaw_deg

    s = 0
    for w in weights:
        s += w
    print "s" , s

    pub_pos.publish(create_ros_pose_array_object())



def main():
    global pub_pos
    rospy.init_node('my_little_nodey', anonymous=True)

    pub_pos = rospy.Publisher('/mcposearray', PoseArray, queue_size=1)
    initialize_pose_array()
    #/assignment6/odom
    rospy.Subscriber("/assignment6/odom", Odometry, odom_callback, queue_size=1)
    #rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    #rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
        #ros_pose_obj = create_ros_pose_array_object(pose_array)
        #pub_pos.publish(ros_pose_obj)
        #rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
