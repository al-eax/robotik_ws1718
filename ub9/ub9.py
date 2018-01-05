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

def quaternion_to_yaw(q):
    return acos(q.w) * 2 * np.sign(q.z)

def yaw_to_quaternion(yaw):
    return Quaternion(0,0,sin(yaw / 2) , cos(yaw / 2))

def angle(x,y):
    return atan2(y,x)

def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)


def create_ros_pose_array_object():
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
    for i in range(100):
        x = random.uniform(-arena_x, arena_x)
        y = random.uniform(-arena_y, arena_y)
        yaw = random.uniform(-math.pi, math.pi)
        pose_array.append(get_pose(x,y,yaw))


def odom_callback(data):
    global old_x
    global old_y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)
    
    # to move each particle by the motion vector of the car, 
    # every particle must have the same yaw as the car, no?
    try:
        delta_x = x - old_x
        delta_y = y - old_y
    # NameError occurs if old_x and old_y are not defined yet
    except NameError:
        old_x = x
        old_y = y
        return
    
    for i in range(len(pose_array)):
        # adding some deviation to x and y
        deviation_x = random.uniform(-delta_x*0.05,delta_x*0.05)
        deviation_y = random.uniform(-delta_y*0.05,delta_y*0.05)
        # calculate the angle of the new vector and add that to the current yaw
        deviation_yaw = angle(deviation_x, deviation_y)
        new_yaw = 0.1 * deviation_yaw + yaw
        
        pose_array[i].position.x += delta_x + deviation_x
        pose_array[i].position.y += delta_y + deviation_y
        pose_array[i].orientation = yaw_to_quaternion(new_yaw)

    old_x = x
    old_y = y
    pub_pos.publish(create_ros_pose_array_object())
    
    
    
def main():
    global pub_pos
    rospy.init_node('my_little_nodey', anonymous=True)
    
    pub_pos = rospy.Publisher('/mcposearray', PoseArray, queue_size=1)
    initialize_pose_array()
    
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    
    #rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
        #ros_pose_obj = create_ros_pose_array_object(pose_array)
        #pub_pos.publish(ros_pose_obj)
        #rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()
