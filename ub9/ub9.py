#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from balloon_detector import BalloonDetector
import numpy as np
import numpy.linalg as la
import math
import time
from math import sqrt, cos, sin,asin, acos, atan2
import random
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose


# size of arena: 10x10
# divide by 2 to get a number between -5 and 5
# so it is centered on (0,0)
arena_x = 10.0/2.0
arena_y = 10.0/2.0


balloons = {"purple": (2.255, 2.425),
            "red": (3.515, 3.04),
            "blue": (4.14, 1.79),
            "green": (2.265, 1.165)}

final_weights = []

def quaternion_to_yaw(q):
    return acos(q.w) * 2 * np.sign(q.z)

def yaw_to_quaternion(yaw):
    return Quaternion(0,0,sin(yaw / 2) , cos(yaw / 2))

def angle(x,y):
    return atan2(y,x)

def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def norm_rad(yaw):
    if yaw > math.pi:
        return yaw - 2 * math.pi
    elif yaw < -math.pi:
        return yaw + 2 * math.pi
    else:
        return yaw

# https://newtonexcelbach.com/2014/03/01/the-angle-between-two-vectors-python-version/
def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)


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



def calc_weight(ptcl, bln_screen_array):
    (p_x,p_y,p_yaw) = ptcl

    final_weight = 1
    std = 10

    (car_x, car_y) = (640 / 2, 480 / 2)

    #mit yaw
    for color, (bln_screen_x, bln_screen_y) in bln_screen_array:
        
        (bln_world_x, bln_world_y) = balloons[color]
        
        rel_car_x = bln_screen_x - car_x
        rel_car_y = bln_screen_y - car_y
        
        car_angle = angle(rel_car_x,rel_car_y)


        rel_ptcl_x = bln_world_x - p_x
        rel_ptcl_y = bln_world_y - p_y
        ptcl_arr = np.array((rel_ptcl_x, rel_ptcl_y))
        ptcl_yaw_vec = np.array((cos(p_yaw),sin(p_yaw)))
        
        ptcl_angle = py_ang(ptcl_arr, ptcl_yaw_vec)

        
        w = math.exp( -((car_angle - ptcl_angle)**2 / std))




        #car_angle = car_angle - c_yaw
        #ptcl_angle = ptcl_angle - p_yaw
        
        #car_arr = np.array((rel_car_x, rel_car_y))
        #ptcl_arr = np.array((rel_ptcl_x, rel_ptcl_y))
        #car_yaw_arr = np.array((cos(c_yaw), sin(c_yaw)))
        #ptcl_yaw_arr = np.array((cos(p_yaw), sin(p_yaw)))

        #car_angle_v2 = py_ang(car_arr, car_yaw_arr)
        #ptcl_angle_v2 = py_ang(ptcl_arr, ptcl_yaw_arr)

        #w2 = math.exp( -((car_angle_v2 - ptcl_angle_v2)**2 / std))
        
        #print "car_angle = ", car_angle, "; car_angle_v2 = ", car_angle_v2
        #print "ptcl_angle = ", ptcl_angle, "; ptcl_angle_v2 = ", ptcl_angle_v2
        #print "w = ", w, "; w2 = ", w2
        #print "-------------------------------------------------------"

        final_weight = w * final_weight

    return final_weight

def norm_weights(weights):
    s = np.sum(weights)
    for i in range(len(weights)):
        weights[i] = weights[i] / s
    return weights


def resample(weights):
    global pose_array
    
    cell_size = 1.0 / len(weights)
    cell_center = cell_size / 2.0

    hold = []

    weight_index = 0
    weights_sum = 0

    for i in np.arange(cell_center, 1, cell_size):
        while weights_sum + weights[weight_index] < i:
            weights_sum += weights[weight_index]
            weight_index += 1
        hold.append(weight_index)
        
    for i in range(len(hold)):
        pose_obj = get_pose(pose_array[hold[i]].position.x, pose_array[hold[i]].position.y, quaternion_to_yaw(pose_array[hold[i]].orientation))
        pose_array[i] = pose_obj


def odom_callback(data):
    global old_x
    global old_y
    global old_yaw
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)

    # to move each particle by the motion vector of the car,
    # every particle must have the same yaw as the car, no?
    try:
        delta_x = x - old_x
        delta_y = y - old_y
        delta_yaw = yaw - old_yaw
    # NameError occurs if old_x and old_y are not defined yet
    except NameError:
        old_x = x
        old_y = y
        old_yaw = yaw
        return

    weights = []
    for i in range(len(pose_array)):
        # adding some deviation to x and y
        current_yaw = quaternion_to_yaw(pose_array[i].orientation) - delta_yaw
        current_yaw = norm_rad(random.uniform(0.9 * current_yaw, 1.1 * current_yaw))

        yaw_vector = np.array((cos(current_yaw), sin(current_yaw)))

        vector_length = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # add noise to vector

        vector_length = random.uniform(0.75 * vector_length, 1.25 * vector_length)

        yaw_vector = yaw_vector * vector_length

        pose_array[i].position.x += yaw_vector[0]
        pose_array[i].position.y += yaw_vector[1]
        pose_array[i].orientation = yaw_to_quaternion(current_yaw)

    old_x = x
    old_y = y
    old_yaw = yaw
    pub_pos.publish(create_ros_pose_array_object())

def callback(data):
    img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    detector.calculate_best_position(img)
    bln_array = detector.get_balloon_positions()
    bln_screen_coords = []
    for balloon in bln_array:
        bln_screen_coords.append((balloon[0][0], balloon[1]))
    #print bln_screen_coords
    #print len(bln_screen_coords)
    #print "-------------------------------------------------------------"
    
    weights = []
    for i in range(len(pose_array)):
        weight = calc_weight((pose_array[i].position.x,pose_array[i].position.y, quaternion_to_yaw(pose_array[i].orientation)), bln_screen_coords)
        weights.append(weight)
    final_weights = norm_weights(weights)
    resample(final_weights)
    

def main():
    global pub_pos
    global bridge
    global detector
    rospy.init_node('my_little_nodey', anonymous=True)

    pub_pos = rospy.Publisher('/mcposearray', PoseArray, queue_size=1)
    initialize_pose_array()

    bridge = CvBridge()

    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
    
    detector = BalloonDetector()
    
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback, queue_size=1)


    #rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
        #ros_pose_obj = create_ros_pose_array_object(pose_array)
        #pub_pos.publish(ros_pose_obj)
        #rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
