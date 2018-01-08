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


# create random particles
def initialize_pose_array():
    global pose_array
    pose_array = []
    for i in range(100):
        x = random.uniform(-arena_x, arena_x)
        y = random.uniform(-arena_y, arena_y)
        yaw = random.uniform(-math.pi, math.pi)
        pose_array.append(get_pose(x,y,yaw))

def create_grid():
    # 1. create the grid
    d = {} # this will hold our cells

    for i in range(len(pose_array)):

        p_x = pose_array[i].position.x
        p_y = pose_array[i].position.y
        p_yaw = quaternion_to_yaw( pose_array[i].orientation)

        # calc grid coordinates from particle
        grid_x = round(p_x * 10 ) #0.25)
        grid_y = round(p_y * 10 )#0.25)
        coords = (grid_x, grid_y)

        # append particle to corresponding grid
        if coords in d:
            d[coords].append( (p_x, p_y, p_yaw) )
        else:
            d[coords] = [(p_x, p_y, p_yaw)]

    # 2. find the cell with most particles:
    top_cell = []
    top_key = None
    for key in d:
        lst = d[key]
        if len(lst) > len(top_cell):
            top_cell = lst
            top_key = key
    # 3. sum x,y,yaw
    sum_y = 0
    sum_x = 0
    sum_yaw_cos = 0
    sum_yaw_sin = 0

    for (x,y,yaw) in top_cell:
        sum_x += x
        sum_y += y
        sum_yaw_sin += math.sin(yaw)
        sum_yaw_cos += math.cos(yaw)

    # 4. calc average
    print "len", len(top_cell)
    avg_x = sum_x / len(top_cell)
    avg_y = sum_y / len(top_cell)
    avg_yaw = math.atan2(sum_yaw_sin, sum_yaw_cos)

    return (avg_x,avg_y,avg_yaw)


def calc_weight(ptcl, bln_screen_array):
    (p_x,p_y,p_yaw) = ptcl

    final_weight = 1
    std = 10

    # car position in image
    (car_x, car_y) = (640 / 2, 480 / 2)

    for color, (bln_screen_x, bln_screen_y) in bln_screen_array:
        (bln_world_x, bln_world_y) = balloons[color]

        # vec car to ballon in image
        rel_car_x = bln_screen_x - car_x
        rel_car_y = bln_screen_y - car_y

        # vec (0,-1) to calc angle angle in real world
        y_minus = np.array((0,-1))
        rel_car = np.array((rel_car_x,rel_car_y))
        car_angle = py_ang(y_minus, rel_car)

        # vec particle to ballon in real world coordinates
        rel_ptcl_x = bln_world_x - p_x
        rel_ptcl_y = bln_world_y - p_y

        ptcl_arr = np.array((rel_ptcl_x, rel_ptcl_y))
        ptcl_yaw_vec = np.array((cos(p_yaw),sin(p_yaw)))

        ptcl_angle = py_ang(ptcl_arr, ptcl_yaw_vec)
        #calc weight
        w = math.exp( -((car_angle - ptcl_angle)**2 / std))
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

def create_odom_obj(x,y,yaw):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.pose.pose = Pose(Point(x, y, 0) , yaw_to_quaternion(yaw))
    return msg


def odom_callback(data):
    data.header.stamp = rospy.Time.now()
    odom_now_pub.publish(data)

    global old_x
    global old_y
    global old_yaw
    #odom data from car
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = quaternion_to_yaw(data.pose.pose.orientation)

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
        current_yaw = quaternion_to_yaw(pose_array[i].orientation) - delta_yaw
        # add some noise to angle:
        current_yaw = norm_rad(random.uniform(0.995 * current_yaw, 1.005 * current_yaw))

        vector_length = math.sqrt(delta_x ** 2 + delta_y ** 2)

        #make yaw to vector to scale it by vector_length
        yaw_vector = np.array((cos(current_yaw), sin(current_yaw)))

        # add some noise to vector
        vector_length = random.uniform(0.75 * vector_length, 1.25 * vector_length)

        yaw_vector = yaw_vector * vector_length

        pose_array[i].position.x += yaw_vector[0]
        pose_array[i].position.y += yaw_vector[1]
        pose_array[i].orientation = yaw_to_quaternion(current_yaw)

    old_x = x
    old_y = y
    old_yaw = yaw

    pub_pos.publish(create_ros_pose_array_object())

    (gx,gy,gyaw) = create_grid()
    odom = create_odom_obj(gx,gy,gyaw)
    odom_pub.publish(odom)


def callback(data):
    img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    detector.calculate_best_position(img)
    bln_array = detector.get_balloon_positions()
    bln_screen_coords = []
    for balloon in bln_array:
        bln_screen_coords.append((balloon[0][0], balloon[1]))

    weights = []
    for i in range(len(pose_array)):
        weight = calc_weight((pose_array[i].position.x, pose_array[i].position.y, quaternion_to_yaw(pose_array[i].orientation)), bln_screen_coords)
        weights.append(weight)
    final_weights = norm_weights(weights)
    resample(final_weights)
    create_grid()


def main():
    global pub_pos
    global bridge
    global detector
    global odom_pub
    global odom_now_pub
    odom_pub = rospy.Publisher("/mcpf_gps", Odometry, queue_size=200)
    odom_now_pub = rospy.Publisher("/odom_now", Odometry, queue_size=200)
    rospy.init_node('my_little_nodey', anonymous=True)

    pub_pos = rospy.Publisher('/mcposearray', PoseArray, queue_size=1)
    initialize_pose_array()

    bridge = CvBridge()

    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
    detector = BalloonDetector()
    rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
