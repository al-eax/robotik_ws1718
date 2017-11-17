#!/usr/bin/env python
import rospy
import signal
import sys
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def setCell(x,y):
    global grid

    res = grid.info.resolution
    x_scaled = (x * 1.0 / res) + grid.info.width/2
    y_scaled = (y * 1.0 / res) + grid.info.height/2

    if x_scaled >= grid.info.width or x_scaled < 0 or y_scaled >= grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * grid.info.height
    grid.data[int(offset) + int(round(y_scaled) - 1)] = 100


def visualize():
    global grid
    line = ""
    for x in range(grid.info.width):
        for y in range(grid.info.height):
            index = conv_2d_1d(x,y,grid.data,grid.info.width)
            if grid.data[index] == -1:
                line += "0-1 "
            if grid.data[index] == -0:
                line += "000 "
            if grid.data[index] == 100:
                line += "100 "
        line += "\n"
    print line
    print "##############################################"

def init():
    global grid
    global pub_grid
    pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)
    grid = OccupancyGrid()
    grid.info.width = 50
    grid.info.height = 50
    grid.info.origin.position.z = 0
    grid.info.origin.orientation.x = 0
    grid.info.origin.orientation.y = 0
    grid.info.origin.orientation.z = 0
    grid.info.origin.orientation.w = 1
    grid.info.origin.position.x = int(-1.0 * grid.info.width / 2.0) * grid.info.resolution
    grid.info.origin.position.y = int(-1.0 * grid.info.height / 2.0) * grid.info.resolution


    grid.info.resolution = 0.5
    grid.data = [UNKNOWN for i in range(grid.info.width*grid.info.height)]
    rospy.init_node('foobar', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)

#https://scipython.com/book/chapter-6-numpy/examples/creating-a-rotation-matrix-in-numpy/
def rotate(v,a):
    c, s = np.cos(a), np.sin(a)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    return R*v

def f(x,m):
    return x*m

UNKNOWN = -1
FREE = 0
OBST = 100

def addValidLidar(a,l):
    global grid
    #print grid
    e1 = np.array([[1],[0]])
    vec = rotate(e1*l,a)
    m = vec[0] / vec[1] #x/y
    free = []
    # irgendwo + 50, 50 um in mittelpunkt zu schieben
    for x in range(vec[0]):
        y = int(f(x,m) + grid.info.origin.position.y)
        x += grid.info.origin.position.x
        free.append((x,y))
        index = conv_2d_1d(x,y , grid.data, grid.info.width)
        grid.data[int(index)] = FREE
        setCell(x,y)

    #index = int(conv_2d_1d(vec[0],vec[1], grid.data, grid.info.width))
    #grid.data[index] = OBST




def pubGrid():
    global grid
    global pub_grid
    pub_grid.publish(grid)

def conv_2d_1d(x,y,data,w):
    l = len(data)
    return int(x*l/w + y)

def resetGrid():
    global grid
    grid.data[:] = UNKNOWN

def scanCallback(data):
    global grid
    rs = data.ranges
    for a in range(len(rs)): #a = angle
        r = rs[a] #radius
        if not math.isinf(r):
            addValidLidar(a,r)
    pubGrid()
    visualize()

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
