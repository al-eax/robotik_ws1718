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


def init():
    global grid
    global pub_grid
    pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)
    grid = OccupancyGrid()
    grid.info.width = 100
    grid.info.height = 100
    grid.info.origin.position = (50,50,0)

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
    e1 = np.array([[1],[0]])
    vec = rotate(e1*l,a)
    m = vec[0] / vec[1] #x/y
    free = []
    for x in range(ve[0]):
        y = f(x,m)
        free.append((x,int(y)))
        index = conv_2d_1d(x,int(y) , grid.data, grid.info.width)
        grid.data[index] = FREE
    grid[v[0],v[1]] = OBST

def pubGrid():
    global grid
    global pub_grid
    pub_grid.publish(grid)

def conv_2d_1d(x,y,data,w,h):
    l = len(data)
    return x*l/w + y

def resetGrid():
    global grid
    grid[:] = UNKNOWN

def scanCallback(data):
    global grid
    grid = OccupancyGrid()
    rs = data.ranges
    for i in range(len(rs)):
        r = rs[i]
        if not math.isinf(r):
            addValidLidar(r)


if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
