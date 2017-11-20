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

#x,y in meters
def setCell(x,y,val):
    global grid
    res = grid.info.resolution
    #x = -x
    #y = -y
    x_scaled = x * 1.0 / res + grid.info.width / 2.0   # (x * 1.0 / res) + grid.info.width/2.0 #x + grid.info.width / 2.0 #
    y_scaled = y * 1.0 / res + grid.info.height / 2.0   #(y * 1.0 / res) + grid.info.height/2.0 #y + grid.info.height / 2.0  #

    if x_scaled >= grid.info.width or x_scaled < 0 or y_scaled >= grid.info.height or y_scaled < 0:
        return
    offset = (int(round(x_scaled)) - 1) * grid.info.height
    if (grid.data[int(offset) + int(round(y_scaled) - 1)] == OBST):
        return
    grid.data[int(offset) + int(round(y_scaled) - 1)] = val



def init():
    global grid
    global pub_grid
    pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=1)
    grid = OccupancyGrid()
    grid.info.resolution = 1/10.0 # 10 Kaestchen pro M
    grid.info.width = 120 #12m
    grid.info.height = 120 #12m
    grid.info.origin.orientation.x = 0
    grid.info.origin.orientation.y = 0
    grid.info.origin.orientation.z = 0
    grid.info.origin.orientation.w = 1
    grid.info.origin.position.x = 0
    grid.info.origin.position.y = 0
    grid.info.origin.position.z = 0.1

    grid.data = [UNKNOWN for i in range(grid.info.width*grid.info.height)]
    rospy.init_node('foobar', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=1)

#https://scipython.com/book/chapter-6-numpy/examples/creating-a-rotation-matrix-in-numpy/
def rotate(v,a):
    rad = (a * np.pi / 180.0) + np.pi / 2
    c, s = np.cos(rad), np.sin(rad)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    return np.dot(R,v)


def get_line(start, end):
    #http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

def f(x,m):
    return x*m

UNKNOWN = -1
FREE = 0
OBST = 100  #obst ist gesund

def addValidLidar(a,l):
    global grid
    #print grid
    e1 = np.array([[1.0],[0.0]])
    vec = rotate(e1*l, a)
    #print "fpp",vec
    if vec[1] == 0:
        m = 0
    else:
        m = vec[1] / vec[0] #y/x = steigung
    '''
        for x in range(vec[0]):
            y = float(f(x,m))
            setCell(x,y,FREE)
        for y in range(vec[1]):
            if m == 0:
                break
            x = float(f(y,1/m))
            setCell(x,y,FREE)
    '''
    l = get_line((0,0),(int(vec[0] * 100),int(vec[1] * 100)))
    u = [(x / 100.0, y / 100.0 ) for (x,y) in l]
    for (x,y) in u:
        setCell(x,y,FREE)
    setCell((vec[0]),(vec[1]),OBST)

def setInfWhite(a):
    global grid
    e1 = np.array([[1.0],[0.0]])
    inf_length = min(grid.info.height, grid.info.width)
    vec = rotate(e1*inf_length, a)
    if vec[1] == 0:
        m = 0
    else:
        m = vec[1] / vec[0] #x/y = steigung
    for x in range(vec[0]):
        y = float(f(x,m))
        #setCell(-x,y,FREE)
    for y in range(vec[1]):
        x = float(f(y,1/m))
        #setCell(-x,y,FREE)
    l = get_line((0,0),(int(vec[0] * 100),int(vec[1] * 100)))
    u = [(x / 100.0, y / 100.0 ) for (x,y) in l]
    for (x,y) in u:
        setCell(x,y,FREE)

def pubGrid():
    global grid
    global pub_grid
    pub_grid.publish(grid)
    resetGrid()

def conv_2d_1d(x,y,data,w):
    l = len(data)
    return int(x*l/w + y)

def resetGrid():
    global grid
    grid.data = [UNKNOWN for i in range(grid.info.width*grid.info.height)]

def scanCallback(data):
    global grid
    #print data.range_min, data.range_max max = 6 meters
    rs = data.ranges
    for a in range(len(rs)): #a = angle
        r = rs[a] #radius
        if not math.isinf(r) and data.intensities[a] != 0:
            #print a,r
            addValidLidar(a,r)
        else:
            setInfWhite(a,)
    pubGrid()
    #visualize()

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
