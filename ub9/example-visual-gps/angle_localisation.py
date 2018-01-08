#!/usr/bin/env python2

import rospy
from balloon_detector import BalloonDetector
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
import numpy as np

import math


class ImageHandler:
    def __init__(self):
        self.image_pub_marked = rospy.Publisher("/assignment6/image_marked_ang", Image, queue_size=200, latch=True)
        self.odom_pub = rospy.Publisher("/assignment6/odom", Odometry, queue_size=200)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.detector = BalloonDetector()

        pose_covar = PoseWithCovariance(Pose(Point(0, 0, 0), Quaternion()), None)
        self.odom = Odometry(Header(frame_id='odom'), 'base_link', pose_covar, None)

    def callback(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        xy = self.detector.calculate_best_position(img)
        for bln in self.detector.balloon_positions:
            print bln
        # show detected points in published image
        self.detector.draw_markers(img)
        self.image_pub_marked.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        # Don't publish a pose if location can't be reliably determined
        if xy is None:
            print("No location found")
            return

        yaw_angle = self.detector.calculate_angle()

        # publish odometry message
        header = self.odom.header
        header.seq = data.header.seq
        header.stamp = data.header.stamp

        pose = self.odom.pose.pose
        pos = pose.position
        pos.x, pos.y = xy

        quaternion = pose.orientation
        quaternion.z, quaternion.w = math.sin(yaw_angle / 2), math.cos(yaw_angle / 2)

        self.odom_pub.publish(self.odom)

        # print position and yaw_angle as degrees in the terminal
        print('{:<30} yaw: {:6.1f}'.format(xy, np.rad2deg(yaw_angle)))


def main():
    rospy.init_node('example_visual_gps')
    ImageHandler()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()
