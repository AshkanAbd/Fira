#!/usr/bin/env python2.7

import rospy
import cv_bridge
import cv2 as cv
import sensor_msgs.msg
import numpy as np


class ImageProcess:
    min_dis = 0.5
    max_dis = 5.5
    node_name = None
    bridge = None
    depth_frame = None
    rgb_frame = None
    cloud_points = None
    lower_color = np.array([[[170, 75, 50]]])
    upper_color = np.array([[[180, 255, 200]]])

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('/camera/depth/image_raw', sensor_msgs.msg.Image, self.get_depth_image, queue_size=1000)
        rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, self.get_rgb_image, queue_size=1000)
        rospy.Subscriber('/camera/depth/points', sensor_msgs.msg.PointCloud2, self.get_cloud_point, queue_size=1000)
        self.combine_img()

    def combine_img(self):
        while True:
            if self.depth_frame is not None and self.rgb_frame is not None:
                frame = cv.bitwise_and(self.rgb_frame, self.rgb_frame, mask=self.depth_frame)
                frame_edge = cv.Canny(frame, 100, 200)
                _, contours, _ = cv.findContours(frame_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                frame1 = np.zeros(frame.shape, frame.dtype)
                for cnt in contours:
                    if cv.contourArea(cnt) > 20:
                        cv.drawContours(frame1, [cnt], 0, (255, 0, 0), 2)
                cv.imshow('a', self.depth_frame)
                cv.imshow('b', self.rgb_frame)
                cv.imshow('final', frame)
                cv.imshow('final1', frame_edge)
                cv.imshow('final2', frame1)
                cv.waitKey(1)

    def get_cloud_point(self, points):
        self.cloud_points = points

    def get_depth_image(self, image):
        frame = self.bridge.imgmsg_to_cv2(image, '32FC1')
        frame = (255 * ((frame - self.max_dis) / (self.max_dis - self.min_dis)))
        frame = frame.astype(np.uint8)
        frame = cv.medianBlur(frame, 5)
        frame = cv.bilateralFilter(frame, 9, 75, 75)
        frame[frame < 40] = 0
        frame[frame > 50] = 0
        self.depth_frame = frame

    def get_rgb_image(self, image):
        frame = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        frame = cv.medianBlur(frame, 5)
        frame = cv.bilateralFilter(frame, 9, 75, 75)
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame = cv.inRange(frame_hsv, self.lower_color, self.upper_color)
        self.rgb_frame = frame


if __name__ == '__main__':
    ImageProcess('image')
    rospy.spin()
