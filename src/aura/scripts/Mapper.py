#!/usr/bin/env python2

import rospy
import roslib
import tf
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import std_msgs.msg


class Mapper:
    node_name = None
    base_map = np.array([])
    rate = None
    map_publisher = None
    publish_obj = None
    br = None

    def __init__(self, node_name):
        self.node_name = node_name
        self.br = tf.TransformBroadcaster()
        rospy.init_node(node_name)
        self.rate = rospy.Rate(20)
        self.map_publisher = rospy.Publisher('base_map', nav_msgs.msg.OccupancyGrid, queue_size=1000)
        self.prepare_map()
        rospy.Subscriber('/add_to_map', std_msgs.msg.Float64MultiArray, self.add_to_map, queue_size=10000)
        self.publish()

    def add_to_map(self, msg):
        pose = self.convert_from_robot_to_map1(msg.data[1], msg.data[0])
        if not np.isnan(pose):
            self.base_map[int(pose)] = int(msg.data[2])

    def prepare_map(self):
        self.publish_obj = nav_msgs.msg.OccupancyGrid()
        self.publish_obj.header.frame_id = 'map'
        self.publish_obj.info.height = 500
        self.publish_obj.info.width = 800
        self.publish_obj.info.resolution = 0.015
        self.publish_obj.info.origin.orientation.x = 0
        self.publish_obj.info.origin.orientation.y = 0
        self.publish_obj.info.origin.orientation.z = 0
        self.publish_obj.info.origin.position.x = -3
        self.publish_obj.info.origin.position.y = -(250 * self.publish_obj.info.resolution)
        self.publish_obj.info.origin.position.z = 0
        self.base_map = np.zeros((800 * 500), dtype=np.uint8)

    def publish(self):
        while not rospy.is_shutdown():
            self.publish_obj.header.stamp = rospy.Time.now()
            self.publish_obj.data = self.base_map.tolist()
            self.map_publisher.publish(self.publish_obj)
            self.rate.sleep()

    def convert_from_robot_to_map1(self, robot_y, robot_x):
        map_x = round((robot_x - self.publish_obj.info.origin.position.x) / self.publish_obj.info.resolution)
        map_y = round((robot_y - self.publish_obj.info.origin.position.y) / self.publish_obj.info.resolution)
        return (map_y * self.publish_obj.info.width) + map_x

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.publish_obj.info.origin.position.x) / self.publish_obj.info.resolution)
        map_y = round((robot_y - self.publish_obj.info.origin.position.y) / self.publish_obj.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.publish_obj.info.resolution) + self.publish_obj.info.origin.position.x
        robot_y = (map_y * self.publish_obj.info.resolution) + self.publish_obj.info.origin.position.y
        return robot_y, robot_x


if __name__ == '__main__':
    Mapper('mapper')
    rospy.spin()
