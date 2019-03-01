#!/usr/bin/env python2.7

import rospy
import geometry_msgs.msg
import math
import Controller
import numpy as np
import nav_msgs.msg


class Move:
    odome = None
    map_sihdir = None
    controller = None
    dst = (5.0, 0)

    def __init__(self, odom_topic, map_topic):
        self.controller = Controller.Controller('test')
        rospy.Subscriber(odom_topic, nav_msgs.msg.Odometry, self.get_odom)
        rospy.Subscriber(map_topic, nav_msgs.msg.OccupancyGrid, self.get_map)
        self.controller.publish(0.2, 0, 1)
        rospy.spin()

    def get_odom(self, odom):
        self.odome = odom

    def get_map(self, map_obj):
        self.map_sihdir = map_obj
        sikim = np.asarray(map_obj.data, dtype=np.int8).reshape(40, 70)
        current = self.convert_from_robot_to_map(self.odome.pose.pose.position.y, self.odome.pose.pose.position.x)
        q = 0
        s = []
        s.append(current)
        if (((pow(self.odome.pose.pose.position.y - 5, 2)) + (pow(self.odome.pose.pose.position.x - 5, 2))) <= 0.6):
            s = ((pow(self.odome.pose.pose.position.y - 5, 2)) + (pow(self.odome.pose.pose.position.x - 5, 2)))
            self.controller.publish(s, q, 1)
        elif sikim[int((current[0]))][int((current[1])) + 5] != 100:
            current_sihdir = sikim[int((current[0]))][int((current[1])) + 3]
            if (sikim[int((current[0])) + 3][int((current[1]))] != 100 and sikim[int((current[0])) - 3][
                int((current[1]))] != 100):
                self.go_ha_go()
            elif (sikim[int((current[0])) + 3][int((current[1]))] == 100):
                self.chap()
                self.controller.publish(0, 45, 1)


            elif (sikim[int((current[0])) - 3][int((current[1]))] == 100):
                self.rast()
                self.controller.publish(0, 45, 1)

        # else:
        #     if (sikim[int((current[0])) + 8][int((current[1]))] != -1):
        #         q -= 45
        #         self.chap()
        #     elif (sikim[int((current[0])) - 8][int((current[1]))] != -1):
        #         q += 45
        #         self.rast()

    def chap(self):
        self.controller.publish(0.3, -45, 1)

    def rast(self):
        self.controller.publish(0.3, 45, 1)

    def go_ha_go(self):
        self.controller.publish(0.3, 0, 1)

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.map_sihdir.info.origin.position.x) / self.map_sihdir.info.resolution)
        map_y = round((robot_y - self.map_sihdir.info.origin.position.y) / self.map_sihdir.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.map_sihdir.info.resolution) + self.map_sihdir.info.origin.position.x
        robot_y = (map_y * self.map_sihdir.info.resolution) + self.map_sihdir.info.origin.position.y
        return robot_y, robot_x


if __name__ == '__main__':
    Move('/odom', '/aura/base_map')
