#!/usr/bin/env python2

import rospy
import numpy as np
import nav_msgs.msg
import Queue


class CostMapCreator:
    publish_map = None
    map_publisher = None

    def __init__(self, map_topic, publish_topic):
        self.publish_map = nav_msgs.msg.OccupancyGrid()
        self.map_publisher = rospy.Publisher(publish_topic, nav_msgs.msg.OccupancyGrid, queue_size=1000)
        rospy.Subscriber(map_topic, nav_msgs.msg.OccupancyGrid, self.get_map, queue_size=1000)

    def get_map(self, map_msg):
        self.publish_map.header = map_msg.header
        self.publish_map.info = map_msg.info
        data = np.asarray(map_msg.data, dtype=np.int8)
        reshaped = data.reshape((map_msg.info.height, map_msg.info.width))
        ones = np.where(reshaped == 100)
        for i in xrange(len(ones[0])):
            for j in xrange(-2, 3, 1):
                for k in xrange(-2, 3, 1):
                    if ones[0][i] + j < map_msg.info.height and ones[1][i] + k < map_msg.info.width:
                        if reshaped[ones[0][i] + j, ones[1][i] + k] == 0:
                            reshaped[ones[0][i] + j, ones[1][i] + k] = 10
        self.publish_map.data = reshaped.reshape(map_msg.info.height * map_msg.info.width).tolist()
        self.map_publisher.publish(self.publish_map)

    def spin(self):
        rospy.spin()


class GlobalPlan:
    odom = None
    costmap = None
    end_pose = None
    height = None
    width = None

    def __init__(self, costmap_topic, odom_topic, end_pose):
        first_map = rospy.wait_for_message(costmap_topic, nav_msgs.msg.OccupancyGrid)
        self.get_odom(rospy.wait_for_message(odom_topic, nav_msgs.msg.Odometry))
        self.height = first_map.info.height
        self.width = first_map.info.width
        self.costmap = first_map
        self.end_pose = convert_from_robot_to_map(end_pose[0], end_pose[1], first_map)
        rospy.Subscriber(costmap_topic, nav_msgs.msg.OccupancyGrid, self.get_costmap, queue_size=1000)
        rospy.Subscriber(odom_topic, nav_msgs.msg.Odometry, self.get_odom, queue_size=1000)

    def get_plan(self):
        reshaped_data = np.asarray(self.costmap.data).reshape((self.height, self.width))
        start_pose = convert_from_robot_to_map(self.odom.y, self.odom.x, self.costmap)
        current = start_pose
        main_queue = [current]
        close_list = set()
        plan_map = {}
        found = False
        while True:
            if len(main_queue) == 0:
                break
            if self.end_pose in close_list:
                found = True
                break
            current = main_queue.pop(0)
            close_list.add(current)
            neighbors = [(current[0] - 1, current[1]), (current[0] + 1, current[1]), (current[0], current[1] - 1),
                         (current[0], current[1] + 1)]
            for cell in neighbors:
                if self.height > cell[0] > -1 and self.width > cell[1] > -1:
                    # if self.valid_cell(cell, reshaped_data):
                    if cell not in close_list:
                        if cell not in main_queue:
                            if reshaped_data[int(cell[0]), int(cell[1])] == 0:
                                main_queue.append(cell)
                                plan_map[cell] = current
        return found, plan_map, start_pose

    def generate_plan(self, plan_map, start_pose):
        final_plan = list()
        nxt = self.end_pose
        while True:
            if nxt[0] == start_pose[0] and nxt[1] == start_pose[1]:
                break
            nxt = plan_map[nxt]
            final_plan.append(nxt)
        final_plan.reverse()
        return final_plan

    def valid_cell(self, cell, costmap):
        for i in xrange(-2, 3, 1):
            for j in xrange(-2, 3, 1):
                if self.height - 2 > cell[0] + i > -1:
                    if -1 < cell[1] + j < self.width - 2:
                        if costmap[int(cell[0]) + i, int(cell[1]) + j] != 0:
                            return False
        return True

    def get_costmap(self, map_msg):
        self.costmap = map_msg

    def get_odom(self, odom_msg):
        self.odom = odom_msg.pose.pose.position


def convert_from_robot_to_map(robot_y, robot_x, map_info):
    map_x = round((robot_x - map_info.info.origin.position.x) / map_info.info.resolution)
    map_y = round((robot_y - map_info.info.origin.position.y) / map_info.info.resolution)
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x, map_info):
    robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


if __name__ == '__main__':
    rospy.init_node('test_cost')
    costmap = CostMapCreator('/map', '/costmap')
    plan = GlobalPlan('/costmap', '/odom', (0, 5))
    a, b, c = plan.get_plan()
    if a:
        print(plan.generate_plan(b, c))
    # else:
    #     print("Error")
    costmap.spin()
