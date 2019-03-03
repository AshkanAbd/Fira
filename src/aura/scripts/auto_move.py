#!/usr/bin/env python2

import rospy
import actionlib.msg
import actionlib
import nav_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import Controller
import sensor_msgs.msg
import cv_bridge
import cv2 as cv
import numpy as np
import std_msgs.msg


class SendGoal:
    map_info = None
    move_base_goal = None
    client = None
    final_goal = ()
    max_try = 5
    wait = 0
    wait_for_obj = False
    final = False
    nxt_goal = None
    reached = False
    bridge = None
    clear_pub = None
    last_time = None

    def __init__(self, node_name='AutoMoveBase', map_topic="base_map", depth_topic='depth_image'):
        rospy.init_node(node_name)
        self.setup_move_base()
        self.get_map(rospy.wait_for_message('/' + map_topic, nav_msgs.msg.OccupancyGrid))
        rospy.Subscriber('/' + map_topic, nav_msgs.msg.OccupancyGrid, self.get_map)
        # rospy.Subscriber('/' + depth_topic, sensor_msgs.msg.Image, self.get_img)
        self.clear_pub = rospy.Publisher('/aura/clear_map', std_msgs.msg.Empty, queue_size=1000)
        self.bridge = cv_bridge.CvBridge()

    def get_map(self, msg):
        self.map_info = msg
        reshape = np.asarray(msg.data).reshape((msg.info.height, msg.info.width))
        odom = rospy.wait_for_message('/aura/odom', nav_msgs.msg.Odometry)
        odom = odom.pose.pose.position
        map_y, map_x = self.convert_from_robot_to_map(odom.y, odom.x)
        map_y = int(map_y)
        map_x = int(map_x)
        sub = reshape[map_y - 10:map_y + 10, map_x - 10:map_x + 10]
        if self.wait_for_obj:
            if len(np.where(sub == 100)[0]) == 0:
                # if (not self.last_time is None) and (rospy.Time.now().to_sec() - self.last_time >= 1):
                c = Controller.Controller('/cmd_vel_mux/input/navi')
                c.publish(0.5, 0, 2)
                self.set_goal(self.nxt_goal, None)
                self.nxt_goal = None
                self.wait_for_obj = False
                e = std_msgs.msg.Empty()
                for i in xrange(3):
                    self.clear_pub.publish(e)
            else:
                e = std_msgs.msg.Empty()
                for i in xrange(3):
                    self.clear_pub.publish(e)
                self.last_time = rospy.Time.now().to_sec()

    def get_img(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, '32FC1')
        tmp = np.where(frame != np.float32('nan'))
        if self.wait_for_obj:
            # if len(tmp[0]) > 0:
            #     if self.wait == 0:
            #         print("Object detected")
            #         self.wait = 1
            # else:
            #     if self.wait == 1:
            print("Object gone")
            self.wait = 2
            c = Controller.Controller('/cmd_vel_mux/input/navi')
            c.publish(0.7, 0, 1)
            c.publish(0, 0, 1)
            self.set_goal(self.nxt_goal, None)
            self.nxt_goal = None
            self.wait_for_obj = False

    def setup_move_base(self):
        self.client = actionlib.SimpleActionClient('/aura/move_base', move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()
        self.move_base_goal = move_base_msgs.msg.MoveBaseGoal()

    def set_goal(self, (goal1), (goal2)):
        if not goal2 is None:
            self.nxt_goal = goal2
        self.final_goal = (goal1[0], goal1[1])
        self.send_goal(goal1[0], goal1[1])

    def send_goal(self, goal_x, goal_y):
        # self.client.cancel_all_goals()
        goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = "/aura/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1
        print(str(goal_x) + ' ,' + str(goal_y) + ' accepted by move_base')
        self.move_base_goal.target_pose = goal
        self.client.send_goal(self.move_base_goal, self.goal_status)

    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6
    # ---RECALLING=7---RECALLED=8---LOST=9
    def goal_status(self, data1, data2):
        print('goal status = ' + str(data1))
        if data1 != 3:
            if self.max_try > 0:
                self.max_try = self.max_try - 1
                self.send_goal(self.final_goal[0], self.final_goal[1])
        else:
            if self.nxt_goal is None:
                controller = Controller.Controller('/cmd_vel_mux/input/navi')
                controller.publish(0, 45, 1)
                controller.publish(0.5, 0, 2)
                controller.publish(0, 0, 1)
            else:
                print("Now wait for object")
                self.wait_for_obj = True

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.map_info.info.origin.position.x) / self.map_info.info.resolution)
        map_y = round((robot_y - self.map_info.info.origin.position.y) / self.map_info.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.map_info.info.resolution) + self.map_info.info.origin.position.x
        robot_y = (map_y * self.map_info.info.resolution) + self.map_info.info.origin.position.y
        return robot_y, robot_x


if __name__ == '__main__':
    auto_move = SendGoal('a', 'aura/base_map', 'aura/depth_image')
    # auto_move.set_goal((2.3, -1.1), (5.2, 0))
    auto_move.set_goal((5.5, 0), None)
    rospy.spin()
