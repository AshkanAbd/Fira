#!/usr/bin/env python2

import rospy
import actionlib.msg
import actionlib
import nav_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg


class SendGoal:
    map_info = None
    move_base_goal = None
    client = None
    final_goal = ()

    def __init__(self, node_name='AutoMoveBase', map_topic="base_map"):
        rospy.init_node(node_name)
        self.setup_move_base()
        self.get_map(rospy.wait_for_message('/' + map_topic, nav_msgs.msg.OccupancyGrid))
        rospy.Subscriber('/' + map_topic, nav_msgs.msg.OccupancyGrid, self.get_map)

    def get_map(self, msg):
        self.map_info = msg

    def setup_move_base(self):
        self.client = actionlib.SimpleActionClient('/aura/move_base', move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()
        self.move_base_goal = move_base_msgs.msg.MoveBaseGoal()

    def set_goal(self, goal_x, goal_y):
        self.final_goal = (goal_x, goal_y)
        self.send_goal(goal_x, goal_y)

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
            self.send_goal(self.final_goal[0], self.final_goal[1])

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.map_info.info.origin.position.x) / self.map_info.info.resolution)
        map_y = round((robot_y - self.map_info.info.origin.position.y) / self.map_info.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.map_info.info.resolution) + self.map_info.info.origin.position.x
        robot_y = (map_y * self.map_info.info.resolution) + self.map_info.info.origin.position.y
        return robot_y, robot_x


if __name__ == '__main__':
    auto_move = SendGoal('a','aura/base_map')
    auto_move.set_goal(5.0, 0)
    rospy.spin()
