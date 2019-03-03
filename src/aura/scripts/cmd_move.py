#!/usr/bin/env python2

import numpy as np
import rospy
import actionlib.msg
import actionlib
import nav_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import Controller
import planner
import math


class Move:
    plan = None
    controller = None
    costmap = None

    def __init__(self, map_topic, odom_topic, costmap_topic, cmd_topic, goal):
        self.costmap = planner.CostMapCreator(map_topic, costmap_topic)
        self.plan = planner.GlobalPlan(costmap_topic, odom_topic, goal)
        self.controller = Controller.Controller(cmd_topic)
        self.start()
        self.costmap.spin()

    def request_plan(self):
        valid, plan_map, start_pose = self.plan.get_plan()
        if valid:
            return True, self.plan.generate_plan(plan_map, start_pose), start_pose
        return False, None, start_pose

    def debug(self, pub, plan):
        data = np.asarray(self.plan.costmap.data)
        r = rospy.Rate(10)
        for cell in plan:
            pose = cell[0] * self.plan.costmap.info.width + cell[1]
            data[int(pose)] = 100
        msg = self.plan.costmap
        msg.data = data.tolist()
        for i in xrange(50):
            pub.publish(msg)
            r.sleep()

    def start(self):
        r = rospy.Rate(0.5)
        pub = rospy.Publisher('/bfs', nav_msgs.msg.OccupancyGrid, queue_size=1000)
        while True:
            valid, plan, start = self.request_plan()
            if not valid:
                print "E"
                continue
            sub_plan = plan[:10]
            self.debug(pub, sub_plan)
            o_x, o_y = planner.convert_from_map_to_robot(sub_plan[len(sub_plan) - 1][0], sub_plan[len(sub_plan) - 1][1],
                                                         self.plan.costmap)
            o_x1, o_y1 = planner.convert_from_map_to_robot(start[1], start[0], self.plan.costmap)
            dst = math.sqrt(((o_x - o_x1) ** 2) + ((o_y - o_y1) ** 2))
            if o_x1 - o_x != 0:
                m = (o_y1 - o_y) / (o_x1 - o_x)
                degree = math.degrees(math.atan(m))
                if degree > 0:
                    if o_y1 - o_y < 0:
                        self.controller.publish(dst / 5, degree / 5, 5)
                        self.controller.publish(0, (180 - degree) / 5, 5)
                    else:
                        self.controller.publish(dst / 5, (degree + 180) / 5, 5)
                        self.controller.publish(0, (360 - 180 + degree) / 5, 5)
                else:
                    if o_y1 - o_y < 0:
                        self.controller.publish(dst / 5, (degree + 180) / 5, 5)
                        self.controller.publish(0, (360 - 180 + degree) / 5, 5)
                    else:
                        self.controller.publish(dst / 5, degree / 5, 5)
                        self.controller.publish(0, (360 - degree) / 5, 5)
            else:
                self.controller.publish(dst / 5, 0, 5)
            print ("MOVE")
            r.sleep()
            # self.controller.publish(0, math.atan(m), 10)


if __name__ == '__main__':
    rospy.init_node('a')
    move = Move('/map', '/odom', '/costmap', '/cmd_vel_mux/input/navi', (0, 5))
    # move.start()
