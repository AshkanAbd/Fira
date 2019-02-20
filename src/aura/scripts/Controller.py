#!/usr/bin/env python2.7

import rospy
import geometry_msgs.msg
import math


class Controller:
    cmd_publisher = None
    node_name = None
    rate = None

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        self.cmd_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', geometry_msgs.msg.Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def publish(self, meter, degree):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = meter
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = math.radians(degree)
        for i in xrange(10):
            self.cmd_publisher.publish(twist)
            self.rate.sleep()


if __name__ == '__main__':
    control = Controller('controller')
    control.publish( -0.5, 0)
    pass
