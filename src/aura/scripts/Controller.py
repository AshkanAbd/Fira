#!/usr/bin/env python2.7

import rospy
import geometry_msgs.msg
import math


class Controller:
    cmd_publisher = None
    rate = None

    def __init__(self, cmd_topic):
        self.cmd_publisher = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def publish(self, meter, degree, time):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = meter
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = math.radians(degree)
        for i in xrange(time * 10):
            self.cmd_publisher.publish(twist)
            self.rate.sleep()


if __name__ == '__main__':
<<<<<<< HEAD
    control = Controller('controller')
    control.publish(-0.5, 0)
=======
    control = Controller('/cmd_vel_mux/input/navi')
    control.publish(-0.5, 0, 1)
>>>>>>> f69d126eaa879bc74065d3c36d83c83ceb618e43
    pass
