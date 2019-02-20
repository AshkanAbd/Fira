#!/usr/bin/env python2

import rospy
import sensor_msgs.msg
import sensor_msgs.point_cloud2


def callback(image):
    print(next(sensor_msgs.point_cloud2.read_points(image, skip_nans=True, uvs=[[205, 209]])))


cloud = None

if __name__ == '__main__':
    rospy.init_node('a')
    rospy.Subscriber('/camera/depth/points', sensor_msgs.msg.PointCloud2, callback, queue_size=1000)
    rospy.spin()
