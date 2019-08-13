from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy
import cv_bridge
import robot_cli


class Transform:
    robot_cli = robot_cli.robot_cli
    cmd_subscriber = rospy.Subscriber
    image_publisher = rospy.Publisher
    bridge = cv_bridge.CvBridge

    def __init__(self):
        self.robot_cli = robot_cli.robot_cli("127.0.0.1")
        self.cmd_subscriber = rospy.Subscriber('/aura/cmd_vel', Twist, self.cmd_handler, queue_size=1000)
        self.image_publisher = rospy.Publisher('/aura/image', Image, queue_size=1000)
        self.bridge = cv_bridge.CvBridge()
        self.image_handler()

    def cmd_handler(self, twist):
        self.robot_cli.publish_twist(twist)

    def image_handler(self):
        rate = rospy.Rate(10)
        while True:
            image = self.robot_cli.get_image()
            image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.image_publisher.publish(image)
            rate.sleep()


if __name__ == '__main__':
    t = Transform()
