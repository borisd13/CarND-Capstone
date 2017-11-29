#!/usr/bin/env python

import cv2
import rospy
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObstacleDetector(object):
    def __init__(self):
        rospy.init_node('obstacle_detector')

        self.waypoints = None
        self.camera_image = None
        self.bridge = CvBridge()

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        self.pub = rospy.Publisher('/obstacle_waypoint', Lane, queue_size=1)

        rospy.spin()

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # rospy.loginfo("{}".format(type(waypoints.waypoints)))

    def image_cb(self, image):
        self.camera_image = image
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # cv2.imwrite("sample.jpg", cv_image)
        # message = "height:{}, width:{}, cv_image type:{}".format(image.height, image.width, type(cv_image))
        # rospy.loginfo(message)

    def obstacle_cb(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/wp_updater'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.get_waypoint_velocity(waypoints)
        self.pub.publish(lane)


if __name__ == '__main__':
    try:
        ObstacleDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start obstacle detector node.')
       
