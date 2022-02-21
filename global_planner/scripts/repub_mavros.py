#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RepubMav():
    """
    get the message from openvslam and publish it to /mavros/mocap/pose after
    translation
    """

    def __init__(self):
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        # self.pub_cam = rospy.Publisher('/camera/pose', PoseStamped, queue_size=1)

        self.output = PoseStamped()

        # initialize boolean variable that registers is the sought topic has
        # been found
        found = False
        # scan for any of the two topics published by opevnslam
        while not found:
            topics = rospy.get_published_topics()
            for topic, taip in topics:
                if topic == '/run_localization/camera_pose' or topic == '/run_slam/camera_pose':
                    self.sub = rospy.Subscriber(topic, Odometry, self.repub, queue_size=1)
                    rospy.loginfo("found topic %s to translate", topic)
                    found = True
                    break
                else:
                    rospy.loginfo("topics not found, yet")
                    rospy.sleep(0.5)

        self.sub = rospy.Subscriber('/run_localization/camera_pose', Odometry, self.repub, queue_size=1)
        rospy.loginfo("subscriber to found topic created")

        # local rate variable to be used only here with this value
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.output.header.stamp = rospy.Time.now()
            self.pub.publish(self.output)
            # replace pubishing camera_pose from openvslam and do it here
            # self.pub_cam.publish(self.output)
            rate.sleep()

    def repub(self, from_vslam):
        """
        this callback function just translates the message, so the publisher in the constructor
        can publish it a given rate, independent from the original message, while it keeps
        being updted
        """

        self.output.header = from_vslam.header

        self.output.pose.position.x = from_vslam.pose.pose.position.x
        self.output.pose.position.y = from_vslam.pose.pose.position.z
        self.output.pose.position.z = from_vslam.pose.pose.position.y

        self.output.pose.orientation.x = -from_vslam.pose.pose.orientation.x
        self.output.pose.orientation.y = -from_vslam.pose.pose.orientation.z
        self.output.pose.orientation.z = -from_vslam.pose.pose.orientation.y
        self.output.pose.orientation.w = -from_vslam.pose.pose.orientation.w


if __name__ == "__main__":
    rospy.init_node('translate_odom_to_pose')
    respublica = RepubMav()
    rospy.spin()
