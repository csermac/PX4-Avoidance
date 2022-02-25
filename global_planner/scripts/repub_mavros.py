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

        # initialize boolean variable that registers is the sought topic has
        # been found
        found = False
        # scan for any of the two topics published by opevnslam
        while not found:
            topics = rospy.get_published_topics()
            for topic, taip in topics:
                if topic == '/run_localization/camera_pose' or topic == '/run_slam/camera_pose':
                    self.sub = rospy.Subscriber(topic, Odometry, self.repub, queue_size=1)
                    rospy.loginfo(f"found topic \"{topic}\" to translate")
                    found = True
                    break
            if not found:
                rospy.loginfo("topics not found, yet")
            rospy.Rate(0.1).sleep()

        # self.sub = rospy.Subscriber('/run_localization/camera_pose', Odometry, self.repub, queue_size=1)
        rospy.loginfo(f"subscriber to {topic} created")

    def repub(self, from_vslam):
        """
        The callback function that realies on openvslam for the publication rate,
        whichm might not be enough
        """

        output = PoseStamped()

        output.header = from_vslam.header

        # conversion from https://docs.px4.io/master/en/ros/external_position_estimation.html#reference-frames
        output.pose.position.x = from_vslam.pose.pose.position.x
        output.pose.position.y = from_vslam.pose.pose.position.y
        output.pose.position.z = from_vslam.pose.pose.position.z

        output.pose.orientation.x = from_vslam.pose.pose.orientation.x
        output.pose.orientation.y = from_vslam.pose.pose.orientation.y
        output.pose.orientation.z = from_vslam.pose.pose.orientation.z
        output.pose.orientation.w = from_vslam.pose.pose.orientation.w

        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node('translate_odom_to_pose')
    respublica = RepubMav()
    rospy.spin()
