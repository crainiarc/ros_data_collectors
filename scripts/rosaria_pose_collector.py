#!/usr/bin/env python
from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from data_collector import DataCollector

class RosAriaPoseCollector(DataCollector):

    def __init__(self):
        super(RosAriaPoseCollector, self).__init__()
        self.data_topic = rospy.get_param('~data_topic', 'RosAria/pose')

        rospy.Subscriber(self.data_topic, Odometry, self.data_callback)
        rospy.loginfo("Subscribed to %s topic", self.data_topic)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('rosaria_pose_collector', anonymous=True)
    node = RosAriaPoseCollector()
