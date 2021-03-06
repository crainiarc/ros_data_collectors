#!/usr/bin/env python
from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from data_collector import DataCollector

class OdometryCollector(DataCollector):

    def __init__(self):
        super(OdometryCollector, self).__init__()
        self.data_topic = rospy.get_param('~data_topic', 'odom')

        rospy.Subscriber(self.data_topic, Odometry, self.data_callback)
        rospy.loginfo("Subscribed to %s topic", self.data_topic)
        rospy.spin()
        self._write_to_file()


if __name__ == "__main__":
    rospy.init_node('odometry_collector', anonymous=True)
    node = OdometryCollector()
