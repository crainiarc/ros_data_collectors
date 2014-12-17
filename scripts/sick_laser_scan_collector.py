#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from data_collector import DataCollector

class SickLaserScanCollector(DataCollector):

    def __init__(self):
        super(SickLaserScanCollector, self).__init__()
        self.data_topic = rospy.get_param('~data_topic', 'scan')

        rospy.Subscriber(self.data_topic, LaserScan, self.data_callback)
        rospy.loginfo("Subscribed to %s topic", self.data_topic)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('sick_laser_scan_collector', anonymous=True)
    node = SickLaserScanCollector()
