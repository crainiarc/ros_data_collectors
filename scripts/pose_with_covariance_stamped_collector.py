#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from data_collector import DataCollector

class PoseWithCovarianceStampedCollector(DataCollector):

    def __init__(self):
        super(PoseWithCovarianceStampedCollector, self).__init__()
        self.data_topic = rospy.get_param('~data_topic', 'pose')

        rospy.Subscriber(self.data_topic, PoseWithCovarianceStamped, self.data_callback)
        rospy.loginfo("Subscribed to %s topic", self.data_topic)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('pose_with_covariance_stamped_collector', anonymous=True)
    node = PoseWithCovarianceStampedCollector()
