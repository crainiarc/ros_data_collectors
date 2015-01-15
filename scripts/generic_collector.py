#!/usr/bin/env python
from __future__ import print_function

import rospy
from data_collector import DataCollector

class GenericCollector(DataCollector):

    def __init__(self):
        super(GenericCollector, self).__init__()

        self.module_name = rospy.get_param('~module_name', 'std_msgs.msg')
        self.data_type = rospy.get_param('~data_type', 'String')
        self.data_topic = rospy.get_param('~data_topic', 'clock')

        self.data_class = getattr((__import__(self.module_name)).msg, self.data_type)
        rospy.Subscriber(self.data_topic, self.data_class, self.data_callback)

        rospy.loginfo("Subscribed to %s topic", self.data_topic)
        rospy.spin()
        self._write_to_file()


if __name__ == "__main__":
    rospy.init_node('generic_collector', anonymous=True)
    node = GenericCollector()
