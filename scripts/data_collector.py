#!/usr/bin/env python
from __future__ import print_function

import json
import rospy

from sensor_msgs.msg import Joy
from rosbridge_library.internal.message_conversion import extract_values

class DataCollector(object):

    def __init__(self):
        self.id = 0
        self.curr_data = None
        self.data_list = []

        self.id = int(rospy.get_param('~starting_id', '0'))
        self.joystick_topic = rospy.get_param('~joystick_topic', 'joy')
        self.out_file_name = rospy.get_param('~out_file', 'file.json')
        self.trigger_button = int(rospy.get_param('~trigger_button', '0'))

        rospy.Subscriber(self_joystick_topic, Joy, self._joystick_triggered)
        rospy.loginfo("Subscribed to %s topic", self.joystick_topic)


    def data_callback(self, data):
        self.curr_data = data


    def _joystick_triggered(self, joystick):
        if joystick.buttons[self.trigger_button] == 1:
            rospy.loginfo("Data collection triggered. Writing data to file...")
            self.id += 1
            self.data_list += [{'id': self.id, 'data': extract_values(self.curr_data)}]
            self._write_to_file()
            rospy.loginfo("Done writing data to file")


    def _write_to_file(self):
        with open(self.out_file_name, 'w') as f:
            json.dump(self.data_list, f)
            rospy.loginfo("Wrote to file %s", self.out_file_name)
