#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import Joy

module_name = None
message_type = None
trigger_button = None


def callback(data):
    pub = rospy.Publisher('joy', Joy)
    buttons = [0] * (trigger_button + 1)
    buttons[trigger_button] = 1

    joy_msg = Joy()
    joy_msg.buttons = buttons
    pub.publish(joy_msg)


def joy_relay():
    global module_name, message_type, trigger_button
    rospy.init_node('joy_relay', anonymous=True)

    module_name = rospy.get_param('~module_name', 'std_msgs.msg')
    message_type = rospy.get_param('~message_type', 'String')
    message_topic = rospy.get_param('~message_topic', 'joy')
    trigger_button = int(rospy.get_param('~trigger_button', '0'))

    message_class = getattr((__import__(module_name)).msg, message_type)
    rospy.Subscriber(message_topic, message_class, callback)

    rospy.spin()


if __name__ == '__main__':
    joy_relay()
