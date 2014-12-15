#!/usr/bin/env python
from __future__ import print_function

import rospy
import socket
from data_collector import DataCollector

class PhoneImageCollector(DataCollector):

    def __init__(self):
        super(RosAriaPoseCollector, self).__init__()

        self.socket_port = rospy.get_param('~port', '50000')
        self.socket_host = rospy.get_param('~host', '127.0.0.1')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM))

        rospy.spin()


    def _connect_to_phone(self):
        self.sock.connect((self.socket_host, self.socket_port))


if __name__ == "__main__":
    rospy.init_node('rosaria_pose_collector', anonymous=True)
    node = PhoneImageCollector()
