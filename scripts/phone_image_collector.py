#!/usr/bin/env python
from __future__ import print_function

import rospy
import socket
from data_collector import DataCollector

class PhoneImageCollector(DataCollector):

    def __init__(self):
        super(PhoneImageCollector, self).__init__()

        self.socket_port = rospy.get_param('~port', '50000')
        self.socket_host = rospy.get_param('~host', socket.gethostname())
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.bind((self.socket_host, int(self.socket_port)))
        self.server_sock.listen(1)
        rospy.loginfo("Start listening for connections on %s:%s", self.socket_host, self.socket_port)

        self.client_sock, address = self.server_sock.accept()
        rospy.loginfo("Client connected from  %s", address)

        rospy.spin()


    def _joystick_triggered(self, joystick):
        if joystick.buttons[self.trigger_button] == 1:
            self.id += 1

            self._request_data()
            data_buffer = self._receive_data()

            self.data_list += [{'id': self.id, 'data': extract_values(data)}]
            self._write_to_file()


    def _request_data(self):
      msg = 1
      total_sent = 0

      rospy.loginfo("Requesting image...")
      while total_sent < 1:
          sent = self.client_sock.send(msg)
          if sent == 0:
              raise RuntimeError("socket connection broken")
          total_sent = total_sent + sent


    def _receive_data(self):
      pass


if __name__ == "__main__":
    rospy.init_node('rosaria_pose_collector', anonymous=True)
    node = PhoneImageCollector()
