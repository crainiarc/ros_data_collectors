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
        self.client_sock.close()
        self.server_sock.close()


    def _joystick_triggered(self, joystick):
        rospy.loginfo('joystick triggerred')
        self._request_data()
        data_buffer = self._receive_data()
        print(data_buffer)
        # if joystick.buttons[self.trigger_button] == 1:
        #     self.id += 1

        #     self._request_data()
        #     data_buffer = self._receive_data()

        #     self.data_list += [{'id': self.id, 'data': extract_values(data)}]
        #     self._write_to_file()


    def _request_data(self):
      msg = '1\n'
      total_sent = 0

      rospy.loginfo("Requesting image...")
      while total_sent < 1:
          sent = self.client_sock.send(msg)
          if sent == 0:
              raise RuntimeError("Socket connection broken")
          total_sent = total_sent + sent


    def _receive_data(self):
        chunk = None
        chunks = []
        bytes_recd = 0

        rospy.loginfo("Receiving data")
        while chunk != 0:
            chunk = self.client_sock.recv(1)
            if chunk == '':
                raise RuntimeError("Socket connection broken")
            elif chunk == 0:
                break
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
            print(''.join(chunks))
        rospy.loginfo("Successfully received data")
        return ''.join(chunks)


if __name__ == "__main__":
    rospy.init_node('rosaria_pose_collector', anonymous=True)
    node = PhoneImageCollector()
