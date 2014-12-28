#!/usr/bin/env python
from __future__ import print_function

import json
import rospy
import socket
from data_collector import DataCollector

class PhoneImageCollector(DataCollector):

    def __init__(self):
        super(PhoneImageCollector, self).__init__()

        self.image_file_path = rospy.get_param('~image_file_path', '~')
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
        if joystick.buttons[self.trigger_button] == 1:
            self.id += 1
            self._request_data()
            data, image = self._receive_data()

            self.data_list += [{'id': self.id, 'data': data}]
            self._write_to_file()

            with open(self.image_file_path + '/image_' + str(self.id) + '.jpg', 'wb') as f:
                f.write(image.decode('base64'))


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
        rospy.loginfo("Receiving data")

        # Read data other than images
        total_bytes, bytes_recd = self._read_int()
        sensor_data, bytes_recd = self._read_bytes(total_bytes)

        sensor_data = json.loads(sensor_data)
        image_data = sensor_data["Image Data"]
        del sensor_data["Image Data"]

        rospy.loginfo("Successfully received data")
        return sensor_data, image_data


    def _read_int(self):
        chunk = None
        chunks = []
        bytes_recd = 0

        while chunk != 0:
            chunk = self.client_sock.recv(1)
            if chunk == '':
                raise RuntimeError("Socket connection broken")
            elif chunk == 0 or chunk < '0' or chunk > '9':
                break
            chunks.append(chunk)
            bytes_recd += len(chunk)

        return int(b''.join(chunks)), bytes_recd


    def _read_bytes(self, num_bytes):
        chunk = None
        chunks = []
        bytes_recd = 0

        while bytes_recd < num_bytes:
            chunk = self.client_sock.recv(min(num_bytes, 2048))
            if chunk == '':
                raise RuntimeError("Socket connection broken")
            elif chunk == 0:
                break
            chunks.append(chunk)
            bytes_recd += len(chunk)

        return b''.join(chunks), bytes_recd


if __name__ == "__main__":
    rospy.init_node('phone_image_collector', anonymous=True)
    node = PhoneImageCollector()
