#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy

import json
from math import sqrt

class DataCollector:

    def __init__(self, odom_file, loc_file):
        rospy.init_node('data_collector', anonymous=True)

        self.odom_file = odom_file
        self.loc_file = loc_file

        self.distance_travelled = 0.0

        self.id = 1
        self.odom = Odometry()
        self.amcl_pose = PoseWithCovarianceStamped()

        # Subscribe to various topics
        rospy.Subscriber('RosAria/pose', Odometry, self.odom_callback)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('joy', Joy, self.get_odom_callback)

        rospy.spin()


    def odom_callback(self, data):
        self.distance_travelled += self.dist(self.odom, data)
        self.odom = data


    def amcl_pose_callback(self, data):
        self.amcl_pose = data


    def dist(self, odom1, odom2):
        x_dist = (abs(odom1.pose.pose.position.x) - abs(odom2.pose.pose.position.x)) ** 2
        y_dist = (abs(odom1.pose.pose.position.y) - abs(odom2.pose.pose.position.y)) ** 2
        z_dist = (abs(odom1.pose.pose.position.z) - abs(odom2.pose.pose.position.z)) ** 2

        return sqrt(x_dist + y_dist + z_dist)


    def get_odom_callback(self, data):
        if data.buttons[0] == 1:
            output = self.build_dict()
            self.write_to_files(output)
            print(str(self.id) + " " + str(self.distance_travelled))

    def write_to_files(self, output):
        position = output['odom']['position']
        print(str(position['x']) + ' ' + str(position['y']) + ' ' +
            str(output['odom']['orientation']['yaw']), file=self.odom_file)

        position = output['amcl']['position']
        print(str(position['x']) + ' ' + str(position['y']) + ' ' +
            str(output['amcl']['orientation']['yaw']), file=self.loc_file)


    def build_dict(self):
        output = {'id': self.id, 'odom': {}, 'amcl': {}}

        # Processing odometry information
        output['odom']['position'] = {
            'x': self.odom.pose.pose.position.x,
            'y': self.odom.pose.pose.position.y,
            'z': self.odom.pose.pose.position.z
        }

        odom_quarternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        )
        odom_roll, odom_pitch, odom_yaw = euler_from_quaternion(odom_quarternion)
        output['odom']['orientation'] = {
            'roll': odom_roll,
            'pitch': odom_pitch,
            'yaw': odom_yaw
        }

        # Processing amcl information
        output['amcl']['position'] = {
            'x': self.amcl_pose.pose.pose.position.x,
            'y': self.amcl_pose.pose.pose.position.y,
            'z': self.amcl_pose.pose.pose.position.z
        }
        amcl_quarternion = (
            self.amcl_pose.pose.pose.orientation.x,
            self.amcl_pose.pose.pose.orientation.y,
            self.amcl_pose.pose.pose.orientation.z,
            self.amcl_pose.pose.pose.orientation.w
        )
        amcl_roll, amcl_pitch, amcl_yaw = euler_from_quaternion(amcl_quarternion)
        output['amcl']['orientation'] = {
            'roll': amcl_roll,
            'pitch': amcl_pitch,
            'yaw': amcl_yaw
        }

        self.id += 1
        return output


if __name__ == '__main__':
    with open('odom1', 'w', 1) as odom, open('loc1', 'w', 1) as loc:
        node = DataCollector(odom, loc)
