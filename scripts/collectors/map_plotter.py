#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy

import json
from math import sqrt
from collections import deque

class DataCollector:
    def __init__(self, loc_file):
        rospy.init_node('map_plotter', anonymous=True)

        self.loc_file = loc_file
        path = self.construct_path()

        pub = rospy.Publisher('trajectory', Path)
        while not rospy.is_shutdown():
            pub.publish(path)
            rospy.sleep(1.0)

        # self.distance_travelled = 0.0

        # self.id = 1
        # self.odom = Odometry()
        # self.wb_msgs = deque([{}, {}, {}]) # will get the last 3 readings
        # self.amcl_pose = PoseWithCovarianceStamped()

        # # Subscribe to various topics
        # rospy.Subscriber('wb_publisher', String, self.wb_callback)
        # rospy.Subscriber('RosAria/pose', Odometry, self.odom_callback)
        # rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        # rospy.Subscriber('joy', Joy, self.get_odom_callback)

        # rospy.spin()

    def construct_path(self):
        path = Path()
        path.header.frame_id = "map"
        seq = 0
        for line in self.loc_file.readlines():
            line_list = line.split()
            pose = PoseStamped()

            pose.header.seq = seq
            pose.header.frame_id = "map"

            pose.pose.position.x = float(line_list[0])
            pose.pose.position.y = float(line_list[1])

            x, y, z, w = quaternion_from_euler(0, 0, float(line_list[2]))
            pose.pose.orientation.x = x
            pose.pose.orientation.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w

            path.poses.append(pose)
            seq += 1

        return path

if __name__ == '__main__':
    with  open('loc5', 'r', 1) as loc:
        node = DataCollector(loc)
