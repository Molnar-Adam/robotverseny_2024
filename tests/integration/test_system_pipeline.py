#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS rendszer teszt: teljes vezérlési és vizualizációs lánc.

Pipeline:
    /scan -> pid_error.py -> /error -> control.py -> /cmd_vel -> path_and_steering.cpp
    /odom -> path_and_steering.cpp -> /marker_path + /marker_text + /marker_steering

Ez a teszt egyszerre ellenőrzi a hibaszámítást, a vezérlési kimenetet és a
vizualizációs marker publikálást.
"""

import unittest

import rospy
import rostest
from control_msgs.msg import PidState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


MSG_TIMEOUT = 10.0


class TestSystemPipeline(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_system_pipeline', anonymous=True)

        self.received_error_msg = None
        self.received_cmd_vel_msg = None
        self.received_path_msg = None
        self.received_text_msg = None
        self.received_steering_msg = None

        self.error_subscriber = rospy.Subscriber('/error', PidState, self.error_callback)
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.path_subscriber = rospy.Subscriber('/marker_path', Path, self.path_callback)
        self.text_subscriber = rospy.Subscriber('/marker_text', Marker, self.text_callback)
        self.steering_subscriber = rospy.Subscriber('/marker_steering', Marker, self.steering_callback)

        self.scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)

        rospy.sleep(2.0)

    def error_callback(self, msg):
        self.received_error_msg = msg

    def cmd_vel_callback(self, msg):
        self.received_cmd_vel_msg = msg

    def path_callback(self, msg):
        self.received_path_msg = msg

    def text_callback(self, msg):
        self.received_text_msg = msg

    def steering_callback(self, msg):
        self.received_steering_msg = msg

    def wait_for_connections(self):
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.scan_publisher.get_num_connections() > 0 and self.odom_publisher.get_num_connections() > 0:
                return True
            rate.sleep()

        return False

    def create_mock_scan(self, front_distance=3.0, left_distance=2.0, right_distance=2.0):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'base_laser'
        scan.angle_min = 0.0
        scan.angle_max = 6.28319
        scan.angle_increment = 0.0174533
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 10.0
        scan.ranges = [5.0] * 360
        scan.ranges[60] = left_distance
        scan.ranges[300] = right_distance
        scan.ranges[359] = front_distance
        scan.intensities = []
        return scan

    def create_mock_odom(self, x=1.25, y=-0.4):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        return odom

    def publish_inputs(self, scan, odom):
        self.received_error_msg = None
        self.received_cmd_vel_msg = None
        self.received_path_msg = None
        self.received_text_msg = None
        self.received_steering_msg = None

        self.odom_publisher.publish(odom)
        self.scan_publisher.publish(scan)

    def wait_for_outputs(self, timeout=MSG_TIMEOUT):
        timeout_time = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout_time:
            if (
                self.received_error_msg is not None
                and self.received_cmd_vel_msg is not None
                and self.received_path_msg is not None
                and self.received_text_msg is not None
                and self.received_steering_msg is not None
            ):
                return True
            rate.sleep()

        return False

    def test_system_pipeline_end_to_end(self):
        self.assertTrue(self.wait_for_connections(), 'A /scan vagy /odom publisher nem csatlakozott időben a rendszerhez!')

        scan = self.create_mock_scan(front_distance=3.0, left_distance=1.0, right_distance=3.0)
        odom = self.create_mock_odom(x=1.25, y=-0.4)

        self.publish_inputs(scan, odom)

        self.assertTrue(self.wait_for_outputs(), 'A teljes rendszer nem adott vissza minden várt kimenetet időben!')

        self.assertLess(self.received_error_msg.error, 0.0, 'Bal oldal közelebb esetén az error negatív kell legyen!')
        self.assertLess(self.received_cmd_vel_msg.angular.z, 0.0, 'Negatív error esetén jobbra kell fordulni!')
        self.assertGreaterEqual(len(self.received_path_msg.poses), 1, 'A path markernek legalább egy pontot tartalmaznia kell!')
        self.assertEqual(self.received_path_msg.header.frame_id, 'odom_combined')
        self.assertEqual(self.received_text_msg.header.frame_id, 'base_link')
        self.assertEqual(self.received_steering_msg.header.frame_id, 'base_link')
        self.assertGreater(len(self.received_steering_msg.points), 0, 'A steering markernek pontokat kell tartalmaznia!')


if __name__ == '__main__':
    rostest.rosrun('megoldas_gyor24', 'test_system_pipeline', TestSystemPipeline)