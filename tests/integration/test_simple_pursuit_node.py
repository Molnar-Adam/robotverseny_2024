#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Integracios Teszt: simple_pursuit.py node (izolalt)
=======================================================

Pipeline:
    TESZT -> /scan (LaserScan) -> simple_pursuit.py -> /cmd_vel (Twist) -> TESZT
"""

import math
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


MSG_TIMEOUT = 10.0


class TestSimplePursuitNode(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_simple_pursuit_node', anonymous=True)

        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

        rospy.sleep(1.0)

    def cmd_vel_callback(self, msg):
        self.received_cmd_vel_msg = msg
        self.cmd_vel_received = True

    def wait_for_connections(self):
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.scan_publisher.get_num_connections() > 0:
                return True
            rate.sleep()

        return False

    def wait_for_cmd_vel(self):
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.cmd_vel_received:
                return True
            rate.sleep()

        return False

    def publish_scan_and_wait(self, scan, retries=3):
        for _ in range(retries):
            self.cmd_vel_received = False
            self.received_cmd_vel_msg = None
            self.scan_publisher.publish(scan)
            if self.wait_for_cmd_vel():
                return True
        return False

    def create_mock_scan(self, default_distance=2.0):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 359.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 10.0

        scan.ranges = [default_distance] * 360
        scan.intensities = []
        return scan

    def set_range_at_degree(self, scan, deg, value):
        rad = math.radians(deg)
        idx = int((rad - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(len(scan.ranges) - 1, idx))
        scan.ranges[idx] = value

    def test_1_node_initialization(self):
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]

        self.assertIn('/cmd_vel', topic_names, 'A simple_pursuit.py node nem publikal a /cmd_vel topicra!')
        self.assertTrue(self.wait_for_connections(), 'A /scan publisher nem csatlakozott idoben a node-hoz!')

    def test_2_publish_scan_produces_cmd_vel(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        scan = self.create_mock_scan(default_distance=2.0)
        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett /cmd_vel uzenet scan publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.linear.x), 'A linear.x nem lehet nan/inf!')
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.angular.z), 'Az angular.z nem lehet nan/inf!')

    def test_3_reverse_zone_close_obstacle_slows_down_significantly(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        baseline_scan = self.create_mock_scan(default_distance=2.0)
        baseline_ok = self.publish_scan_and_wait(baseline_scan, retries=3)
        self.assertTrue(baseline_ok, 'Nem erkezett baseline /cmd_vel uzenet!')
        baseline_speed = self.received_cmd_vel_msg.linear.x

        scan = self.create_mock_scan(default_distance=2.0)
        for deg in (170, 175, -170, -175):
            self.set_range_at_degree(scan, deg, 0.1)

        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett /cmd_vel uzenet reverse-zone tesztnel!')
        reduced_speed = self.received_cmd_vel_msg.linear.x

        self.assertLess(
            reduced_speed,
            baseline_speed,
            msg='Kozeli hatso akadaly eseten a sebessegnek csokkennie kell!',
        )
        self.assertLess(
            reduced_speed,
            0.3,
            msg='Kozeli hatso akadaly eseten alacsony sebesseg varhato (<0.3 m/s)!',
        )

    def test_4_repeated_scans_keep_node_responsive(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        first_scan = self.create_mock_scan(default_distance=2.0)
        second_scan = self.create_mock_scan(default_distance=1.2)

        success_first = self.publish_scan_and_wait(first_scan, retries=2)
        self.assertTrue(success_first, 'Az elso scan utan nem erkezett /cmd_vel!')
        first_cmd = self.received_cmd_vel_msg.linear.x

        success_second = self.publish_scan_and_wait(second_scan, retries=2)
        self.assertTrue(success_second, 'A masodik scan utan nem erkezett /cmd_vel!')
        second_cmd = self.received_cmd_vel_msg.linear.x

        self.assertTrue(math.isfinite(first_cmd))
        self.assertTrue(math.isfinite(second_cmd))


if __name__ == '__main__':
    rostest.rosrun(
        'megoldas_gyor24',
        'test_simple_pursuit_node',
        TestSimplePursuitNode,
    )
