#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS integration test for path_and_steering.cpp.

Pipeline:
    TEST -> /odom + /cmd_vel -> path_and_steering -> /marker_path + /marker_text + /marker_steering -> TEST
"""

import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


MSG_TIMEOUT = 10.0


class TestPathAndSteeringNode(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_path_and_steering_node', anonymous=True)

        self.received_path_msg = None
        self.received_text_msg = None
        self.received_steering_msg = None

        self.path_subscriber = rospy.Subscriber('/marker_path', Path, self.path_callback)
        self.text_subscriber = rospy.Subscriber('/marker_text', Marker, self.text_callback)
        self.steering_subscriber = rospy.Subscriber('/marker_steering', Marker, self.steering_callback)

        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.sleep(1.0)

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
            if self.odom_publisher.get_num_connections() > 0 and self.cmd_vel_publisher.get_num_connections() > 0:
                return True
            rate.sleep()

        return False

    def make_odom(self, x, y):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        return odom

    def make_twist(self, speed, steering):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering
        return twist

    def publish_inputs(self, x, y, speed, steering):
        self.received_path_msg = None
        self.received_text_msg = None
        self.received_steering_msg = None

        self.odom_publisher.publish(self.make_odom(x, y))
        self.cmd_vel_publisher.publish(self.make_twist(speed, steering))

    def wait_for_expected_outputs(self, x, y, speed, timeout=MSG_TIMEOUT):
        expected_text = '{:.2f}m/s'.format(speed)
        timeout_time = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout_time:
            if self.received_path_msg is None or self.received_text_msg is None or self.received_steering_msg is None:
                rate.sleep()
                continue

            if not self.received_path_msg.poses:
                rate.sleep()
                continue

            last_pose = self.received_path_msg.poses[-1].pose
            path_matches = (
                abs(last_pose.position.x - x) < 1e-3
                and abs(last_pose.position.y - y) < 1e-3
                and self.received_path_msg.header.frame_id == 'odom_combined'
            )
            text_matches = self.received_text_msg.text == expected_text
            steering_matches = self.received_steering_msg.header.frame_id == 'base_link' and len(self.received_steering_msg.points) > 0

            if path_matches and text_matches and steering_matches:
                return True

            rate.sleep()

        return False

    def test_node_publishes_path_text_and_steering_markers(self):
        self.assertTrue(self.wait_for_connections(), 'A teszt publisher-ek nem csatlakoztak a node-hoz idoben!')

        x = 1.25
        y = -0.4
        speed = 0.8
        steering = 0.25

        self.publish_inputs(x, y, speed, steering)

        self.assertTrue(
            self.wait_for_expected_outputs(x, y, speed),
            'Nem erkezett elvart marker/path kimenet a path_and_steering node-tol!',
        )

        self.assertGreaterEqual(len(self.received_path_msg.poses), 1)
        self.assertLessEqual(len(self.received_path_msg.poses), 8)

    def test_path_is_trimmed_to_configured_size(self):
        self.assertTrue(self.wait_for_connections(), 'A teszt publisher-ek nem csatlakoztak a node-hoz idoben!')

        for index in range(6):
            self.publish_inputs(0.1 * index, 0.0, 0.6, 0.1)
            rospy.sleep(0.1)

        self.assertTrue(
            self.wait_for_expected_outputs(0.5, 0.0, 0.6),
            'A path_and_steering node nem frissitette idoben a path kimenetet!',
        )
        self.assertLessEqual(
            len(self.received_path_msg.poses),
            4,
            'A path hossza nem lett a megadott path_size-re korlatozva!',
        )


if __name__ == '__main__':
    rostest.rosrun(
        'megoldas_gyor24',
        'test_path_and_steering_node',
        TestPathAndSteeringNode,
    )