#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Integrációs Teszt: control.py node (izolált)
===============================================

Ez a teszt ellenőrzi, hogy a control.py node:
1. Sikeresen elindul
2. Feliratkozik az /error topicra (PidState bemenet)
3. Publikál a /cmd_vel topicra helyes Twist kimenettel

Pipeline:
    TESZT -> /error (PidState) -> control.py -> /cmd_vel (Twist) -> TESZT

Ez egy célzott integrációs teszt: a control node-ot izoláltan vizsgálja,
a pid_error node nélkül.
"""

import unittest

import rospy
import rostest
from control_msgs.msg import PidState
from geometry_msgs.msg import Twist


MSG_TIMEOUT = 10.0


class TestControlNode(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_control_node', anonymous=True)

        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.error_publisher = rospy.Publisher('/error', PidState, queue_size=10)

        rospy.sleep(1.0)
        rospy.loginfo("TestControlNode setUp completed")

    def cmd_vel_callback(self, msg):
        self.received_cmd_vel_msg = msg
        self.cmd_vel_received = True

    def publish_error(self, error_value, velocity_value, frame_id='simple'):
        msg = PidState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.error = error_value
        msg.error_dot = velocity_value
        self.error_publisher.publish(msg)

    def wait_for_cmd_vel(self):
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.cmd_vel_received:
                return True
            rate.sleep()

        return False

    def test_1_node_initialization(self):
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]
        self.assertIn('/cmd_vel', topic_names, "A control.py node nem publikál a /cmd_vel topicra!")

    def test_2_zero_error_keeps_zero_steering(self):
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.publish_error(error_value=0.0, velocity_value=1.5, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        self.assertAlmostEqual(
            self.received_cmd_vel_msg.angular.z,
            0.0,
            places=3,
            msg="Nulla error esetén a kormányparancsnak 0-nak kell lennie!",
        )
        self.assertGreater(
            self.received_cmd_vel_msg.linear.x,
            0.0,
            msg="Nulla error mellett is legyen pozitív előremeneti sebesség!",
        )

    def test_3_negative_error_turns_right(self):
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.publish_error(error_value=-0.6, velocity_value=1.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        self.assertLess(
            self.received_cmd_vel_msg.angular.z,
            0.0,
            msg="Negatív error esetén jobbra fordulás kell (angular.z < 0)!",
        )

    def test_4_positive_error_turns_left(self):
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.publish_error(error_value=0.3, velocity_value=1.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        self.assertGreater(
            self.received_cmd_vel_msg.angular.z,
            0.0,
            msg="Pozitív error esetén balra fordulás kell (angular.z > 0)!",
        )

    # ============================================
    # LIMIT TESZTEK
    # ============================================

    def test_5_limit_velocity_simple_mode_max(self):
        """
        Teszt: Simple mode-ban nagy sebesség bemenet után max 4.0-ra clamp
        
        control.py-ban: if velocity > 4.0: velocity = 4.0
        """
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        # Nagyon nagy error, nagy sebesség
        self.publish_error(error_value=0.5, velocity_value=10.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        self.assertLessEqual(
            self.received_cmd_vel_msg.linear.x,
            4.0 * 0.2,  # control.py-ban: msg_cmd.linear.x = velocity * 0.2
            msg=f"Simple mode-ban a sebesség max 4.0-ra clamp! (kapott: {self.received_cmd_vel_msg.linear.x})",
        )

    def test_6_limit_velocity_simple_mode_min(self):
        """
        Teszt: Simple mode-ban a sebesség nem lehet negatív
        
        control.py-ban: if velocity < 0: velocity = 0.0
        """
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        # Negatív bemenet
        self.publish_error(error_value=0.0, velocity_value=-2.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        self.assertGreaterEqual(
            self.received_cmd_vel_msg.linear.x,
            0.0,
            msg=f"Simple mode-ban a sebesség nem lehet negatív! (kapott: {self.received_cmd_vel_msg.linear.x})",
        )

    def test_7_limit_large_positive_error(self):
        """
        Teszt: Nagyon nagy pozitív error → jobbra fordulás, de nem unbounded
        
        control.py-ban: control_error = kp*error + kd*(error - prev_error)
        Ez szöget hoz: angle = servo_offset + control_error*π/180
        
        A +steering értéknek nagyobb logikus limit nélkül, de ellenőrizzük, hogy nem nan/inf
        """
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        # Nagy pozitív error
        self.publish_error(error_value=100.0, velocity_value=1.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        
        #Kimenet érvényességének ellenőrzése
        import math
        self.assertTrue(
            math.isfinite(self.received_cmd_vel_msg.angular.z),
            msg="Nagy pozitív error után a steering nem lehet nan/inf!",
        )
        
        # Simple mode-ban nagy error esetén jobbra fordul (angular.z < 0 vagy nagy szög)
        rospy.loginfo(f"Nagy +error után steering: {self.received_cmd_vel_msg.angular.z}")

    def test_8_limit_large_negative_error(self):
        """
        Teszt: Nagyon nagy negatív error → balra fordulás, de nem unbounded
        
        Hasonlóan az előzőhöz, de negatív irányban
        """
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        # Nagyon nagy negatív error
        self.publish_error(error_value=-100.0, velocity_value=1.0, frame_id='simple')
        success = self.wait_for_cmd_vel()

        self.assertTrue(success, "Nem érkezett /cmd_vel üzenet időben!")
        
        #Kimenet érvényességének ellenőrzése
        import math
        self.assertTrue(
            math.isfinite(self.received_cmd_vel_msg.angular.z),
            msg="Nagy negatív error után a steering nem lehet nan/inf!",
        )
        
        rospy.loginfo(f"Nagy -error után steering: {self.received_cmd_vel_msg.angular.z}")


if __name__ == '__main__':
    rostest.rosrun(
        'megoldas_gyor24',
        'test_control_node',
        TestControlNode,
    )
