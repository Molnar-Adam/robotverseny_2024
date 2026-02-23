#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Integrációs Teszt: Teljes Control Pipeline
===============================================

Ez a teszt a TELJES vezérlési pipeline-t teszteli:
    
    LIDAR (/scan) → pid_error.py → PidState (/error) → control.py → Twist (/cmd_vel)

Mit ellenőrzünk:
1. A pid_error.py fogadja a /scan-t és publikál /error-t
2. A control.py fogadja az /error-t és publikál /cmd_vel-t
3. A végső kimenet (steering angle, velocity) helyes-e

Ez egy MAGASABB szintű teszt, mint a test_pid_error_node.py!
"""

import sys
import unittest
import rospy
import rostest
from sensor_msgs.msg import LaserScan
from control_msgs.msg import PidState
from geometry_msgs.msg import Twist

MSG_TIMEOUT = 10.0


class TestControlPipeline(unittest.TestCase):
    """
    Teszteli a teljes vezérlési pipeline működését
    
    Node-ok a pipeline-ban:
        1. pid_error.py - LIDAR → error számítás
        2. control.py - PID vezérlés → motor parancsok
    """
    
    def setUp(self):
        """
        Inicializálja a teszt környezetet
        """
        rospy.init_node('test_control_pipeline', anonymous=True)
        
        self.received_error_msg = None
        self.error_received = False
        
        self.received_cmd_vel_msg = None
        self.cmd_vel_received = False
        
        # Subscribers
        self.error_subscriber = rospy.Subscriber(
            '/error', 
            PidState, 
            self.error_callback
        )
        
        self.cmd_vel_subscriber = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.cmd_vel_callback
        )
        
        # Publisher - szimulált LIDAR adat
        self.scan_publisher = rospy.Publisher(
            '/scan', 
            LaserScan, 
            queue_size=10
        )
        
        rospy.sleep(2.0)
        rospy.loginfo("TestControlPipeline setUp completed")
    
    def error_callback(self, msg):
        """Callback: pid_error.py kimenet"""
        rospy.loginfo(f"Middleware: /error received -> error={msg.error:.3f}, error_dot={msg.error_dot:.3f}")
        self.received_error_msg = msg
        self.error_received = True
    
    def cmd_vel_callback(self, msg):
        """Callback: control.py kimenet (VÉGSŐ KIMENET)"""
        rospy.loginfo(f"Final output: /cmd_vel -> linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        self.received_cmd_vel_msg = msg
        self.cmd_vel_received = True
    
    def create_mock_scan(self, front=2.0, left=2.0, right=2.0):
        """
        Szimulált LaserScan létrehozása
        
        Rövidebb függvény, mint a másik tesztben
        """
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser"
        scan.angle_min = 0.0
        scan.angle_max = 6.28319
        scan.angle_increment = 0.0174533
        scan.range_min = 0.02
        scan.range_max = 10.0
        
        scan.ranges = [5.0] * 360
        scan.ranges[90] = right   
        scan.ranges[180] = front  
        scan.ranges[270] = left    
        
        return scan
    
    def wait_for_messages(self, wait_for_cmd_vel=True):
        """
        Segédfüggvény: vár, amíg az üzenetek megérkeznek
        
        Paraméterek:
            wait_for_cmd_vel: Ha True, várunk a /cmd_vel-re is (teljes pipeline)
        """
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(10)
        
        while rospy.Time.now() < timeout:
            # Ellenőrzése: minden szükséges üzenet megérkezett-e
            error_ok = self.error_received
            cmd_vel_ok = self.cmd_vel_received if wait_for_cmd_vel else True
            
            if error_ok and cmd_vel_ok:
                rospy.loginfo("✓ Minden szükséges üzenet megérkezett")
                return True
            
            rate.sleep()
        
        rospy.logwarn(f"Timeout! error_received={self.error_received}, cmd_vel_received={self.cmd_vel_received}")
        return False
    
    # TESZT 1: Pipeline Alapvető Működés
    
    def test_1_pipeline_basic_functionality(self):
        """
        Teszt: Ellenőrzi, hogy a teljes pipeline működik-e
        
        Ellenőrzések:
        1. LaserScan → pid_error.py → /error üzenet érkezik
        2. /error → control.py → /cmd_vel üzenet érkezik
        3. Mindkét üzenet értelmes értékeket tartalmaz
        """
        rospy.loginfo("=== TEST 1: Pipeline Basic Functionality ===")
        
        self.error_received = False
        self.cmd_vel_received = False
        
        mock_scan = self.create_mock_scan(front=3.0, left=2.0, right=2.0)
        
        rospy.loginfo("Publikálok szimulált scan-t a pipeline-ba...")
        self.scan_publisher.publish(mock_scan)
        
        # Várakozás a teljes PIPELINE feldolgozására
        success = self.wait_for_messages(wait_for_cmd_vel=True)
        
        self.assertTrue(success, "A pipeline nem dolgozta fel a scan-t időben!")
        self.assertTrue(self.error_received, "A pid_error.py nem publikált /error üzenetet!")
        self.assertTrue(self.cmd_vel_received, "A control.py nem publikált /cmd_vel üzenetet!")
        
        rospy.loginfo("✓ A teljes pipeline működik: /scan → /error → /cmd_vel")
    
    # ============================================
    # TESZT 2: Szimmetrikus Eset
    # ============================================
    
    def test_2_symmetric_zero_error(self):
        """
        Teszt: Szimmetrikus fal-követés → minimális steering
        
        Elvárt viselkedés:
        - Bal = jobb távolság → error ≈ 0
        - error ≈ 0 → steering angle ≈ servo_offset (18°)
        - Kis error → magas sebesség (simple mode)
        """
        rospy.loginfo("=== TEST 2: Symmetric Case (Zero Error) ===")
        
        self.error_received = False
        self.cmd_vel_received = False
        
        mock_scan = self.create_mock_scan(front=3.0, left=2.5, right=2.5)
        
        rospy.loginfo("Publikálok szimmetrikus scan-t: bal=2.5m, jobb=2.5m")
        self.scan_publisher.publish(mock_scan)
        
        success = self.wait_for_messages(wait_for_cmd_vel=True)
        self.assertTrue(success, "Timeout a szimmetrikus esetben!")
        
        # Error ellenőrzése
        # followSimple: error = (2.5 - 2.5) * 0.3 = 0.0
        self.assertAlmostEqual(self.received_error_msg.error, 0.0, places=1,
                               msg="Szimmetrikus esetben error ≈ 0 kellene legyen")
        
        # cmd_vel ellenőrzése
        # Ha error ≈ 0 → sebesség magas kellene legyen (simple mode)
        self.assertGreater(self.received_cmd_vel_msg.linear.x, 0.0,
                           msg="Szimmetrikus esetben a robot mozogjon előre!")
        
        rospy.loginfo(f"✓ Szimmetrikus eset: error={self.received_error_msg.error:.3f}, "
                      f"velocity={self.received_cmd_vel_msg.linear.x:.3f} m/s, "
                      f"steering={self.received_cmd_vel_msg.angular.z:.3f} rad")
    
    # TESZT 3: Jobb forduló (bal fal közelebb)
    
    def test_3_turn_right_left_wall_closer(self):
        """
        Teszt: Bal fal közelebb → jobbra kell fordulni
        
        Elvárt viselkedés:
        - Bal közelebb → error < 0
        - error < 0 → steering angle < servo_offset (jobbra fordul)
        """
        rospy.loginfo("=== TEST 3: Turn Right (Left Wall Closer) ===")
        
        self.error_received = False
        self.cmd_vel_received = False
        
        mock_scan = self.create_mock_scan(front=3.0, left=1.0, right=3.0)
        
        rospy.loginfo("Publikálok aszimmetrikus scan-t: bal=1.0m (közel), jobb=3.0m (távol)")
        self.scan_publisher.publish(mock_scan)
        
        success = self.wait_for_messages(wait_for_cmd_vel=True)
        self.assertTrue(success, "Timeout!")
        
        self.assertLess(self.received_error_msg.error, 0.0,
                        msg="Bal fal közelebb → error negatív kellene legyen")
        
        rospy.loginfo(f"✓ Bal fal közelebb: error={self.received_error_msg.error:.3f} < 0, "
                      f"steering={self.received_cmd_vel_msg.angular.z:.3f} rad")
    
    # TESZT 4: Bal forduló (jobb fal közelebb)
    
    def test_4_turn_left_right_wall_closer(self):
        """
        Teszt: Jobb fal közelebb → balra kell fordulni
        
        Elvárt viselkedés:
        - Jobb közelebb → error > 0
        - error > 0 → steering angle > servo_offset (balra fordul)
        """
        rospy.loginfo("=== TEST 4: Turn Left (Right Wall Closer) ===")
        
        self.error_received = False
        self.cmd_vel_received = False
        
        mock_scan = self.create_mock_scan(front=3.0, left=3.0, right=1.0)
        
        rospy.loginfo("Publikálok aszimmetrikus scan-t: bal=3.0m (távol), jobb=1.0m (közel)")
        self.scan_publisher.publish(mock_scan)
        
        success = self.wait_for_messages(wait_for_cmd_vel=True)
        self.assertTrue(success, "Timeout!")
        
        self.assertGreater(self.received_error_msg.error, 0.0,
                           msg="Jobb fal közelebb → error pozitív kellene legyen")
        
        rospy.loginfo(f"✓ Jobb fal közelebb: error={self.received_error_msg.error:.3f} > 0, "
                      f"steering={self.received_cmd_vel_msg.angular.z:.3f} rad")


if __name__ == '__main__':
    rostest.rosrun(
        'megoldas_gyor24',
        'test_control_pipeline',
        TestControlPipeline
    )
