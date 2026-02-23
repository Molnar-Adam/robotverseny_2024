#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Integrációs Teszt: pid_error.py node
========================================

Ez a teszt ellenőrzi, hogy a pid_error.py node:
1. Sikeresen elindul
2. Feliratkozik a /scan topicra
3. Publikál az /error topicra helyes PidState üzenettel

A teszt VALÓS ROS node-okat indít (nem mock-okat használ)!
"""

import sys
import unittest
import rospy
import rostest
from sensor_msgs.msg import LaserScan
from control_msgs.msg import PidState


MSG_TIMEOUT = 10.0 


class TestPidErrorNode(unittest.TestCase):
    """
    Teszteli a pid_error.py node integrációját
    """
    
    def setUp(self):
        """
        Ez minden teszt ELŐTT lefut
        
        Mit csinál:
        - Inicializálja a ROS node-ot (test_pid_error_node néven)
        - Beállítja az üzenet tárolókat
        """
        # ROS node inicializáció teszteléshez
        rospy.init_node('test_pid_error_node', anonymous=True)
        
        # Üzenet tárolók
        self.received_error_msg = None
        self.error_msg_received = False
        
        # Feliratkozás a /error topicra
        self.error_subscriber = rospy.Subscriber(
            '/error', 
            PidState, 
            self.error_callback
        )
        
        # Publisher a /scan topicra, hogy szimulált LIDAR adat küldésére
        self.scan_publisher = rospy.Publisher(
            '/scan', 
            LaserScan, 
            queue_size=10
        )
        
        rospy.sleep(1.0)
        
        rospy.loginfo("TestPidErrorNode setUp completed")
    
    def error_callback(self, msg):
        """
        Callback függvény: amikor a pid_error.py publikál az /error topicra
        
        Ez a függvény AUTOMATIKUSAN meghívódik, amikor üzenet érkezik
        """
        rospy.loginfo(f"Received error message: error={msg.error}, error_dot={msg.error_dot}")
        self.received_error_msg = msg
        self.error_msg_received = True
    
    def create_mock_scan(self, front_distance=2.0, left_distance=2.0, right_distance=2.0):
        """
        Szimulált LaserScan üzenet létrehozása
        
        Paraméterek:
            front_distance: Távolság előre (90°), méterben
            left_distance: Távolság balra (180°), méterben  
            right_distance: Távolság jobbra (0°), méterben
        """
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser"
        
        scan.angle_min = 0.0
        scan.angle_max = 6.28319  # ~2π radiánban
        scan.angle_increment = 0.0174533  # ~1° radiánban
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 10.0
        
        scan.ranges = [5.0] * 360
        
        # 0° = jobbra, 90° = előre, 180° = balra
        scan.ranges[90] = right_distance   # 0° -> index 90
        scan.ranges[180] = front_distance  # 90° -> index 180
        scan.ranges[270] = left_distance   # 180° -> index 270
        
        scan.intensities = []
        
        return scan
    
    # TESZT 1: Node Inicializáció
    
    def test_1_node_initialization(self):
        """
        Teszt: Ellenőrzi, hogy a pid_error.py node sikeresen elindult-e
        
        Hogyan működik:
        - A rostest automatikusan elindítja a node-okat a .test fájl alapján
        - Topic létezésének ellenőrzése
        """
        rospy.loginfo("=== TEST 1: Node Initialization ===")
        
        # Aktív topicok lekérése
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]
        
        # /error topic ellenőrzése
        # Ha a pid_error.py fut, akkor publikál erre a topicra
        self.assertIn('/error', topic_names, 
                      "A pid_error.py node nem publikál az /error topicra!")
        
        rospy.loginfo("✓ A pid_error.py node sikeresen elindult és publikál az /error topicra")
    
    # TESZT 2: Szimmetrikus Fallkövetés
    
    def test_2_symmetric_wall_following(self):
        """
        Teszt: Szimmetrikus fal-követés (bal = jobb távolság)
        
        Mit várunk:
        - Ha bal és jobb oldal távolsága egyenlő → error ≈ 0
        - A followSimple() függvény: error = (left - right) * 0.3
        - Ha left = right = 2.0m → error = (2.0 - 2.0) * 0.3 = 0.0
        """
        rospy.loginfo("=== TEST 2: Symmetric Wall Following ===")
        
        self.received_error_msg = None
        self.error_msg_received = False
        
        mock_scan = self.create_mock_scan(
            front_distance=3.0,
            left_distance=2.0,  
            right_distance=2.0  
        )
        
        # Szimulált scan publikálása
        rospy.loginfo("Publikálok szimulált LaserScan-t: bal=2.0m, jobb=2.0m")
        self.scan_publisher.publish(mock_scan)
        
        # Várakozás, amíg a pid_error.py feldolgozza és publikáljon /error-t
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(10) 
        
        while not self.error_msg_received and rospy.Time.now() < timeout:
            rate.sleep()
        
        self.assertTrue(self.error_msg_received, 
                        "Nem érkezett PidState üzenet az /error topicra!")
        
        # Ellenőrzés: error = ~0
        # followSimple: error = (left - right) * 0.3 = (2.0 - 2.0) * 0.3 = 0.0
        self.assertIsNotNone(self.received_error_msg)
        self.assertAlmostEqual(self.received_error_msg.error, 0.0, places=2,
                               msg=f"Szimmetrikus esetben az error ~0 kellene legyen, de {self.received_error_msg.error}")
        
        rospy.loginfo(f"✓ Szimmetrikus eset: error = {self.received_error_msg.error} ≈ 0.0")
    
    # TESZT 3: Aszimmetrikus Fallkövetés (bal közelebb)
    
    def test_3_left_closer_wall_following(self):
        """
        Teszt: Bal oldal közelebb van → negatív error
        
        Mit várunk:
        - Ha bal oldal közelebb (1.0m) és jobb távol (3.0m) → error < 0
        - followSimple: error = (left - right) * 0.3 = (1.0 - 3.0) * 0.3 = -0.6
        """
        rospy.loginfo("=== TEST 3: Left Closer Wall Following ===")
        
        self.received_error_msg = None
        self.error_msg_received = False
        
        mock_scan = self.create_mock_scan(
            front_distance=3.0,
            left_distance=1.0,   
            right_distance=3.0   
        )
        
        rospy.loginfo("Publikálok szimulált LaserScan-t: bal=1.0m, jobb=3.0m")
        self.scan_publisher.publish(mock_scan)
        
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(10)
        
        while not self.error_msg_received and rospy.Time.now() < timeout:
            rate.sleep()
        
        self.assertTrue(self.error_msg_received, 
                        "Nem érkezett PidState üzenet!")
        
        # Ellenőrzés: error = (1.0 - 3.0) * 0.3 = -0.6
        expected_error = (1.0 - 3.0) * 0.3  # = -0.6
        self.assertAlmostEqual(self.received_error_msg.error, expected_error, places=1,
                               msg=f"Bal közelebb esetén error={expected_error} kellene legyen")
        
        rospy.loginfo(f"✓ Bal közelebb: error = {self.received_error_msg.error} ≈ {expected_error}")
    
    # TESZT 4: Jobb oldal közelebb
    
    def test_4_right_closer_wall_following(self):
        """
        Teszt: Jobb oldal közelebb van → pozitív error
        
        Mit várunk:
        - Ha jobb oldal közelebb (1.5m) és bal távol (2.5m) → error > 0
        - followSimple: error = (2.5 - 1.5) * 0.3 = 0.3
        """
        rospy.loginfo("=== TEST 4: Right Closer Wall Following ===")
        
        self.received_error_msg = None
        self.error_msg_received = False
        
        mock_scan = self.create_mock_scan(
            front_distance=3.0,
            left_distance=2.5,   
            right_distance=1.5   
        )
        
        rospy.loginfo("Publikálok szimulált LaserScan-t: bal=2.5m, jobb=1.5m")
        self.scan_publisher.publish(mock_scan)
        
        timeout = rospy.Time.now() + rospy.Duration(MSG_TIMEOUT)
        rate = rospy.Rate(10)
        
        while not self.error_msg_received and rospy.Time.now() < timeout:
            rate.sleep()
        
        self.assertTrue(self.error_msg_received, 
                        "Nem érkezett PidState üzenet!")
        
        expected_error = (2.5 - 1.5) * 0.3
        self.assertAlmostEqual(self.received_error_msg.error, expected_error, places=1,
                               msg=f"Jobb közelebb esetén error={expected_error} kellene legyen")
        
        rospy.loginfo(f"✓ Jobb közelebb: error = {self.received_error_msg.error} ≈ {expected_error}")


if __name__ == '__main__':
    """
    Ez a rész akkor fut le, ha közvetlenül futtatjuk a fájlt
    
    rostest.rosrun():
        - 1. paraméter: csomag neve (package.xml-ben definiált)
        - 2. paraméter: teszt neve (megjelenik a kimenetben)
        - 3. paraméter: teszt osztály
    """
    rostest.rosrun(
        'megoldas_gyor24',  
        'test_pid_error_node',  
        TestPidErrorNode
    )
