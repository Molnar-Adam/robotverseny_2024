#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Unit tesztek a control.py PID kontroller függvényéhez
"""

#python -m pytest tests/test_control.py -v


import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import control
from unittest.mock import MagicMock, patch

class MockPidState:
    """Mock PidState üzenet teszteléshez"""
    def __init__(self, error, error_dot, frame_id="simple"):
        self.error = error
        self.error_dot = error_dot
        self.header = MagicMock() 
        self.header.frame_id = frame_id

    
class TestControl:
    
    def setup_method(self):
        """Ez MINDEN teszt ELŐTT lefut"""

        control.kp = 10.0
        control.kd = 0.01
        control.kp_vel = 42.0
        control.kd_vel = 0.0

        control.ki = 0.0
        control.servo_offset = 18.0*math.pi/180
        control.prev_error = 0.0
        control.error = 0.0
        control.integral = 0.0
        control.vel_input = 1.0
        pass


    @patch('control.pub')
    def test_zero_error_simple_mode(self, mock_pub):
        """Teszt: nincs hiba, simple módban"""
        mock_data = MockPidState(error=0.0, error_dot=1.0, frame_id="simple")
        
        control.control(mock_data)
        
        published_msg = mock_pub.publish.call_args[0][0]
        
        # linear.x = velocity * 0.2 = 1.0 * 0.2 = 0.2
        assert published_msg.linear.x == pytest.approx(0.2)
    
        # angular.z = data.error = 0.0
        assert published_msg.angular.z == pytest.approx(0.0)


    @patch('control.pub')
    def test_negative_error_simple_mode(self, mock_pub):    
        """Teszt: negatív hiba, simple módban"""
        mock_data = MockPidState(error=-0.3, error_dot=1.5, frame_id="simple")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
        assert published_msg.linear.x == pytest.approx(0.8)
        assert published_msg.angular.z == pytest.approx(-0.3)


    @patch('control.pub')
    def test_velocity_limit_high(self, mock_pub):    
        """Teszt: Nincs hiba, de nagy sebesség"""
        mock_data = MockPidState(error=0, error_dot=10.0, frame_id="simple")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
        assert published_msg.linear.x == pytest.approx(0.8)
        assert published_msg.angular.z == pytest.approx(0)

        
        
    @patch('control.pub')
    def test_velocity_limit_low(self, mock_pub):    
        """Teszt: Nincs hiba, de alacsony sebesség"""
        mock_data = MockPidState(error=0, error_dot=-1.0, frame_id="simple")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
        assert published_msg.linear.x == pytest.approx(0)
        assert published_msg.angular.z == pytest.approx(0.0)


    @patch('control.pub')
    def test_positive_error_non_simple_mode(self, mock_pub):    
        """Teszt: Pozitív hiba, nem simple mód"""
        mock_data = MockPidState(error=0.1, error_dot=1.0, frame_id="other")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
        assert published_msg.linear.x == pytest.approx(0.06)
        assert published_msg.angular.z == pytest.approx(0.1)

    
    @patch('control.pub')
    def test_prev_error_tracking(self, mock_pub):
        """Teszt: prev_error követése két hívás során"""
    
        # === ELSŐ HÍVÁS ===
        mock_data1 = MockPidState(error=0.5, error_dot=1.0, frame_id="simple")
        control.control(mock_data1)
    

        assert control.prev_error == pytest.approx(2.5)
    
        # === MÁSODIK HÍVÁS ===
        mock_data2 = MockPidState(error=0.3, error_dot=1.0, frame_id="simple")
        control.control(mock_data2)
    

        assert control.prev_error == pytest.approx(1.5)
    
    @patch('control.pub')
    def test_angle_limit_positive_non_simple(self, mock_pub):
        """Teszt: szög limit +30° non-simple módban"""
        mock_data = MockPidState(error=5.0, error_dot=1.0, frame_id="other")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]

        assert published_msg.linear.x == pytest.approx(0.06)
        assert published_msg.angular.z == pytest.approx(5.0)


    @patch('control.pub')
    def test_angle_limit_negative_non_simple(self, mock_pub):
        """Teszt: szög limit -30° non-simple módban"""
        mock_data = MockPidState(error=-5.0, error_dot=1.0, frame_id="other")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
    
        assert published_msg.linear.x == pytest.approx(0.06)
        assert published_msg.angular.z == pytest.approx(-5.0)


    @patch('control.pub')
    def test_small_angle_high_velocity_non_simple(self, mock_pub):
        """Teszt: nagyon kis szög (<1°) -> nagy sebesség non-simple"""
    
        mock_data = MockPidState(error=-0.035, error_dot=1.0, frame_id="other")
    
        control.control(mock_data)
    
        published_msg = mock_pub.publish.call_args[0][0]
    
    
        assert published_msg.linear.x == pytest.approx(0.16)
        assert published_msg.angular.z == pytest.approx(-0.035)


    @patch('control.pub')
    def test_velocity_limit_high_non_simple(self, mock_pub):
        """Teszt: sebesség limit 2.5 non-simple módban"""
        mock_data = MockPidState(error=0.0, error_dot=10.0, frame_id="other")
        
        control.control(mock_data)
        
        published_msg = mock_pub.publish.call_args[0][0]
        

        assert published_msg.linear.x == pytest.approx(0.16)
        assert published_msg.angular.z == pytest.approx(0.0)