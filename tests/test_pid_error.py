#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Unit tesztek a pid_error.py függvényeihez
"""
#python -m pytest tests/test_pid_error.py -v

import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from pid_error import getRange, followSimple, followCenter


class MockLaserScan:
    """Mock LaserScan objektum teszteléshez"""
    def __init__(self, ranges):
        self.ranges = ranges


class TestGetRange:
    """Tesztek a getRange függvényhez"""
    
    def test_basic_range_at_zero(self):
        """Teszt: 0 fok -> index 90"""
        ranges = [float(i) for i in range(360)]  # 0, 1, 2, ..., 359
        mock_scan = MockLaserScan(ranges)
        
        dist = getRange(mock_scan, 0)
        # angle=0 -> index = 360 * (0 + 90) / 360 = 90 -> érték: 90.0
        assert dist == pytest.approx(90.0)
    
    def test_inf_value(self):
        """Teszt: végtelen érték kezelése"""
        ranges = [1.0] * 360
        ranges[90] = float('inf')  # 0 fok pozícióra inf
        mock_scan = MockLaserScan(ranges)
        
        dist = getRange(mock_scan, 0)
        assert dist == pytest.approx(0.02) 
    
    def test_nan_value(self):
        """Teszt: NaN érték kezelése"""
        ranges = [1.0] * 360
        ranges[90] = float('nan')  # 0 fok pozícióra NaN
        mock_scan = MockLaserScan(ranges)
        
        dist = getRange(mock_scan, 0)
        assert dist == pytest.approx(10.0)
    
    def test_angle_limit(self):
        """Teszt: szög korlátozás (>269.9)"""
        ranges = [float(i) for i in range(360)]
        mock_scan = MockLaserScan(ranges)
        
        # 300 fok > 269.9, ezért korlátozza 269.9-re
        dist = getRange(mock_scan, 300)
        assert dist == pytest.approx(359.0)




class TestFollowSimple:
    """Tesztek a followSimple függvényhez - falkövetés logika"""
    
    def test_symmetric_corridor(self):
        """Teszt: szimmetrikus folyosó (egyenes menet)"""
        # Minden pozícióra alapértelmezett 2.0 méter
        ranges = [2.0] * 360
    
        # -30 fok -> index = 360 * (-30 + 90) / 360 = 60
        ranges[60] = 1.0   # Bal oldal: left_d
    
        # 210 fok -> index = 360 * (210 + 90) / 360 = 300
        ranges[300] = 1.0  # Jobb oldal: right_d
    
        # 270 fok -> korlátozva 269.9 -> index = 360 * (269.9 + 90) / 360 = 359.9 -> int(359.9) = 359
        ranges[359] = 5.0  # Előre: forward_d
    
        mock_scan = MockLaserScan(ranges)
        error, curr_dist, velocity = followSimple(mock_scan)
    
        # error = (left_d - right_d) * 0.3
        assert error == pytest.approx(0.0)
    
        # velocity = forward_d * 0.6
        assert velocity == pytest.approx(3.0)
    
        # curr_dist mindig 0
        assert curr_dist == 0
    
    
    def test_pull_right(self):
        """Teszt: bal oldal közelebb -> jobbra húz"""
        ranges = [2.0] * 360
    
        ranges[60] = 0.5   # Bal oldal: left_d
    
        ranges[300] = 1.5  # Jobb oldal: right_d
    
        ranges[359] = 5.0  # Előre: forward_d
    
        mock_scan = MockLaserScan(ranges)
        error, curr_dist, velocity = followSimple(mock_scan)
    
        assert error == pytest.approx(-0.3)
    
        assert velocity == pytest.approx(3.0)
    
        assert curr_dist == 0
    
    
    def test_obstacle_ahead_slow(self):
        """Teszt: akadály előre -> lassítás"""
        ranges = [2.0] * 360
    
        ranges[60] = 1.0   # Bal oldal: left_d
    
        ranges[300] = 1.0  # Jobb oldal: right_d
    
        ranges[359] = 2.0  # Előre: forward_d
    
        mock_scan = MockLaserScan(ranges)
        error, curr_dist, velocity = followSimple(mock_scan)
    
        assert error == pytest.approx(0.0)
    
        assert velocity == pytest.approx(1.2)
    
        assert curr_dist == 0
    
    def test_obstacle_ahead_stop(self):
        """Teszt: nagyon közeli akadály -> megállás"""
        ranges = [2.0] * 360
    
        ranges[60] = 1.0   # Bal oldal: left_d
    
        ranges[300] = 1.0  # Jobb oldal: right_d
    
        ranges[359] = 0.2  # Előre: forward_d
    
        mock_scan = MockLaserScan(ranges)
        error, curr_dist, velocity = followSimple(mock_scan)
    
        assert error == pytest.approx(0.0)
    
        assert velocity == pytest.approx(0.0)
    
        assert curr_dist == 0



class TestFollowCenter:
    """Tesztek a followCenter függvényhez - középvonal követés"""
    
    def test_centered_parallel_walls(self):
        """Teszt: robot középen, párhuzamos falak"""
        ranges = [1.0] * 360
        
        # Bal oldal mérések (0° és -60°)
        # index(0°) = 90, index(-60°) = 30
        ranges[90] = 2.0   # a (bal)
        ranges[30] = 2.0   # b (bal)
        
        # Jobb oldal mérések (180° és 210°)
        # index(180°) = 270, index(210°) = 300
        ranges[270] = 2.0  # a (jobb)
        ranges[300] = 2.0  # b (jobb)
        
        mock_scan = MockLaserScan(ranges)
        error, curr_dist = followCenter(mock_scan)
        
        assert error == pytest.approx(0.0, abs=0.1) 
    
    def test_reference_case(self):
        """Teszt: referencia eset - regression test"""
        # Konkrét ismert konfiguráció
        ranges = [3.0] * 360
        ranges[90] = 2.5
        ranges[30] = 2.0
        ranges[270] = 2.5
        ranges[300] = 2.0
        
        mock_scan = MockLaserScan(ranges)
        error, curr_dist = followCenter(mock_scan)
        
        assert error == pytest.approx(0, abs=0.01)
        assert curr_dist == pytest.approx(0, abs=0.01)

    def test_closer_to_left_wall(self):
        """Teszt: robot közelebb a bal falhoz -> jobbra korrigál"""
        ranges = [5.0] * 360
    
        # Bal oldal: KÖZEL (1 méter)
        ranges[90] = 1.0   # 0° - bal
        ranges[30] = 1.0   # -60° - bal
    
        # Jobb oldal: TÁVOL (3 méter)
        ranges[270] = 3.0  # 180° - jobb
        ranges[300] = 3.0  # 210° - jobb
    
        mock_scan = MockLaserScan(ranges)
        error, curr_dist = followCenter(mock_scan)
    
        # Robot közelebb van balra -> error < 0 (jobbra kell menni)
        assert error < 0.0
        # curr_dist = jobb - bal = 3 - 1 = 2 (jobb oldal távolabb)
        assert curr_dist > 0.0