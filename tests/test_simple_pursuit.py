#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Unit tesztek a simple_pursuit.py matematikai függvényeihez
"""
#python -m pytest tests/test_simple_pursuit.py -v

import pytest
import math
import numpy as np
import sys
import os
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import simple_pursuit


class MockLaserScan:
    def __init__(self, ranges, angle_min=-math.pi, angle_max=math.pi):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / len(ranges)


@pytest.fixture(autouse=True)
def reset_simple_pursuit_globals(monkeypatch):
    simple_pursuit.prev_steering_err = 0.0
    simple_pursuit.prev_velocity = 0.0
    simple_pursuit.marker_points = SimpleNamespace(points=[])
    simple_pursuit.pub = MagicMock()
    simple_pursuit.pubst1 = MagicMock()
    simple_pursuit.marker_pub = MagicMock()
    yield

class TestCalcPointPos:
    """Tesztek a calcPointPos függvényhez"""
    
    def test_zero_degree(self):
        """Teszt: 0 fok (jobbra), range = 1.0"""
        x, y = simple_pursuit.calcPointPos(1.0, 0.0)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_90_degree(self):
        """Teszt: 90 fok (előre), range = 2.0"""
        x, y = simple_pursuit.calcPointPos(2.0, math.radians(90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(2.0, abs=1e-9)
    
    def test_180_degree(self):
        """Teszt: 180 fok (balra), range = 1.5"""
        x, y = simple_pursuit.calcPointPos(1.5, math.radians(180))
        assert x == pytest.approx(-1.5, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_zero_range(self):
        """Teszt: range = 0, bármilyen szög"""
        x, y = simple_pursuit.calcPointPos(0.0, math.radians(45))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_negative_angle(self):
        """Teszt: negatív szög (-90 fok), range = 3.0"""
        x, y = simple_pursuit.calcPointPos(3.0, math.radians(-90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(-3.0, abs=1e-9)



class TestCalcPursuitAngle:
    """Tesztek a calcPursuitAngle függvényhez"""

    def test_straight_ahead(self):
        """Teszt: cél egyenesen előre (x=1.0, y=0.0)"""
        angle = simple_pursuit.calcPursuitAngle(1.0, 0.0)
        assert angle == pytest.approx(0.0)

    def test_sideway(self):
        """Teszt: cél oldalra (x=0.0, y=1.0)"""
        angle = simple_pursuit.calcPursuitAngle(0.0, 1.0)
        #python -c "import math; WHEELBASE=0.3187; goal_x=0.0; goal_y=1.0; alpha=math.atan2(goal_y, goal_x); ld=math.sqrt(goal_x**2 + goal_y**2); print(math.atan2(2.0*WHEELBASE*math.sin(alpha)/ld, 1))"
        assert angle == pytest.approx(0.5674, abs=1e-4)  

    def test_diagonal(self):
        """Teszt: cél Átlósan (x=1.0, y=1.0) - 45 fok"""
        angle = simple_pursuit.calcPursuitAngle(1.0, 1.0)
        assert angle == pytest.approx(0.3085, abs=1e-4) 

    def test_close_distance(self):
        """Teszt: cél rövid távon, bármilyen irányba (x=0.0, y=0.1)"""
        angle = simple_pursuit.calcPursuitAngle(0.0, 0.1)
        assert angle == pytest.approx(1.4151, abs=1e-4) 

    def test_far_distance(self):
        """Teszt: cél hosszú távon, bármilyen irányba (x=100.0, y=10.0)"""
        angle = simple_pursuit.calcPursuitAngle(100.0, 10.0)
        assert angle == pytest.approx(0.0006, abs=1e-4) 


class TestSimplePursuitCoreLogic:
    def test_get_distance_short_scan_returns_default(self):
        ranges = [1.0] * 10
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)

        distance = simple_pursuit.getDistance(ranges, angles)

        assert distance == pytest.approx(0.4)

    def test_get_distance_reverse_zone_clamp(self):
        ranges = [2.0] * 360
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=True)

        reverse_idx = np.where(np.radians(170) < angles)[0][0]
        ranges[reverse_idx] = 0.2

        distance = simple_pursuit.getDistance(ranges, angles)

        assert distance == pytest.approx(0.5)

    def test_get_angle_symmetric_environment(self):
        ranges = [2.0] * 360
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=True)

        angle, left_d, right_d = simple_pursuit.getAngle(ranges, angles)

        assert angle == pytest.approx(0.0, abs=0.05)
        assert left_d < 0.0
        assert right_d > 0.0

    def test_follow_simple_applies_transform_and_smoothing(self, monkeypatch):
        mock_scan = MockLaserScan([1.0] * 360)

        monkeypatch.setattr(simple_pursuit, 'getDistance', lambda ranges, angles: -1.0)
        monkeypatch.setattr(simple_pursuit, 'getAngle', lambda ranges, angles: (0.2, 1.0, -1.0))

        def fake_transform(point_stamped, _):
            return SimpleNamespace(point=SimpleNamespace(x=2.0, y=1.0))

        def fake_pursuit_angle(goal_x, goal_y):
            assert goal_x == pytest.approx(1.8)
            assert goal_y == pytest.approx(1.0)
            return 0.6

        monkeypatch.setattr(simple_pursuit.tf2_geometry_msgs, 'do_transform_point', fake_transform)
        monkeypatch.setattr(simple_pursuit, 'calcPursuitAngle', fake_pursuit_angle)

        steering_err, velocity = simple_pursuit.followSimple(mock_scan)

        assert steering_err == pytest.approx(0.3)
        assert velocity == pytest.approx(0.5)
        assert simple_pursuit.prev_steering_err == pytest.approx(0.3)
        assert simple_pursuit.prev_velocity == pytest.approx(0.5)
        assert simple_pursuit.marker_points.points == []

    def test_follow_simple_fallback_when_transform_fails(self, monkeypatch):
        mock_scan = MockLaserScan([1.0] * 360)

        monkeypatch.setattr(simple_pursuit, 'getDistance', lambda ranges, angles: -2.0)
        monkeypatch.setattr(simple_pursuit, 'getAngle', lambda ranges, angles: (0.1, 0.5, -0.5))

        def failing_transform(point_stamped, _):
            raise RuntimeError("transform failure")

        calls = []

        def fake_pursuit_angle(goal_x, goal_y):
            calls.append((goal_x, goal_y))
            return -0.4

        monkeypatch.setattr(simple_pursuit.tf2_geometry_msgs, 'do_transform_point', failing_transform)
        monkeypatch.setattr(simple_pursuit, 'calcPursuitAngle', fake_pursuit_angle)

        steering_err, velocity = simple_pursuit.followSimple(mock_scan)

        assert calls == [(1, -1)]
        assert steering_err == pytest.approx(-0.2)
        assert velocity == pytest.approx(1.0)

    def test_callback_laser_publishes_scaled_cmd_vel(self, monkeypatch):
        class DummyTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0)
                self.angular = SimpleNamespace(z=0.0)

        monkeypatch.setattr(simple_pursuit, 'followSimple', lambda data: (0.4, 2.0))
        monkeypatch.setattr(simple_pursuit, 'Twist', DummyTwist)

        simple_pursuit.callbackLaser(SimpleNamespace())

        published_msg = simple_pursuit.pub.publish.call_args[0][0]
        assert published_msg.angular.z == pytest.approx(0.4)
        assert published_msg.linear.x == pytest.approx(1.0)