#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Unit tesztek a simple_pursuit.py matematikai függvényeihez
"""

import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from simple_pursuit import calcPointPos, calcPursuitAngle

class TestCalcPointPos:
    """Tesztek a calcPointPos függvényhez"""
    
    def test_zero_degree(self):
        """Teszt: 0 fok (jobbra), range = 1.0"""
        x, y = calcPointPos(1.0, 0.0)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_90_degree(self):
        """Teszt: 90 fok (előre), range = 2.0"""
        x, y = calcPointPos(2.0, math.radians(90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(2.0, abs=1e-9)
    
    def test_180_degree(self):
        """Teszt: 180 fok (balra), range = 1.5"""
        x, y = calcPointPos(1.5, math.radians(180))
        assert x == pytest.approx(-1.5, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_zero_range(self):
        """Teszt: range = 0, bármilyen szög"""
        x, y = calcPointPos(0.0, math.radians(45))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_negative_angle(self):
        """Teszt: negatív szög (-90 fok), range = 3.0"""
        x, y = calcPointPos(3.0, math.radians(-90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(-3.0, abs=1e-9)



class TestCalcPursuitAngle:
    """Tesztek a calcPursuitAngle függvényhez"""

    def test_straight_ahead(self):
        """Teszt: cél egyenesen előre (x=1.0, y=0.0)"""
        angle = calcPursuitAngle(1.0, 0.0)
        assert angle == pytest.approx(0.0)

    def test_sideway(self):
        """Teszt: cél oldalra (x=0.0, y=1.0)"""
        angle = calcPursuitAngle(0.0, 1.0)
        #python -c "import math; WHEELBASE=0.3187; goal_x=0.0; goal_y=1.0; alpha=math.atan2(goal_y, goal_x); ld=math.sqrt(goal_x**2 + goal_y**2); print(math.atan2(2.0*WHEELBASE*math.sin(alpha)/ld, 1))"
        assert angle == pytest.approx(0.5674, abs=1e-4)  

    def test_diagonal(self):
        """Teszt: cél Átlósan (x=1.0, y=1.0) - 45 fok"""
        angle = calcPursuitAngle(1.0, 1.0)
        assert angle == pytest.approx(0.3085, abs=1e-4) 

    def test_close_distance(self):
        """Teszt: cél rövid távon, bármilyen irányba (x=0.0, y=0.1)"""
        angle = calcPursuitAngle(0.0, 0.1)
        assert angle == pytest.approx(1.4151, abs=1e-4) 

    def test_far_distance(self):
        """Teszt: cél hosszú távon, bármilyen irányba (x=100.0, y=10.0)"""
        angle = calcPursuitAngle(100.0, 10.0)
        assert angle == pytest.approx(0.0006, abs=1e-4) 