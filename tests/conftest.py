# -*- coding: utf-8 -*-
"""
pytest konfiguráció - mock ROS modulok
"""
import sys
from unittest.mock import MagicMock

# Mock-oljuk a ROS modulokat, mielőtt bármit importálnánk
sys.modules['rospy'] = MagicMock()
sys.modules['tf2_ros'] = MagicMock()
sys.modules['tf2_geometry_msgs'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['visualization_msgs'] = MagicMock()
sys.modules['visualization_msgs.msg'] = MagicMock()
sys.modules['control_msgs'] = MagicMock()
sys.modules['control_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()