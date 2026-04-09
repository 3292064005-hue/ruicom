import importlib
import sys
import types
import unittest
from unittest import mock


class _FakeTimeValue:
    def __init__(self, value):
        self._value = float(value)

    def to_sec(self):
        return self._value


class _FakeTimeModule:
    current_value = 0.0

    @staticmethod
    def now():
        return _FakeTimeValue(_FakeTimeModule.current_value)

    @staticmethod
    def from_sec(value):
        return _FakeTimeValue(value)


class NodeClockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._original_rospy = sys.modules.get('rospy')
        fake_rospy = types.SimpleNamespace(Time=_FakeTimeModule)
        sys.modules['rospy'] = fake_rospy
        cls.time_core = importlib.import_module('ruikang_recon_baseline.time_core')

    @classmethod
    def tearDownClass(cls):
        if cls._original_rospy is None:
            sys.modules.pop('rospy', None)
        else:
            sys.modules['rospy'] = cls._original_rospy

    def test_ros_business_time_uses_rospy_time(self):
        _FakeTimeModule.current_value = 12.5
        clock = self.time_core.NodeClock('ros')
        self.assertEqual(clock.now_business_sec(), 12.5)
        self.assertEqual(clock.now_ros_time().to_sec(), 12.5)

    def test_ros_clock_zero_is_preserved(self):
        _FakeTimeModule.current_value = 0.0
        clock = self.time_core.NodeClock('ros')
        self.assertEqual(clock.now_business_sec(), 0.0)

    def test_wall_business_time_uses_time_time(self):
        clock = self.time_core.NodeClock('wall')
        with mock.patch.object(self.time_core.time, 'time', return_value=42.0):
            self.assertEqual(clock.now_business_sec(), 42.0)
            self.assertEqual(clock.now_wall_sec(), 42.0)
