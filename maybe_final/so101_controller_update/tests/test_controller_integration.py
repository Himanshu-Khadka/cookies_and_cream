"""Offline regression checks. No robotic arm/serial device required."""
import importlib.util
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'lib'))
sys.path.insert(0, str(ROOT / 'working_code'))

import read_encoders
import read_controller
import stream_encoders_to_arm as arm


def reading(**overrides):
    data = dict(arm_roll=-76.73, arm_twist=7.71, arm_v=0.0,
                joy_y=0.0, trigger_percent=0.0, button=1)
    data.update(overrides)
    return SimpleNamespace(**data)


class TestController(unittest.TestCase):
    def test_reader_9_field_combined_stream(self):
        r = read_encoders.parse_stream_line('12000,-76.73,7.71,280.0,0.5,-0.1,1,1,70.5')
        self.assertEqual(r.button, 1)
        self.assertEqual(r.trigger, 1)
        self.assertEqual(r.trigger_percent, 70.5)
        self.assertEqual(r.joy_y, -0.1)

    def test_old_firmware_is_refused_for_proportional_grip(self):
        r = read_encoders.parse_stream_line('12000,-76.73,7.71,280.0,0.5,-0.1,1,1')
        self.assertTrue(math.isnan(r.trigger_percent))
        self.assertIn('trigger_percent', arm.valid_input(reading(trigger_percent=r.trigger_percent)))
        self.assertTrue(math.isnan(read_encoders.parse_stream_line('12000,-76.73,7.71,280.0').trigger_percent))

    def test_stlink_trigger_percent_word_layout(self):
        def f(v): return struct.unpack('<I', struct.pack('<f', v))[0]
        raw = [read_controller.SHARE_MAGIC, 18, 1000, f(1), f(2), f(3), f(0.25), f(-0.5),
               0x00400080, 3, 0x00010001, f(66.7)]
        class FakeOCD:
            def read_words(self, address, count): return raw[:count]
        instance = read_controller.ControllerReader.__new__(read_controller.ControllerReader)
        instance._ocd = FakeOCD()
        instance._address = 0x20000000
        instance._last_count = None
        instance._last_change = 0
        val = instance.read()
        self.assertAlmostEqual(val.trigger_percent, 66.7, places=4)
        self.assertEqual(val.button, 1)
        self.assertEqual(val.joy_y, -0.5)

    def test_mapping(self):
        args = arm.parser().parse_args([])
        state = arm.TargetState(args)
        pan, roll = state.mapped(reading())
        self.assertAlmostEqual(pan, 2.676, places=2)
        self.assertAlmostEqual(roll, -27.6228, places=2)
        self.assertEqual(state.mapped(reading(arm_roll=-800))[1], -180)

    def test_velocity_curve_and_deadzone(self):
        self.assertEqual(arm.shaped_velocity(0.05, .1, 2), 0)
        self.assertEqual(arm.shaped_velocity(1, .1, 2), 2)
        self.assertEqual(arm.shaped_velocity(-1, .1, 2), -2)
        self.assertLess(abs(arm.shaped_velocity(.5, .1, 2)), abs(arm.shaped_velocity(.9, .1, 2)))

    def test_clutch_rebases_no_jump(self):
        args = arm.parser().parse_args([])
        state = arm.TargetState(args)
        first = reading(arm_roll=-450, arm_twist=400)
        state.engage(first, pan=17, roll=-33, y_cm=3, z_cm=-2, pitch_deg=15)
        state.update(first, 0.125, 'deg')
        self.assertAlmostEqual(state.pan_target, 17)
        self.assertAlmostEqual(state.roll_target, -33)
        self.assertAlmostEqual(state.y_cm, 3)
        self.assertAlmostEqual(state.z_cm, -2)
        state.pause()
        state.engage(reading(arm_roll=300, arm_twist=-400), pan=17, roll=-33,
                     y_cm=3, z_cm=-2, pitch_deg=15)
        state.update(reading(arm_roll=300, arm_twist=-400), 0.125, 'deg')
        self.assertAlmostEqual(state.pan_target, 17)
        self.assertAlmostEqual(state.roll_target, -33)

    def test_rail_velocity_and_filter(self):
        a = arm.parser().parse_args([])
        state = arm.TargetState(a)
        state.engage(reading(), pan=0, roll=0, y_cm=0, z_cm=0, pitch_deg=0)
        for i in range(30):
            state.update(reading(arm_v=360, joy_y=.8), .125, 'deg')
        self.assertGreater(state.y_cm, 0)
        self.assertGreater(state.z_cm, 0)
        state.update(reading(arm_v=0, joy_y=0), .125, 'deg')
        self.assertLess(state.z_speed, a.z_max_speed)

    def test_bad_sensor_freezes(self):
        self.assertIn('arm_v', arm.valid_input(reading(arm_v=math.nan)))
        self.assertIn('turn count', arm.valid_input(reading(), rail_uncertain=True))
        self.assertIsNone(arm.valid_input(reading(button=0, arm_v=math.nan)))

    def test_gripper_endpoints_and_reverse(self):
        self.assertEqual(arm.gripper_target(0, (-50, 50), False), -50)
        self.assertEqual(arm.gripper_target(100, (-50, 50), False), 50)
        self.assertEqual(arm.gripper_target(0, (-50, 50), True), 50)
        self.assertEqual(arm.gripper_target(100, (-50, 50), True), -50)

if __name__ == '__main__':
    unittest.main()
