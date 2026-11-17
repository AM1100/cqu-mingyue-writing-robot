"""Offline checks only: never instantiate real PWM or import machine."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware'))
import CQU_fivebar_fixed as f
import main as entry


class OfflineFirmwareTests(unittest.TestCase):
    def test_import_does_not_load_hardware(self):
        self.assertNotIn('machine', sys.modules)

    def test_distribution_disables_automatic_writing(self):
        self.assertIs(f.CALIBRATED, False)
        self.assertIs(entry.AUTOSTART, False)
        with patch.object(f, 'make_servo') as make:
            self.assertIsNone(entry.run())
            make.assert_not_called()

    def test_uncalibrated_start_rejected_before_pwm(self):
        with patch.object(f, 'make_servo') as make:
            with self.assertRaises(ValueError):
                entry.validate_ready(text='8')
            make.assert_not_called()

    def test_geometry_round_trip(self):
        for x in (3, 13, 23):
            for y in (55, 60, 65, 70):
                a, b = f.solve_pure_geom(x, y)
                px, py = f.forward(a, b)
                self.assertAlmostEqual(px, x, places=7)
                self.assertAlmostEqual(py, y, places=7)

    def check_calibration(self, g1, g2, expected_direction):
        c = f.calibrate_side_two_point(90, g1, 110, g2)
        self.assertEqual(c['direction'], expected_direction)
        for e, g in ((90, g1), (110, g2)):
            got = f.angle_to_pulse(g, c['direction'], c['trim_deg'],
                                   c['geom_center_deg'], c['us_per_deg'])
            self.assertAlmostEqual(got, 1500 + f._US_PER_DEG * (e - 90), places=6)

    def test_positive_two_point_calibration(self):
        self.check_calibration(120, 130, 1)

    def test_reverse_two_point_calibration(self):
        self.check_calibration(60, 50, -1)

    def test_unreachable_point_rejected(self):
        with self.assertRaises(ValueError):
            f.solve_pure_geom(999, 999)

    def test_three_character_group_centered_in_64_mm(self):
        boxes = f.text_boxes('CQU', board_center=(13, 65))
        self.assertEqual(len(boxes), 3)
        self.assertAlmostEqual(boxes[-1][1][2] - boxes[0][1][0], 64)
        self.assertAlmostEqual((boxes[0][1][0] + boxes[-1][1][2]) / 2, 13)

    def test_diagnostic_does_not_create_pwm(self):
        with patch.object(f, 'make_servo') as make:
            result = f.diagnose_text('8')
            self.assertIn('ok', result)
            make.assert_not_called()


if __name__ == '__main__':
    unittest.main()
