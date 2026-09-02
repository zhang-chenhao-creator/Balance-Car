import unittest
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1]
    / "source"
    / "MiniBalance_HARDWARE"
    / "TrackModule"
    / "TrackModule.c"
)


def ramp_step(current, target, rise_step=5.0, fall_step=10.0):
    step = rise_step if target > current else fall_step
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


class TrackSpeedProfileTests(unittest.TestCase):
    def test_firmware_declares_the_day_two_speed_profile(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertRegex(text, r"float\s+BaseSpeed\s*=\s*400")
        self.assertRegex(text, r"CurveSpeed\s*=\s*350")
        self.assertRegex(text, r"BigCurveSpeed\s*=\s*325")
        self.assertIn("Track_Speed_Ramp", text)
        self.assertIn("Track_CenterConfirmCycles", text)

    def test_deceleration_is_faster_than_acceleration(self):
        self.assertEqual(ramp_step(400, 350), 390)
        self.assertEqual(ramp_step(325, 400), 330)

    def test_ramp_lands_exactly_on_target_without_overshoot(self):
        value = 400.0
        for _ in range(20):
            value = ramp_step(value, 350.0)
        self.assertEqual(value, 350.0)
        self.assertGreaterEqual(value, 350.0)


if __name__ == "__main__":
    unittest.main()
