import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1] / "source"
AVOID = ROOT / "MiniBalance" / "CONTROL" / "avoid_routine.c"
GUARD = ROOT / "MiniBalance" / "CONTROL" / "obstacle_guard.h"
TRACK = ROOT / "MiniBalance_HARDWARE" / "TrackModule" / "TrackModule.c"
US = ROOT / "MiniBalance_HARDWARE" / "ULTRASONIC" / "ultrasonic_service.c"


class ObstacleAvoidTests(unittest.TestCase):
    def test_path_is_teammate_square_not_perfect_arc(self):
        text = AVOID.read_text(encoding="utf-8")
        for name in (
            "AVOID_STOP_SETTLE",
            "AVOID_TURN1_RIGHT",
            "AVOID_LEG1",
            "AVOID_TURN2_LEFT",
            "AVOID_LEG2",
            "AVOID_TURN3_LEFT",
            "AVOID_SEEK_LINE",
            "AVOID_ALIGN_RIGHT",
            "AVOID_REJOIN_FOLLOW",
        ):
            self.assertIn(name, text)
        self.assertNotIn("OBSTACLE_DIAGONAL", text)
        self.assertNotIn("OBSTACLE_RIGHT_ANGLE", text)

    def test_align_uses_gyro_not_only_straight_pattern(self):
        text = AVOID.read_text(encoding="utf-8")
        self.assertIn("AVOID_ALIGN_MIN_DEG", text)
        self.assertIn("s_turn_deg >= Avoid_TurnAngle", text)
        self.assertIn("AVOID_REJOIN_FOLLOW", text)

    def test_stop_distance_is_farther_than_teammate_default(self):
        text = GUARD.read_text(encoding="utf-8")
        self.assertIn("OBSTACLE_GUARD_STOP_BASE_MM          200U", text)
        self.assertIn("OBSTACLE_GUARD_STOP_MAX_MM           360U", text)

    def test_ultrasonic_uses_65ms_period(self):
        header = (ROOT / "MiniBalance_HARDWARE" / "ULTRASONIC" / "ultrasonic_service.h").read_text(encoding="utf-8")
        self.assertIn("US_SERVICE_PERIOD_5MS     13U", header)
        self.assertTrue(US.exists())

    def test_stable_line_follow_has_no_figure_eight_route(self):
        text = TRACK.read_text(encoding="utf-8")
        self.assertIn("Track_Speed_Ramp", text)
        self.assertNotIn("Track_Route", text)
        self.assertNotIn("RUN_TURN", text)


if __name__ == "__main__":
    unittest.main()
