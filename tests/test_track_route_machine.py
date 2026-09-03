import re
import unittest
from pathlib import Path

from track_route_machine import (
    ACT_LEFT,
    ACT_RIGHT,
    ACT_STOP,
    FINISH_SPEED,
    BRAKE_CYCLES,
    JUNCTION_SPEED,
    LOST_DEBOUNCE,
    RECOVER_CYCLES,
    RUN_BRAKE,
    RUN_FAULT,
    RUN_FINISH,
    RUN_PATROL,
    RUN_SEARCH,
    RUN_TURN,
    RUN_WAIT_START,
    SEARCH_TIMEOUT_CYCLES,
    SIDE_LEFT,
    SIDE_RIGHT,
    START_CENTER_CYCLES,
    STATE_CROSS,
    STATE_LEFT_90_A,
    STATE_LEFT_90_B,
    STATE_LEFT_SMALL,
    STATE_RIGHT_SMALL,
    STATE_LOST,
    STATE_RIGHT_90_A,
    STATE_RIGHT_90_B,
    STATE_RIGHT_BIG,
    STATE_STRAIGHT,
    TURN90,
    TURN_MAX,
    TURN_MIN_CYCLES,
    TURN_TIMEOUT_CYCLES,
    RouteMachine,
)


SOURCE = (
    Path(__file__).parents[1]
    / "source"
    / "MiniBalance_HARDWARE"
    / "TrackModule"
    / "TrackModule.c"
)


def leave_start(machine=None):
    machine = machine or RouteMachine()
    machine.feed(STATE_CROSS, 8)
    machine.feed(STATE_STRAIGHT, START_CENTER_CYCLES)
    return machine


def finish_turn(machine, gap=STATE_RIGHT_BIG):
    machine.feed(gap, BRAKE_CYCLES)
    machine.feed(gap, TURN_MIN_CYCLES)
    machine.feed(STATE_STRAIGHT, 3)
    machine.feed(STATE_STRAIGHT, START_CENTER_CYCLES)
    return machine


class FirmwareContractTests(unittest.TestCase):
    def test_route_table_is_left_right_right_left_stop(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertRegex(
            text,
            r"\{\s*TRIG_LEFT_FORK,\s*ACT_LEFT\s*\}",
        )
        self.assertRegex(
            text,
            r"\{\s*TRIG_CROSS,\s*ACT_RIGHT\s*\}",
        )
        self.assertRegex(
            text,
            r"\{\s*TRIG_RIGHT_FORK,\s*ACT_RIGHT\s*\}",
        )
        self.assertRegex(
            text,
            r"\{\s*TRIG_CROSS,\s*ACT_LEFT\s*\}",
        )
        self.assertRegex(
            text,
            r"\{\s*TRIG_CROSS,\s*ACT_STOP\s*\}",
        )

    def test_junction_speed_is_zero_for_in_place_turn(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertRegex(text, r"float\s+JunctionSpeed\s*=\s*0")
        self.assertRegex(text, r"float\s+LostSpeed\s*=\s*200")
        self.assertRegex(text, r"float\s+FinishSpeed\s*=\s*200")
        self.assertIn("TRACK_BRAKE_CYCLES", text)
        self.assertIn("Track_InPlaceTurn", text)
        self.assertIn("RUN_BRAKE", text)

    def test_left_fork_includes_1100_and_1110(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertIn("STATE_RIGHT_90_A", text)
        self.assertIn("STATE_RIGHT_90_B", text)
        self.assertIn("STATE_RIGHT_BIG", text)
        match = re.search(
            r"Track_IsLeftFork[\s\S]*?STATE_RIGHT_BIG",
            text,
        )
        self.assertIsNotNone(match)


class RouteMachineTests(unittest.TestCase):
    def test_start_cross_does_not_consume_route(self):
        machine = RouteMachine()
        machine.feed(STATE_CROSS, 10)
        self.assertEqual(machine.run, RUN_WAIT_START)
        self.assertEqual(machine.index, 0)

    def test_ordinary_left_bias_does_not_consume_route(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_SMALL, 8)
        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 0)
        self.assertEqual(machine.turn_diff, -15.0)

    def test_left_nudge_uses_min_turn_and_keeps_index(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_SMALL, 4)
        self.assertEqual(machine.index, 0)
        self.assertLess(machine.turn_diff, 0)
        self.assertEqual(machine.turn_diff, -15.0)

    def test_clean_left_fork_locks_left_once(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_A, 1)
        self.assertEqual(machine.run, RUN_BRAKE)
        self.assertEqual(machine.index, 1)
        self.assertEqual(machine.locked_act, ACT_LEFT)
        self.assertEqual(machine.turn_diff, 0.0)
        self.assertEqual(machine.base_speed, 0.0)
        self.assertEqual(machine.in_place, 1)
        machine.feed(STATE_RIGHT_90_A, BRAKE_CYCLES)
        self.assertEqual(machine.run, RUN_TURN)
        self.assertEqual(machine.turn_diff, -TURN90)
        self.assertEqual(machine.index, 1)

    def test_sloppy_1100_also_locks_left(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_B, 1)
        self.assertEqual(machine.run, RUN_BRAKE)
        self.assertEqual(machine.locked_act, ACT_LEFT)
        machine.feed(STATE_RIGHT_90_B, BRAKE_CYCLES)
        self.assertEqual(machine.turn_diff, -TURN90)

    def test_early_1110_locks_left_when_armed(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_BIG, 1)
        self.assertEqual(machine.run, RUN_BRAKE)
        self.assertEqual(machine.locked_act, ACT_LEFT)

    def test_lock_snaps_off_cruise_speed(self):
        machine = leave_start()
        machine.feed(STATE_STRAIGHT, 40)
        self.assertGreater(machine.base_speed, 100.0)
        machine.feed(STATE_RIGHT_90_A, 1)
        self.assertEqual(machine.run, RUN_BRAKE)
        self.assertEqual(machine.base_speed, 0.0)
        self.assertEqual(machine.in_place, 1)

    def test_unarmed_1100_does_not_lock_left(self):
        machine = RouteMachine()
        machine.feed(STATE_RIGHT_90_B, 8)
        self.assertEqual(machine.run, RUN_WAIT_START)
        self.assertEqual(machine.index, 0)

    def test_right_fork_does_not_consume_left_command(self):
        machine = leave_start()
        machine.feed(STATE_LEFT_90_A, 6)
        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 0)

    def test_curve_0011_is_not_a_right_fork(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_A, 1)
        finish_turn(machine)
        machine.feed(STATE_CROSS, 2)
        finish_turn(machine, gap=STATE_LEFT_SMALL)
        machine.feed(STATE_LEFT_90_B, 8)
        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 2)

    def test_turn_ignores_lost_and_does_not_enter_search(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_A, 1)
        machine.feed(STATE_LOST, BRAKE_CYCLES + 10)
        self.assertEqual(machine.run, RUN_TURN)
        self.assertEqual(machine.turn_diff, -TURN90)

    def test_short_lost_blip_does_not_search(self):
        machine = leave_start()
        machine.feed(STATE_LOST, LOST_DEBOUNCE - 1)
        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 0)

    def test_left_overshoot_searches_left(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_SMALL, 1)
        machine.feed(STATE_LOST, LOST_DEBOUNCE)
        self.assertEqual(machine.run, RUN_SEARCH)
        self.assertEqual(machine.last_side, SIDE_LEFT)
        self.assertEqual(machine.turn_diff, -TURN_MAX)

    def test_right_overshoot_searches_right(self):
        machine = leave_start()
        machine.feed(STATE_LEFT_90_B, 1)
        machine.feed(STATE_LOST, LOST_DEBOUNCE)
        self.assertEqual(machine.run, RUN_SEARCH)
        self.assertEqual(machine.last_side, SIDE_RIGHT)
        self.assertEqual(machine.turn_diff, TURN_MAX)

    def test_center_gap_searches_straight(self):
        machine = leave_start()
        machine.feed(STATE_LOST, LOST_DEBOUNCE)
        self.assertEqual(machine.run, RUN_SEARCH)
        self.assertEqual(machine.turn_diff, 0.0)

    def test_search_recovers_after_stable_line(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_SMALL, 1)
        machine.feed(STATE_LOST, LOST_DEBOUNCE)
        machine.feed(STATE_STRAIGHT, RECOVER_CYCLES)
        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 0)

    def test_search_timeout_stops(self):
        machine = leave_start()
        machine.feed(STATE_LOST, LOST_DEBOUNCE + SEARCH_TIMEOUT_CYCLES)
        self.assertEqual(machine.run, RUN_FAULT)
        self.assertEqual(machine.turn_diff, 0.0)
        machine.feed(STATE_LOST, 25)
        self.assertEqual(machine.base_speed, 0.0)

    def test_full_route_left_right_right_left_stop(self):
        machine = leave_start()
        actions = []

        machine.feed(STATE_RIGHT_90_A, 1)
        actions.append(machine.locked_act)
        self.assertEqual(machine.run, RUN_BRAKE)
        finish_turn(machine)

        machine.feed(STATE_CROSS, 2)
        actions.append(machine.locked_act)
        self.assertEqual(machine.run, RUN_BRAKE)
        finish_turn(machine, gap=STATE_LEFT_SMALL)

        machine.feed(STATE_LEFT_90_A, 1)
        actions.append(machine.locked_act)
        self.assertEqual(machine.run, RUN_BRAKE)
        finish_turn(machine, gap=STATE_LEFT_SMALL)

        machine.feed(STATE_CROSS, 2)
        actions.append(machine.locked_act)
        self.assertEqual(machine.run, RUN_BRAKE)
        finish_turn(machine)

        self.assertEqual(machine.run, RUN_PATROL)
        self.assertEqual(machine.index, 4)
        machine.feed(STATE_STRAIGHT, 4)
        self.assertLessEqual(machine.base_speed, FINISH_SPEED + 1e-6)

        machine.feed(STATE_CROSS, 2)
        actions.append(machine.locked_act)
        self.assertEqual(actions, [ACT_LEFT, ACT_RIGHT, ACT_RIGHT, ACT_LEFT, ACT_STOP])
        self.assertEqual(machine.run, RUN_FINISH)
        machine.feed(STATE_CROSS, 30)
        self.assertEqual(machine.run, RUN_FINISH)
        self.assertEqual(machine.turn_diff, 0.0)
        self.assertEqual(machine.base_speed, 0.0)

    def test_same_cross_does_not_increment_twice(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_A, 1)
        finish_turn(machine)
        machine.feed(STATE_CROSS, 20)
        self.assertEqual(machine.index, 2)
        self.assertIn(machine.run, (RUN_BRAKE, RUN_TURN))

    def test_turn_timeout_keeps_searching_left_instead_of_freezing(self):
        machine = leave_start()
        machine.feed(STATE_RIGHT_90_A, 1)
        machine.feed(STATE_LOST, BRAKE_CYCLES + TURN_TIMEOUT_CYCLES)
        self.assertEqual(machine.run, RUN_SEARCH)
        self.assertEqual(machine.last_side, SIDE_LEFT)
        self.assertEqual(machine.turn_diff, -TURN_MAX)


if __name__ == "__main__":
    unittest.main()
