"""Host twin of TrackModule.c route / lost-line state machine."""

STATE_CROSS = 0
STATE_LEFT_90_A = 1
STATE_LEFT_90_B = 3
STATE_RIGHT_90_A = 8
STATE_RIGHT_90_B = 12
STATE_LEFT_BIG = 7
STATE_RIGHT_BIG = 14
STATE_LEFT_SMALL = 11
STATE_RIGHT_SMALL = 13
STATE_STRAIGHT = 9
STATE_LOST = 15

RUN_WAIT_START = 0
RUN_PATROL = 1
RUN_TURN = 2
RUN_SEARCH = 3
RUN_FINISH = 4
RUN_FAULT = 5
RUN_BRAKE = 6

ACT_LEFT = 0
ACT_RIGHT = 1
ACT_STOP = 2

TRIG_LEFT_FORK = 0
TRIG_RIGHT_FORK = 1
TRIG_CROSS = 2

SIDE_CENTER = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2

TURN90 = 80.0
TURN_MAX = 65.0
TURN_MIN = 15.0
BASE_SPEED = 400.0
FINE_SPEED = 375.0
CURVE_SPEED = 350.0
BIG_CURVE_SPEED = 325.0
LOST_SPEED = 200.0
JUNCTION_SPEED = 0.0
FINISH_SPEED = 200.0
RISE_STEP = 5.0
FALL_STEP = 10.0
CENTER_CONFIRM = 3

START_CENTER_CYCLES = 6
FORK_DEBOUNCE = 1
CROSS_DEBOUNCE = 2
BRAKE_CYCLES = 40
TURN_MIN_CYCLES = 30
TURN_CENTER_CYCLES = 3
TURN_TIMEOUT_CYCLES = 240
LOST_DEBOUNCE = 3
RECOVER_CYCLES = 3
SEARCH_TIMEOUT_CYCLES = 240
TURN_WIDE_TAPE_CYCLES = 80

ROUTE = [
    (TRIG_LEFT_FORK, ACT_LEFT),
    (TRIG_CROSS, ACT_RIGHT),
    (TRIG_RIGHT_FORK, ACT_RIGHT),
    (TRIG_CROSS, ACT_LEFT),
    (TRIG_CROSS, ACT_STOP),
]


def ramp_step(current, target, rise_step=RISE_STEP, fall_step=FALL_STEP):
    step = rise_step if target > current else fall_step
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def is_confirmed_left_fork(sensor):
    return sensor == STATE_RIGHT_90_A


def is_left_fork(sensor):
    return sensor in (STATE_RIGHT_90_A, STATE_RIGHT_90_B, STATE_RIGHT_BIG)


def is_confirmed_right_fork(sensor):
    return sensor == STATE_LEFT_90_A


def is_right_fork(sensor):
    return is_confirmed_right_fork(sensor)


def matches_trig(sensor, trig):
    if trig == TRIG_LEFT_FORK:
        return is_left_fork(sensor)
    if trig == TRIG_RIGHT_FORK:
        return is_right_fork(sensor)
    return sensor == STATE_CROSS


def is_confirmed_fork(sensor, trig):
    if trig == TRIG_LEFT_FORK:
        return is_confirmed_left_fork(sensor)
    if trig == TRIG_RIGHT_FORK:
        return is_confirmed_right_fork(sensor)
    return sensor == STATE_CROSS


def needed_debounce(trig):
    if trig == TRIG_CROSS:
        return CROSS_DEBOUNCE
    return FORK_DEBOUNCE


def caught_new_line(sensor, act):
    if sensor == STATE_STRAIGHT:
        return True
    if act == ACT_LEFT:
        return sensor in (
            STATE_RIGHT_SMALL,
            STATE_RIGHT_90_A,
            STATE_RIGHT_90_B,
            STATE_RIGHT_BIG,
        )
    if act == ACT_RIGHT:
        return sensor in (
            STATE_LEFT_SMALL,
            STATE_LEFT_90_A,
            STATE_LEFT_90_B,
            STATE_LEFT_BIG,
        )
    return False


def side_from_sensor(sensor):
    if is_left_fork(sensor) or sensor in (STATE_RIGHT_BIG, STATE_RIGHT_SMALL):
        return SIDE_LEFT
    if is_right_fork(sensor) or sensor in (STATE_LEFT_90_B, STATE_LEFT_BIG, STATE_LEFT_SMALL):
        return SIDE_RIGHT
    return SIDE_CENTER


def both_mids_on_line(sensor):
    return (sensor & 0x06) == 0


def follow_turn(sensor):
    mapping = {
        STATE_CROSS: 0.0,
        STATE_LEFT_90_A: TURN90,
        STATE_LEFT_90_B: TURN90,
        STATE_RIGHT_90_A: -TURN90,
        STATE_RIGHT_90_B: -TURN90,
        STATE_LEFT_BIG: TURN_MAX,
        STATE_RIGHT_BIG: -TURN_MAX,
        STATE_LEFT_SMALL: TURN_MIN,
        STATE_RIGHT_SMALL: -TURN_MIN,
        STATE_STRAIGHT: 0.0,
    }
    return mapping.get(sensor, 0.0)


def locked_turn(act):
    if act == ACT_LEFT:
        return -TURN90
    if act == ACT_RIGHT:
        return TURN90
    return 0.0


def search_turn(side):
    if side == SIDE_LEFT:
        return -TURN_MAX
    if side == SIDE_RIGHT:
        return TURN_MAX
    return 0.0


def target_speed_for_state(sensor):
    if sensor == STATE_STRAIGHT:
        return BASE_SPEED
    if sensor in (STATE_LEFT_SMALL, STATE_RIGHT_SMALL):
        return FINE_SPEED
    if sensor in (STATE_LEFT_90_A, STATE_LEFT_90_B, STATE_RIGHT_90_A, STATE_RIGHT_90_B):
        return CURVE_SPEED
    if sensor in (STATE_LEFT_BIG, STATE_RIGHT_BIG, STATE_CROSS):
        return BIG_CURVE_SPEED
    return LOST_SPEED


class RouteMachine:
    def __init__(self):
        self.base_speed = 0.0
        self.turn_diff = 0.0
        self.speed_state = STATE_STRAIGHT
        self.center_stable = 0
        self.run = RUN_WAIT_START
        self.index = 0
        self.locked_act = ACT_LEFT
        self.last_side = SIDE_CENTER
        self.armed = 0
        self.center_hold = 0
        self.junction_count = 0
        self.lost_count = 0
        self.recover_count = 0
        self.turn_cycles = 0
        self.turn_saw_gap = 0
        self.turn_center_count = 0
        self.search_cycles = 0
        self.brake_cycles = 0
        self.in_place = 0

    def _speed_state(self, sensor):
        if sensor == STATE_STRAIGHT:
            if self.speed_state != STATE_STRAIGHT:
                if self.center_stable < CENTER_CONFIRM:
                    self.center_stable += 1
                if self.center_stable >= CENTER_CONFIRM:
                    self.speed_state = STATE_STRAIGHT
        else:
            self.center_stable = 0
            self.speed_state = sensor
        return self.speed_state

    def _patrol_target(self, sensor):
        if self.index >= len(ROUTE) - 1:
            return FINISH_SPEED
        return target_speed_for_state(self._speed_state(sensor))

    def _update_arming(self, sensor):
        if sensor == STATE_STRAIGHT:
            self.center_hold = min(self.center_hold + 1, 255)
            if self.center_hold >= START_CENTER_CYCLES:
                self.armed = 1
            self.junction_count = 0
            return
        if self.index >= len(ROUTE):
            self.junction_count = 0
            return
        trig = ROUTE[self.index][0]
        if matches_trig(sensor, trig):
            if trig == TRIG_CROSS or self.armed or is_confirmed_fork(sensor, trig):
                self.junction_count = min(self.junction_count + 1, 255)
            elif self.junction_count > 0:
                self.junction_count -= 1
            else:
                self.junction_count = 0
            return
        if self.junction_count > 0:
            self.junction_count -= 1
            return
        if not is_left_fork(sensor) and not is_right_fork(sensor):
            self.armed = 0
            self.center_hold = 0

    def _lock(self):
        self.locked_act = ROUTE[self.index][1]
        self.index += 1
        self.junction_count = 0
        self.armed = 0
        self.center_hold = 0
        self.lost_count = 0
        if self.locked_act == ACT_STOP:
            self.run = RUN_FINISH
            self.turn_diff = 0.0
            self.in_place = 0
            return
        self.run = RUN_BRAKE
        self.brake_cycles = 0
        self.turn_cycles = 0
        self.turn_saw_gap = 0
        self.turn_center_count = 0
        self.turn_diff = 0.0
        self.base_speed = 0.0
        self.in_place = 1

    def _enter_search(self):
        self.run = RUN_SEARCH
        self.search_cycles = 0
        self.recover_count = 0
        self.lost_count = 0
        self.junction_count = 0
        self.turn_diff = search_turn(self.last_side)
        self.in_place = 0 if self.last_side == SIDE_CENTER else 1

    def step(self, sensor):
        if sensor != STATE_LOST:
            self.last_side = side_from_sensor(sensor)
            self.lost_count = 0
        else:
            self.lost_count = min(self.lost_count + 1, 255)

        target = LOST_SPEED
        if self.run == RUN_WAIT_START:
            if sensor == STATE_LOST:
                if self.lost_count >= LOST_DEBOUNCE:
                    self._enter_search()
                target = LOST_SPEED
            else:
                self.turn_diff = follow_turn(sensor)
                if sensor == STATE_STRAIGHT:
                    self.center_hold = min(self.center_hold + 1, 255)
                else:
                    self.center_hold = 0
                if sensor != STATE_CROSS and self.center_hold >= START_CENTER_CYCLES:
                    self.run = RUN_PATROL
                    self.armed = 1
                target = target_speed_for_state(self._speed_state(sensor))
        elif self.run == RUN_PATROL:
            if sensor == STATE_LOST:
                if self.lost_count >= LOST_DEBOUNCE:
                    self._enter_search()
                target = LOST_SPEED
            else:
                self.turn_diff = follow_turn(sensor)
                self._update_arming(sensor)
                if self.index < len(ROUTE) and self.junction_count >= needed_debounce(
                    ROUTE[self.index][0]
                ):
                    self._lock()
                    target = 0.0
                else:
                    target = self._patrol_target(sensor)
        elif self.run == RUN_BRAKE:
            self.turn_diff = 0.0
            self.in_place = 1
            self.brake_cycles = min(self.brake_cycles + 1, 65535)
            if self.brake_cycles >= BRAKE_CYCLES:
                self.run = RUN_TURN
                self.turn_cycles = 0
                self.turn_saw_gap = 0
                self.turn_center_count = 0
                self.turn_diff = locked_turn(self.locked_act)
            target = 0.0
        elif self.run == RUN_TURN:
            self.turn_diff = locked_turn(self.locked_act)
            self.turn_cycles = min(self.turn_cycles + 1, 65535)
            if (not both_mids_on_line(sensor)) or sensor == STATE_LOST:
                self.turn_saw_gap = 1
            if self.turn_cycles >= TURN_TIMEOUT_CYCLES:
                if self.locked_act == ACT_LEFT:
                    self.last_side = SIDE_LEFT
                elif self.locked_act == ACT_RIGHT:
                    self.last_side = SIDE_RIGHT
                self._enter_search()
                target = JUNCTION_SPEED
            else:
                if (
                    self.turn_cycles >= TURN_MIN_CYCLES
                    and (self.turn_saw_gap or self.turn_cycles >= TURN_WIDE_TAPE_CYCLES)
                    and caught_new_line(sensor, self.locked_act)
                ):
                    self.turn_center_count = min(self.turn_center_count + 1, 255)
                else:
                    self.turn_center_count = 0
                if self.turn_center_count >= TURN_CENTER_CYCLES:
                    self.run = RUN_PATROL
                    self.turn_cycles = 0
                    self.turn_saw_gap = 0
                    self.turn_center_count = 0
                    self.armed = 0
                    self.center_hold = 0
                    self.in_place = 0
                    self.turn_diff = follow_turn(sensor)
                target = 0.0
        elif self.run == RUN_SEARCH:
            self.turn_diff = search_turn(self.last_side)
            self.search_cycles = min(self.search_cycles + 1, 65535)
            if sensor != STATE_LOST:
                self.recover_count = min(self.recover_count + 1, 255)
            else:
                self.recover_count = 0
            if self.recover_count >= RECOVER_CYCLES:
                self.run = RUN_PATROL
                self.search_cycles = 0
                self.recover_count = 0
                self.in_place = 0
                self.turn_diff = follow_turn(sensor)
                target = self._patrol_target(sensor)
            elif self.search_cycles >= SEARCH_TIMEOUT_CYCLES:
                self.run = RUN_FAULT
                self.turn_diff = 0.0
                self.in_place = 0
                target = 0.0
            else:
                target = LOST_SPEED if self.last_side == SIDE_CENTER else JUNCTION_SPEED
        else:
            self.turn_diff = 0.0
            self.in_place = 0
            target = 0.0

        self.base_speed = ramp_step(self.base_speed, target)
        return self.run, self.index, self.turn_diff, self.base_speed

    def feed(self, sensor, cycles):
        last = None
        for _ in range(cycles):
            last = self.step(sensor)
        return last
