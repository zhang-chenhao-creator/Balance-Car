import shutil
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "source"
CONTROL = SOURCE / "MiniBalance" / "CONTROL"
AVOID_C = CONTROL / "avoid_routine.c"
AVOID_H = CONTROL / "avoid_routine.h"
GUARD_C = CONTROL / "obstacle_guard.c"
GUARD_H = CONTROL / "obstacle_guard.h"
TRACK = SOURCE / "MiniBalance_HARDWARE" / "TrackModule" / "TrackModule.c"
US_H = SOURCE / "MiniBalance_HARDWARE" / "ULTRASONIC" / "ultrasonic_service.h"


def compile_and_run(harness, include_dirs, sources):
    compiler = shutil.which("gcc")
    if compiler is None:
        raise unittest.SkipTest("gcc is not available for host-side C tests")

    with tempfile.TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        harness_path = temp / "harness.c"
        executable = temp / "harness.exe"
        harness_path.write_text(textwrap.dedent(harness), encoding="utf-8")
        command = [compiler, "-std=c89", "-Wall", "-Wextra", "-Werror"]
        for include_dir in include_dirs:
            command.extend(["-I", str(include_dir)])
        command.append(str(harness_path))
        command.extend(str(source) for source in sources)
        command.extend(["-o", str(executable)])
        compile_result = subprocess.run(command, capture_output=True, text=True)
        if compile_result.returncode:
            raise AssertionError(compile_result.stdout + compile_result.stderr)
        run_result = subprocess.run([str(executable)], capture_output=True, text=True)
        if run_result.returncode:
            raise AssertionError(
                f"host harness failed with exit code {run_result.returncode}"
            )


class ObstacleGuardBehaviorTests(unittest.TestCase):
    def test_guard_uses_three_independent_samples_and_release_hysteresis(self):
        harness = r"""
            #include <assert.h>
            #include <stdint.h>
            #include "obstacle_guard.h"

            static ObstacleGuardOutput step(ObstacleGuardContext *context,
                                            uint32_t sample_id,
                                            uint8_t valid,
                                            uint32_t distance,
                                            uint8_t misses)
            {
                ObstacleGuardInput input;
                input.requested_speed_mm_s = 400;
                input.measured_forward_speed_mm_s = 400;
                input.distance_mm = distance;
                input.sample_id = sample_id;
                input.sample_valid = valid;
                input.miss_count = misses;
                return ObstacleGuard_Update(context, &input);
            }

            int main(void)
            {
                ObstacleGuardContext context;
                ObstacleGuardOutput output;

                ObstacleGuard_Init(&context, 1);
                output = step(&context, 1, 1, 250, 0);
                assert(output.state == OBSTACLE_GUARD_SLOW);
                assert(output.allowed_speed_mm_s == OBSTACLE_GUARD_CONFIRM_MAX_MM_S);
                assert(context.blocked_samples == 1);

                output = step(&context, 1, 1, 250, 0);
                output = step(&context, 1, 1, 250, 0);
                assert(context.blocked_samples == 1);
                assert(output.entered_blocked == 0);

                output = step(&context, 2, 1, 250, 0);
                assert(context.blocked_samples == 2);
                assert(output.state != OBSTACLE_GUARD_BLOCKED);

                output = step(&context, 3, 1, 250, 0);
                assert(output.state == OBSTACLE_GUARD_BLOCKED);
                assert(output.entered_blocked == 1);
                assert(output.allowed_speed_mm_s == 0);

                output = step(&context, 3, 1, 250, 0);
                assert(output.entered_blocked == 0);
                output = step(&context, 4, 1, 500, 0);
                assert(output.state == OBSTACLE_GUARD_BLOCKED);
                output = step(&context, 5, 1, 500, 0);
                assert(output.state == OBSTACLE_GUARD_BLOCKED);
                output = step(&context, 6, 1, 500, 0);
                assert(output.state == OBSTACLE_GUARD_CLEAR);
                return 0;
            }
        """
        compile_and_run(harness, [CONTROL], [GUARD_C])

    def test_invalid_new_sample_resets_candidate_and_misses_degrade_speed(self):
        harness = r"""
            #include <assert.h>
            #include <stdint.h>
            #include "obstacle_guard.h"

            static ObstacleGuardOutput step(ObstacleGuardContext *context,
                                            uint32_t sample_id,
                                            uint8_t valid,
                                            uint32_t distance,
                                            uint8_t misses)
            {
                ObstacleGuardInput input;
                input.requested_speed_mm_s = 400;
                input.measured_forward_speed_mm_s = 300;
                input.distance_mm = distance;
                input.sample_id = sample_id;
                input.sample_valid = valid;
                input.miss_count = misses;
                return ObstacleGuard_Update(context, &input);
            }

            int main(void)
            {
                ObstacleGuardContext context;
                ObstacleGuardOutput output;

                ObstacleGuard_Init(&context, 1);
                step(&context, 1, 1, 250, 0);
                assert(context.blocked_samples == 1);
                step(&context, 2, 0, 250, 1);
                assert(context.blocked_samples == 0);
                step(&context, 3, 1, 250, 0);
                step(&context, 4, 1, 250, 0);
                assert(context.blocked_samples == 2);

                output = step(&context, 5, 0, 250, 3);
                assert(output.state == OBSTACLE_GUARD_DEGRADED);
                assert(output.allowed_speed_mm_s == OBSTACLE_GUARD_DEGRADED_MAX_MM_S);
                assert(context.blocked_samples == 0);
                return 0;
            }
        """
        compile_and_run(harness, [CONTROL], [GUARD_C])


class AvoidRoutineBehaviorTests(unittest.TestCase):
    def _build_stubs(self, directory):
        stubs = {
            "sys.h": """
                #ifndef __SYS_H
                #define __SYS_H
                #include <stdint.h>
                typedef uint8_t u8;
                typedef uint16_t u16;
                typedef uint32_t u32;
                #endif
            """,
            "ultrasonic_service.h": """
                #ifndef __ULTRASONIC_SERVICE_H
                #define __ULTRASONIC_SERVICE_H
                #include "sys.h"
                typedef struct { u32 distance_mm; u32 sample_id; u8 valid; u8 miss_count; } UsSnapshot;
                void UltrasonicService_Init(void);
                void UltrasonicService_Update5ms(void);
                void UltrasonicService_GetSnapshot(UsSnapshot *snap);
                #endif
            """,
            "TrackModule.h": """
                #ifndef __TRACKMODULE_H
                #define __TRACKMODULE_H
                #include "sys.h"
                #define STATE_STRAIGHT 0x09
                #define STATE_LEFT_SMALL 0x0B
                #define STATE_RIGHT_SMALL 0x0D
                #define STATE_LOST 0x0F
                extern float base_speed_mm;
                extern float turn_diff;
                extern float Track_Turn_Scale;
                extern u8 Track_state;
                #endif
            """,
            "control.h": """
                #ifndef __CONTROL_H
                #define __CONTROL_H
                #define EncoderMultiples 4.0
                #define Encoder_precision 500.0
                #define Reduction_Ratio 30.0
                #define Perimeter 210.4867
                extern float Velocity_Left;
                extern float Velocity_Right;
                #endif
            """,
            "show.h": """
                #ifndef __SHOW_H
                #define __SHOW_H
                #endif
            """,
        }
        for name, content in stubs.items():
            (directory / name).write_text(textwrap.dedent(content), encoding="utf-8")

    def _compile_route_harness(self, body):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("gcc is not available for host-side C tests")

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            self._build_stubs(temp)
            route_source = temp / "avoid_routine.c"
            route_header = temp / "avoid_routine.h"
            route_source.write_text(AVOID_C.read_text(encoding="utf-8"), encoding="utf-8")
            route_header.write_text(AVOID_H.read_text(encoding="utf-8"), encoding="utf-8")
            harness = temp / "route_harness.c"
            executable = temp / "route_harness.exe"
            harness.write_text(textwrap.dedent(body), encoding="utf-8")
            command = [
                compiler,
                "-std=c89",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(temp),
                "-I",
                str(CONTROL),
                str(harness),
                str(route_source),
                str(GUARD_C),
                "-o",
                str(executable),
            ]
            compile_result = subprocess.run(command, capture_output=True, text=True)
            if compile_result.returncode:
                raise AssertionError(compile_result.stdout + compile_result.stderr)
            run_result = subprocess.run([str(executable)], capture_output=True, text=True)
            if run_result.returncode:
                raise AssertionError(
                    f"route harness failed with exit code {run_result.returncode}"
                )

    def test_complete_double_polyline_route_and_line_reentry(self):
        harness = r"""
            #include <assert.h>
            #include "avoid_routine.h"
            #include "ultrasonic_service.h"
            #include "TrackModule.h"

            int Encoder_Left = 0, Encoder_Right = 0;
            float Gyro_Turn = 0;
            u8 Lidar_Detect = 1;
            float Velocity_Left = 0, Velocity_Right = 0;
            float base_speed_mm = 400;
            float turn_diff = 0;
            float Track_Turn_Scale = 0.7f;
            u8 Track_state = STATE_LOST;
            static UsSnapshot snapshot;

            void UltrasonicService_Init(void) { snapshot.sample_id = 0; snapshot.valid = 0; snapshot.miss_count = 0; }
            void UltrasonicService_Update5ms(void) { }
            void UltrasonicService_GetSnapshot(UsSnapshot *snap) { *snap = snapshot; }
            static void tick(void) { TrackAvoid_Supervisor5ms(); }
            static void sample(u32 distance) { snapshot.sample_id++; snapshot.valid = 1; snapshot.distance_mm = distance; tick(); }

            int main(void)
            {
                int i;
                TrackAvoid_Init();
                for (i = 0; i < 80; ++i) tick();
                assert(Patrol_Speed_Cmd == 400.0f);

                sample(250); sample(250);
                assert(Avoid_State == AVOID_IDLE);
                sample(250);
                assert(Avoid_State == AVOID_BRAKE);
                assert(Avoid_Active == 1);

                for (i = 0; i < 80 && Avoid_State == AVOID_BRAKE; ++i) tick();
                assert(Avoid_State == AVOID_STOP_SETTLE);
                for (i = 0; i < 80; ++i) tick();
                assert(Avoid_State == AVOID_ARC_RIGHT);

                snapshot.sample_id++; snapshot.distance_mm = 1000;
                Gyro_Turn = 1476.0f;
                for (i = 0; i < 120 && Avoid_State == AVOID_ARC_RIGHT; ++i) tick();
                assert(Avoid_State == AVOID_DIAGONAL);
                assert(Patrol_Turn_Cmd >= 0.0f);

                Gyro_Turn = 0;
                Encoder_Left = 171; Encoder_Right = 171;
                for (i = 0; i < 700 && Avoid_State == AVOID_DIAGONAL; ++i) tick();
                assert(Avoid_State == AVOID_ARC_LEFT);

                Encoder_Left = 0; Encoder_Right = 0;
                Gyro_Turn = 1476.0f;
                tick();
                assert(Patrol_Turn_Cmd < 0.0f);
                for (i = 0; i < 220 && Avoid_State == AVOID_ARC_LEFT; ++i) tick();
                assert(Avoid_State == AVOID_SEARCH_LINE);

                Gyro_Turn = 0;
                Track_state = STATE_LEFT_SMALL;
                tick(); tick();
                assert(Avoid_State == AVOID_SEARCH_LINE);
                tick();
                assert(Avoid_State == AVOID_REENTER_LINE);

                Track_state = STATE_STRAIGHT;
                for (i = 0; i < 50 && Avoid_Active; ++i) tick();
                assert(Avoid_State == AVOID_IDLE);
                assert(Avoid_Active == 0);
                return 0;
            }
        """
        self._compile_route_harness(harness)

    def test_second_obstacle_timeout_and_cumulative_search_limit_enter_safe_hold(self):
        harness = r"""
            #include <assert.h>
            #include "avoid_routine.h"
            #include "ultrasonic_service.h"
            #include "TrackModule.h"

            int Encoder_Left = 0, Encoder_Right = 0;
            float Gyro_Turn = 0;
            u8 Lidar_Detect = 1;
            float Velocity_Left = 0, Velocity_Right = 0;
            float base_speed_mm = 400;
            float turn_diff = 0;
            float Track_Turn_Scale = 0.7f;
            u8 Track_state = STATE_LOST;
            static UsSnapshot snapshot;

            void UltrasonicService_Init(void) { snapshot.sample_id = 0; snapshot.valid = 0; snapshot.miss_count = 0; }
            void UltrasonicService_Update5ms(void) { }
            void UltrasonicService_GetSnapshot(UsSnapshot *snap) { *snap = snapshot; }
            static void tick(void) { TrackAvoid_Supervisor5ms(); }
            static void sample(u32 distance) { snapshot.sample_id++; snapshot.valid = 1; snapshot.distance_mm = distance; tick(); }

            int main(void)
            {
                int i;
                TrackAvoid_Init();
                for (i = 0; i < 80; ++i) tick();
                sample(250); sample(250); sample(250);
                for (i = 0; i < 80 && Avoid_State == AVOID_BRAKE; ++i) tick();
                for (i = 0; i < 80; ++i) tick();
                Gyro_Turn = 1476.0f;
                for (i = 0; i < 120 && Avoid_State == AVOID_ARC_RIGHT; ++i) tick();
                assert(Avoid_State == AVOID_DIAGONAL);

                Gyro_Turn = 0;
                sample(200); sample(200);
                assert(Avoid_State == AVOID_DIAGONAL);
                sample(200);
                assert(Avoid_State == AVOID_ABORT_HOLD);
                for (i = 0; i < 20; ++i) tick();
                assert(Patrol_Speed_Cmd == 0.0f);
                assert(Avoid_Active == 1);

                Lidar_Detect = 0; tick();
                assert(Avoid_State == AVOID_IDLE);
                Lidar_Detect = 1; tick();
                sample(250); sample(250); sample(250);
                for (i = 0; i < 220 && Avoid_State == AVOID_BRAKE; ++i) tick();
                assert(Avoid_State == AVOID_STOP_SETTLE);
                for (i = 0; i < 80; ++i) tick();
                assert(Avoid_State == AVOID_ARC_RIGHT);
                Gyro_Turn = 0;
                for (i = 0; i < 1001 && Avoid_State == AVOID_ARC_RIGHT; ++i) tick();
                assert(Avoid_State == AVOID_ABORT_HOLD);

                /* SEARCH_LINE and REENTER_LINE share one cumulative distance
                   limit, so intermittent line hits cannot reset the 800 mm cap. */
                Lidar_Detect = 0; tick();
                assert(Avoid_State == AVOID_IDLE);
                Lidar_Detect = 1; tick();
                Encoder_Left = 0; Encoder_Right = 0;
                sample(250); sample(250); sample(250);
                for (i = 0; i < 220 && Avoid_State == AVOID_BRAKE; ++i) tick();
                assert(Avoid_State == AVOID_STOP_SETTLE);
                for (i = 0; i < 80; ++i) tick();
                assert(Avoid_State == AVOID_ARC_RIGHT);

                snapshot.sample_id++; snapshot.distance_mm = 1000;
                Gyro_Turn = 1476.0f;
                for (i = 0; i < 120 && Avoid_State == AVOID_ARC_RIGHT; ++i) tick();
                assert(Avoid_State == AVOID_DIAGONAL);
                Gyro_Turn = 0;
                Encoder_Left = 171; Encoder_Right = 171;
                for (i = 0; i < 700 && Avoid_State == AVOID_DIAGONAL; ++i) tick();
                assert(Avoid_State == AVOID_ARC_LEFT);
                Encoder_Left = 0; Encoder_Right = 0;
                Gyro_Turn = 1476.0f;
                for (i = 0; i < 220 && Avoid_State == AVOID_ARC_LEFT; ++i) tick();
                assert(Avoid_State == AVOID_SEARCH_LINE);

                Gyro_Turn = 0;
                Encoder_Left = 30000; Encoder_Right = 30000;
                Track_state = STATE_LEFT_SMALL;
                tick(); tick(); tick();
                assert(Avoid_State == AVOID_REENTER_LINE);
                Track_state = STATE_LOST;
                tick();
                assert(Avoid_State == AVOID_SEARCH_LINE);
                Track_state = STATE_LEFT_SMALL;
                tick(); tick(); tick();
                assert(Avoid_State == AVOID_REENTER_LINE);
                Track_state = STATE_LOST;
                tick();
                assert(Avoid_State == AVOID_ABORT_HOLD);
                return 0;
            }
        """
        self._compile_route_harness(harness)


class SourceIntegrationTests(unittest.TestCase):
    def test_double_polyline_states_and_tunable_defaults_are_present(self):
        header = AVOID_H.read_text(encoding="utf-8")
        source = AVOID_C.read_text(encoding="utf-8")
        for name in (
            "AVOID_BRAKE",
            "AVOID_STOP_SETTLE",
            "AVOID_ARC_RIGHT",
            "AVOID_DIAGONAL",
            "AVOID_ARC_LEFT",
            "AVOID_SEARCH_LINE",
            "AVOID_REENTER_LINE",
            "AVOID_ABORT_HOLD",
        ):
            self.assertIn(name, header)
        self.assertIn("Avoid_RightAngleDeg = 45.0f", source)
        self.assertIn("Avoid_LeftAngleDeg  = 90.0f", source)
        self.assertIn("Avoid_DiagonalMm    = 300.0f", source)

    def test_ultrasonic_period_and_stable_line_follow_are_preserved(self):
        ultrasonic = US_H.read_text(encoding="utf-8")
        track = TRACK.read_text(encoding="utf-8")
        self.assertIn("US_SERVICE_PERIOD_5MS     13U", ultrasonic)
        self.assertIn("Track_Speed_Ramp", track)
        self.assertNotIn("Track_Route", track)
        self.assertNotIn("RUN_TURN", track)

if __name__ == "__main__":
    unittest.main()
