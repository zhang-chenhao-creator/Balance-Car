import unittest
import shutil
import subprocess
import tempfile
import textwrap
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1]
    / "source"
    / "MiniBalance_HARDWARE"
    / "TrackModule"
    / "TrackModule.c"
)
HEADER = SOURCE.with_suffix(".h")


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
        self.assertRegex(text, r"Track_TurnAttackStep\s*=\s*20")
        self.assertRegex(text, r"Track_TurnReleaseStep\s*=\s*10")
        self.assertRegex(text, r"Track_TurnConfirmCycles\s*=\s*2")

    def test_deceleration_is_faster_than_acceleration(self):
        self.assertEqual(ramp_step(400, 350), 390)
        self.assertEqual(ramp_step(325, 400), 330)

    def test_ramp_lands_exactly_on_target_without_overshoot(self):
        value = 400.0
        for _ in range(20):
            value = ramp_step(value, 350.0)
        self.assertEqual(value, 350.0)
        self.assertGreaterEqual(value, 350.0)

    def test_turn_filter_rejects_one_frame_release_and_is_left_right_symmetric(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("gcc is not available for host-side C tests")

        harness = r"""
            #include <assert.h>
            #include "TrackModule.h"

            int fake_port_b[16];
            int fake_port_c[16];

            void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config)
            {
                (void)port;
                (void)config;
            }

            void HAL_GPIO_WritePin(void *port, unsigned int pin, unsigned int value)
            {
                (void)port;
                (void)pin;
                (void)value;
            }

            static void set_state(int state)
            {
                fake_port_c[8] = (state >> 3) & 1;
                fake_port_c[4] = (state >> 2) & 1;
                fake_port_c[9] = (state >> 1) & 1;
                fake_port_b[8] = state & 1;
            }

            static void step(int state)
            {
                set_state(state);
                IRDM_line_inspection();
            }

            int main(void)
            {
                int i;

                TrackModule_Init();
                step(STATE_LEFT_BIG);  assert(turn_diff == 20.0f);
                step(STATE_LEFT_BIG);  assert(turn_diff == 40.0f);
                step(STATE_LEFT_BIG);  assert(turn_diff == 60.0f);
                step(STATE_LEFT_BIG);  assert(turn_diff == 65.0f);

                step(STATE_LEFT_SMALL);
                assert(turn_diff == 65.0f);  /* one-frame release is ignored */
                step(STATE_LEFT_SMALL);
                assert(turn_diff == 55.0f);  /* confirmed release is rate limited */
                step(STATE_LEFT_BIG);
                assert(turn_diff == 65.0f);  /* stronger correction is immediate */

                TrackModule_Init();
                step(STATE_RIGHT_BIG); assert(turn_diff == -20.0f);
                step(STATE_RIGHT_BIG); assert(turn_diff == -40.0f);
                step(STATE_RIGHT_BIG); assert(turn_diff == -60.0f);
                step(STATE_RIGHT_BIG); assert(turn_diff == -65.0f);

                step(STATE_LEFT_BIG);
                assert(turn_diff == -65.0f); /* first reverse frame is ignored */
                step(STATE_LEFT_BIG);
                assert(turn_diff == -55.0f); /* reverse first releases toward zero */
                for (i = 0; i < 6; ++i) step(STATE_LEFT_BIG);
                assert(turn_diff == 0.0f);
                step(STATE_LEFT_BIG);
                assert(turn_diff == 20.0f);
                return 0;
            }
        """

        sys_stub = r"""
            #ifndef __SYS_H
            #define __SYS_H
            #include <stdint.h>
            typedef uint8_t u8;
            typedef uint16_t u16;
            typedef uint32_t u32;
            extern int fake_port_b[16];
            extern int fake_port_c[16];
            #define PBin(n) fake_port_b[(n)]
            #define PCin(n) fake_port_c[(n)]
            typedef struct {
                unsigned int Pin;
                unsigned int Mode;
                unsigned int Pull;
                unsigned int Speed;
            } GPIO_InitTypeDef;
            #define GPIO_PIN_4  (1U << 4)
            #define GPIO_PIN_8  (1U << 8)
            #define GPIO_PIN_9  (1U << 9)
            #define GPIO_MODE_INPUT 0U
            #define GPIO_MODE_OUTPUT_PP 1U
            #define GPIO_PULLDOWN 0U
            #define GPIO_PULLUP 1U
            #define GPIO_NOPULL 2U
            #define GPIO_SPEED_FREQ_LOW 0U
            #define GPIO_PIN_SET 1U
            #define GPIOB ((void *)1)
            #define GPIOC ((void *)2)
            #define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
            #define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
            void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
            void HAL_GPIO_WritePin(void *port, unsigned int pin, unsigned int value);
            #endif
        """

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            (temp / "sys.h").write_text(textwrap.dedent(sys_stub), encoding="utf-8")
            (temp / "TrackModule.h").write_text(HEADER.read_text(encoding="utf-8"), encoding="utf-8")
            (temp / "TrackModule.c").write_text(SOURCE.read_text(encoding="utf-8"), encoding="utf-8")
            (temp / "harness.c").write_text(textwrap.dedent(harness), encoding="utf-8")
            executable = temp / "harness.exe"
            command = [
                compiler,
                "-std=gnu89",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(temp),
                str(temp / "harness.c"),
                str(temp / "TrackModule.c"),
                "-o",
                str(executable),
            ]
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            result = subprocess.run(
                [str(executable)],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            if result.returncode:
                raise AssertionError(f"turn filter harness failed: {result.returncode}")


if __name__ == "__main__":
    unittest.main()
