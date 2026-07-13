#include <stdio.h>
#include <stdlib.h>
#include "obstacle_guard.h"

static int failures;

static void expect_int(const char *name, long actual, long expected)
{
    if (actual != expected)
    {
        printf("FAIL %-32s actual=%ld expected=%ld\n", name, actual, expected);
        failures++;
    }
}

static ObstacleGuardOutput update(ObstacleGuardContext *context,
                                  long requested, long measured, unsigned long distance,
                                  unsigned long sample_id, unsigned valid, unsigned misses)
{
    ObstacleGuardInput input;
    input.requested_speed_mm_s = requested;
    input.measured_forward_speed_mm_s = measured;
    input.distance_mm = distance;
    input.sample_id = sample_id;
    input.sample_valid = (uint8_t)valid;
    input.miss_count = (uint8_t)misses;
    return ObstacleGuard_Update(context, &input);
}

int main(void)
{
    ObstacleGuardContext context;
    ObstacleGuardOutput output;
    unsigned long sample = 1;

    ObstacleGuard_Init(&context, 1);
    output = update(&context, 100, 0, 500, sample++, 1, 0);
    expect_int("100 mm/s stop threshold", output.stop_mm, 138);
    expect_int("100 mm/s slow threshold", output.slow_mm, 318);

    output = update(&context, 300, 0, 500, sample++, 1, 0);
    expect_int("300 mm/s stop threshold", output.stop_mm, 174);
    expect_int("300 mm/s slow threshold", output.slow_mm, 354);

    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("600 mm/s stop threshold", output.stop_mm, 228);
    expect_int("600 mm/s slow threshold", output.slow_mm, 408);

    output = update(&context, 100, 600, 500, sample++, 1, 0);
    expect_int("measured speed raises threshold", output.stop_mm, 228);

    output = update(&context, 600, 0, 318, sample++, 1, 0);
    expect_int("slow state", output.state, OBSTACLE_GUARD_SLOW);
    expect_int("linear speed limit", output.allowed_speed_mm_s, 300);

    output = update(&context, 600, 0, 220, sample++, 1, 0);
    expect_int("blocked state", output.state, OBSTACLE_GUARD_BLOCKED);
    expect_int("blocked speed", output.allowed_speed_mm_s, 0);
    expect_int("blocked transition", output.entered_blocked, 1);

    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("release sample 1 remains blocked", output.state, OBSTACLE_GUARD_BLOCKED);
    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("release sample 2 remains blocked", output.state, OBSTACLE_GUARD_BLOCKED);
    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("release sample 3 clears", output.state, OBSTACLE_GUARD_CLEAR);

    output = update(&context, -300, 0, 100, sample++, 1, 0);
    expect_int("reverse is never limited", output.allowed_speed_mm_s, -300);

    ObstacleGuard_Init(&context, 1);
    output = update(&context, 300, 0, 100, sample++, 1, 0);
    expect_int("second scenario blocks", output.state, OBSTACLE_GUARD_BLOCKED);
    update(&context, 300, 0, 100, sample++, 0, 1);
    update(&context, 300, 0, 100, sample++, 0, 2);
    output = update(&context, 300, 0, 100, sample++, 0, 3);
    expect_int("misses cannot release block", output.state, OBSTACLE_GUARD_BLOCKED);
    expect_int("blocked beats degraded cap", output.allowed_speed_mm_s, 0);

    ObstacleGuard_Init(&context, 1);
    update(&context, 600, 0, 500, sample++, 0, 1);
    update(&context, 600, 0, 500, sample++, 0, 2);
    output = update(&context, 600, 0, 500, sample++, 0, 3);
    expect_int("three misses degrade", output.state, OBSTACLE_GUARD_DEGRADED);
    expect_int("degraded cap", output.allowed_speed_mm_s, 100);
    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("one valid remains degraded", output.state, OBSTACLE_GUARD_DEGRADED);
    output = update(&context, 600, 0, 500, sample++, 1, 0);
    expect_int("two valid recover", output.state, OBSTACLE_GUARD_CLEAR);

    ObstacleGuard_SetEnabled(&context, 0);
    output = update(&context, 600, 0, 50, sample++, 1, 0);
    expect_int("disabled bypass", output.allowed_speed_mm_s, 600);
    expect_int("disabled state", output.state, OBSTACLE_GUARD_DISABLED);

    if (failures)
    {
        printf("%d obstacle guard test(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("All obstacle guard tests passed\n");
    return EXIT_SUCCESS;
}
