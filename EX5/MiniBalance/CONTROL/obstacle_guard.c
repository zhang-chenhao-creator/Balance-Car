#include "obstacle_guard.h"

static int32_t Clamp_Int32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static uint16_t Stop_Distance_From_Speed(int32_t requested_speed,
                                         int32_t measured_forward_speed)
{
    int32_t reference_speed = requested_speed;
    int32_t stop_mm;

    if (reference_speed < measured_forward_speed)
        reference_speed = measured_forward_speed;
    reference_speed = Clamp_Int32(reference_speed, 0,
                                  OBSTACLE_GUARD_REFERENCE_MAX_MM_S);

    stop_mm = (int32_t)OBSTACLE_GUARD_STOP_BASE_MM +
              reference_speed * (int32_t)OBSTACLE_GUARD_REACTION_MS / 1000;
    stop_mm = Clamp_Int32(stop_mm,
                         (int32_t)OBSTACLE_GUARD_STOP_BASE_MM,
                         (int32_t)OBSTACLE_GUARD_STOP_MAX_MM);
    return (uint16_t)stop_mm;
}

void ObstacleGuard_Init(ObstacleGuardContext *context, uint8_t enabled)
{
    context->last_sample_id = 0;
    context->last_distance_mm = 0;
    context->enabled = enabled ? 1U : 0U;
    context->has_valid_distance = 0;
    context->blocked_latched = 0;
    context->degraded_latched = 0;
    context->release_samples = 0;
    context->recovery_samples = 0;
    context->previous_state = enabled ? OBSTACLE_GUARD_CLEAR : OBSTACLE_GUARD_DISABLED;
}

void ObstacleGuard_SetEnabled(ObstacleGuardContext *context, uint8_t enabled)
{
    context->enabled = enabled ? 1U : 0U;
    context->blocked_latched = 0;
    context->degraded_latched = 0;
    context->release_samples = 0;
    context->recovery_samples = 0;
    context->previous_state = enabled ? OBSTACLE_GUARD_CLEAR : OBSTACLE_GUARD_DISABLED;
}

ObstacleGuardOutput ObstacleGuard_Update(ObstacleGuardContext *context,
                                         const ObstacleGuardInput *input)
{
    ObstacleGuardOutput output;
    uint16_t release_mm;
    uint32_t control_distance;
    uint8_t new_sample;
    uint8_t state;

    output.requested_speed_mm_s = input->requested_speed_mm_s;
    output.stop_mm = Stop_Distance_From_Speed(input->requested_speed_mm_s,
                                              input->measured_forward_speed_mm_s);
    output.slow_mm = output.stop_mm + OBSTACLE_GUARD_SLOW_MARGIN_MM;
    release_mm = output.stop_mm + OBSTACLE_GUARD_RELEASE_MARGIN_MM;
    output.entered_blocked = 0;

    if (!context->enabled)
    {
        output.allowed_speed_mm_s = input->requested_speed_mm_s;
        output.state = OBSTACLE_GUARD_DISABLED;
        context->previous_state = output.state;
        return output;
    }

    new_sample = (input->sample_id != context->last_sample_id) ? 1U : 0U;
    if (new_sample)
    {
        context->last_sample_id = input->sample_id;

        if (input->sample_valid)
        {
            context->last_distance_mm = input->distance_mm;
            context->has_valid_distance = 1;

            if (input->distance_mm <= output.stop_mm)
            {
                context->blocked_latched = 1;
                context->release_samples = 0;
            }
            else if (context->blocked_latched)
            {
                if (input->distance_mm >= release_mm)
                {
                    if (context->release_samples < OBSTACLE_GUARD_RELEASE_SAMPLES)
                        context->release_samples++;
                    if (context->release_samples >= OBSTACLE_GUARD_RELEASE_SAMPLES)
                        context->blocked_latched = 0;
                }
                else
                {
                    context->release_samples = 0;
                }
            }

            if (context->degraded_latched)
            {
                if (input->distance_mm > output.stop_mm)
                {
                    if (context->recovery_samples < OBSTACLE_GUARD_RECOVERY_SAMPLES)
                        context->recovery_samples++;
                    if (context->recovery_samples >= OBSTACLE_GUARD_RECOVERY_SAMPLES)
                        context->degraded_latched = 0;
                }
                else
                {
                    context->recovery_samples = 0;
                }
            }
        }
        else
        {
            context->release_samples = 0;
            context->recovery_samples = 0;
            if (input->miss_count >= OBSTACLE_GUARD_MISS_LIMIT)
                context->degraded_latched = 1;
        }
    }

    if (context->blocked_latched)
        state = OBSTACLE_GUARD_BLOCKED;
    else if (context->degraded_latched)
        state = OBSTACLE_GUARD_DEGRADED;
    else
    {
        control_distance = input->sample_valid ? input->distance_mm : context->last_distance_mm;
        if (context->has_valid_distance && control_distance < output.slow_mm)
            state = OBSTACLE_GUARD_SLOW;
        else
            state = OBSTACLE_GUARD_CLEAR;
    }

    output.state = state;
    output.allowed_speed_mm_s = input->requested_speed_mm_s;

    if (input->requested_speed_mm_s > 0)
    {
        if (state == OBSTACLE_GUARD_BLOCKED)
        {
            output.allowed_speed_mm_s = 0;
        }
        else if (state == OBSTACLE_GUARD_DEGRADED)
        {
            output.allowed_speed_mm_s = Clamp_Int32(input->requested_speed_mm_s,
                                                    0,
                                                    OBSTACLE_GUARD_DEGRADED_MAX_MM_S);
        }
        else if (state == OBSTACLE_GUARD_SLOW)
        {
            control_distance = input->sample_valid ? input->distance_mm : context->last_distance_mm;
            if (control_distance <= output.stop_mm)
                output.allowed_speed_mm_s = 0;
            else
                output.allowed_speed_mm_s = input->requested_speed_mm_s *
                    (int32_t)(control_distance - output.stop_mm) /
                    (int32_t)OBSTACLE_GUARD_SLOW_MARGIN_MM;
        }
    }

    if (state == OBSTACLE_GUARD_BLOCKED &&
        context->previous_state != OBSTACLE_GUARD_BLOCKED)
        output.entered_blocked = 1;

    context->previous_state = state;
    return output;
}
