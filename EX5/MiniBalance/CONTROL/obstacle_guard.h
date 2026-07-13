#ifndef __OBSTACLE_GUARD_H
#define __OBSTACLE_GUARD_H

#include <stdint.h>

#define OBSTACLE_GUARD_STOP_BASE_MM          120U
#define OBSTACLE_GUARD_REACTION_MS           180U
#define OBSTACLE_GUARD_STOP_MAX_MM           280U
#define OBSTACLE_GUARD_SLOW_MARGIN_MM         180U
#define OBSTACLE_GUARD_RELEASE_MARGIN_MM       60U
#define OBSTACLE_GUARD_REFERENCE_MAX_MM_S     800
#define OBSTACLE_GUARD_DEGRADED_MAX_MM_S      100
#define OBSTACLE_GUARD_MISS_LIMIT                3U
#define OBSTACLE_GUARD_RELEASE_SAMPLES           3U
#define OBSTACLE_GUARD_RECOVERY_SAMPLES          2U

typedef enum
{
    OBSTACLE_GUARD_DISABLED = 0,
    OBSTACLE_GUARD_CLEAR,
    OBSTACLE_GUARD_SLOW,
    OBSTACLE_GUARD_BLOCKED,
    OBSTACLE_GUARD_DEGRADED
} ObstacleGuardState;

typedef struct
{
    int32_t requested_speed_mm_s;
    int32_t measured_forward_speed_mm_s;
    uint32_t distance_mm;
    uint32_t sample_id;
    uint8_t sample_valid;
    uint8_t miss_count;
} ObstacleGuardInput;

typedef struct
{
    int32_t requested_speed_mm_s;
    int32_t allowed_speed_mm_s;
    uint16_t stop_mm;
    uint16_t slow_mm;
    uint8_t state;
    uint8_t entered_blocked;
} ObstacleGuardOutput;

typedef struct
{
    uint32_t last_sample_id;
    uint32_t last_distance_mm;
    uint8_t enabled;
    uint8_t has_valid_distance;
    uint8_t blocked_latched;
    uint8_t degraded_latched;
    uint8_t release_samples;
    uint8_t recovery_samples;
    uint8_t previous_state;
} ObstacleGuardContext;

void ObstacleGuard_Init(ObstacleGuardContext *context, uint8_t enabled);
void ObstacleGuard_SetEnabled(ObstacleGuardContext *context, uint8_t enabled);
ObstacleGuardOutput ObstacleGuard_Update(ObstacleGuardContext *context,
                                         const ObstacleGuardInput *input);

#endif
