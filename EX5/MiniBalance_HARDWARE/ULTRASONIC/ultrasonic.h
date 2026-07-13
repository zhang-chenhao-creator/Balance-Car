#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

#define ULTRASONIC_SERVICE_PERIOD_MS  65U
#define ULTRASONIC_ECHO_TIMEOUT_US    30000UL
#define ULTRASONIC_MIN_DISTANCE_MM    20U
#define ULTRASONIC_MAX_DISTANCE_MM    4000U

typedef struct
{
    u32 distance_mm;
    u32 sample_id;
    u8 valid;
    u8 miss_count;
} UltrasonicSnapshot;

void Ultrasonic_Init(void);
void Ultrasonic_Service_65ms(void);
void Ultrasonic_GetSnapshot(UltrasonicSnapshot *snapshot);

#endif
