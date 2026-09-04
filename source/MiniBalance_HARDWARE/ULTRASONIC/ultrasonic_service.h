/***********************************************
 * 超声波测距服务
 * TRIG=PC15, ECHO=PA1/TIM2_CH2
 * 65ms 周期：HC-SR04 要求相邻 Trig >= 60ms。
 ***********************************************/
#ifndef __ULTRASONIC_SERVICE_H
#define __ULTRASONIC_SERVICE_H

#include "sys.h"

#define US_SERVICE_PERIOD_5MS     13U
#define US_MIN_DISTANCE_MM        20U
#define US_MAX_DISTANCE_MM        4000U

typedef struct
{
    u32 distance_mm;
    u32 sample_id;
    u8  valid;
    u8  miss_count;
} UsSnapshot;

void UltrasonicService_Init(void);
void UltrasonicService_Update5ms(void);
void UltrasonicService_GetSnapshot(UsSnapshot *snap);

#endif
