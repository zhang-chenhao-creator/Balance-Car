#include "ultrasonic_service.h"
#include "delay.h"
#include "tim.h"

extern u16 TIM2CH2_CAPTURE_STA;
extern u16 TIM2CH2_CAPTURE_VAL;
extern u32 Distance;

#define US_TRIG_HIGH()  PCout(15) = 1
#define US_TRIG_LOW()   PCout(15) = 0

static u8  s_period_count;
static u32 s_sample_id;
static u8  s_miss_count;
static volatile UsSnapshot s_snapshots[2];
static volatile u8 s_active_snapshot;

static void UltrasonicService_Trigger(void)
{
    US_TRIG_HIGH();
    delay_us(15);
    US_TRIG_LOW();
}

void UltrasonicService_Init(void)
{
    s_period_count = 0;
    s_sample_id = 0;
    s_miss_count = 0;
    s_snapshots[0].distance_mm = 0;
    s_snapshots[0].sample_id = 0;
    s_snapshots[0].valid = 0;
    s_snapshots[0].miss_count = 0;
    s_snapshots[1] = s_snapshots[0];
    s_active_snapshot = 0;
    TIM2CH2_CAPTURE_STA = 0;
    UltrasonicService_Trigger();
}

static void UltrasonicService_Service(void)
{
    u16 sta, val;
    u32 us, mm = 0;
    u8  valid = 0;
    u8  next;

    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    sta = TIM2CH2_CAPTURE_STA;
    val = TIM2CH2_CAPTURE_VAL;
    if (sta & 0x80)
        TIM2CH2_CAPTURE_STA = 0;
    else if (sta != 0)
        TIM2CH2_CAPTURE_STA = 0;
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    if (sta & 0x80)
    {
        us = (u32)(sta & 0x3F) * 65536UL + val;
        mm = us * 170UL / 1000UL;
        if (mm >= US_MIN_DISTANCE_MM && mm <= US_MAX_DISTANCE_MM)
            valid = 1;
    }

    if (valid)
        s_miss_count = 0;
    else if (s_miss_count < 255U)
        s_miss_count++;

    next = s_active_snapshot ^ 1U;
    s_snapshots[next].distance_mm = valid ? mm : s_snapshots[s_active_snapshot].distance_mm;
    s_snapshots[next].sample_id = ++s_sample_id;
    s_snapshots[next].valid = valid;
    s_snapshots[next].miss_count = s_miss_count;
    s_active_snapshot = next;

    if (valid)
        Distance = mm;

    UltrasonicService_Trigger();
}

void UltrasonicService_Update5ms(void)
{
    if (++s_period_count >= US_SERVICE_PERIOD_5MS)
    {
        s_period_count = 0;
        UltrasonicService_Service();
    }
}

void UltrasonicService_GetSnapshot(UsSnapshot *snap)
{
    u8 index = s_active_snapshot;
    snap->distance_mm = s_snapshots[index].distance_mm;
    snap->sample_id = s_snapshots[index].sample_id;
    snap->valid = s_snapshots[index].valid;
    snap->miss_count = s_snapshots[index].miss_count;
}
