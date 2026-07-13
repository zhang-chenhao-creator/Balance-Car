#include "ultrasonic.h"
#include "sys.h"
#include "delay.h"

#define ULTRASONIC_TIMER          TIM2
#define ULTRASONIC_ECHO_PORT      GPIOA
#define ULTRASONIC_ECHO_PIN       GPIO_Pin_1
#define ULTRASONIC_TRIG_PORT      GPIOC
#define ULTRASONIC_TRIG_PIN       GPIO_Pin_15

#define ULTRASONIC_WAIT_RISE      1U
#define ULTRASONIC_WAIT_FALL      2U

static volatile u8 measurement_state;
static volatile u8 capture_ready;
static volatile u8 overflow_count;
static volatile u32 captured_ticks;

static volatile UltrasonicSnapshot snapshots[2];
static volatile u8 active_snapshot;
static u32 next_sample_id;
static u8 consecutive_misses;

static void Ultrasonic_Select_Rising_Edge(void)
{
    TIM2->CCER &= (u16)(~TIM_CCER_CC2P);
}

static void Ultrasonic_Select_Falling_Edge(void)
{
    TIM2->CCER |= TIM_CCER_CC2P;
}

static void Ultrasonic_Trigger(void)
{
    capture_ready = 0;
    captured_ticks = 0;
    overflow_count = 0;
    measurement_state = ULTRASONIC_WAIT_RISE;
    TIM_SetCounter(ULTRASONIC_TIMER, 0);
    TIM_SetCompare1(ULTRASONIC_TIMER, (u16)ULTRASONIC_ECHO_TIMEOUT_US);
    TIM_ClearITPendingBit(ULTRASONIC_TIMER, TIM_IT_CC1);
    Ultrasonic_Select_Rising_Edge();

    GPIO_SetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    delay_us(15);
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
}

static void Ultrasonic_Publish(u32 distance_mm, u8 valid)
{
    u8 next = active_snapshot ^ 1U;

    if (valid)
        consecutive_misses = 0;
    else if (consecutive_misses < 255U)
        consecutive_misses++;

    snapshots[next].distance_mm = valid ? distance_mm : Distance;
    snapshots[next].sample_id = ++next_sample_id;
    snapshots[next].valid = valid;
    snapshots[next].miss_count = consecutive_misses;

    /* Commit the complete sample with one atomic byte write. */
    active_snapshot = next;

    if (valid)
        Distance = distance_mm;
    Ultrasonic_Valid = valid;
    Ultrasonic_Miss_Count = consecutive_misses;
}

void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_ICInitTypeDef ic;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = ULTRASONIC_ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_ECHO_PORT, &gpio);

    gpio.GPIO_Pin = ULTRASONIC_TRIG_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_TRIG_PORT, &gpio);
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);

    timer.TIM_Period = 0xFFFF;
    timer.TIM_Prescaler = 71;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ULTRASONIC_TIMER, &timer);

    ic.TIM_Channel = TIM_Channel_2;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0x03;
    TIM_ICInit(ULTRASONIC_TIMER, &ic);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(ULTRASONIC_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
    TIM_Cmd(ULTRASONIC_TIMER, ENABLE);

    measurement_state = 0;
    capture_ready = 0;
    overflow_count = 0;
    captured_ticks = 0;
    active_snapshot = 0;
    next_sample_id = 0;
    consecutive_misses = 0;
    snapshots[0].distance_mm = 0;
    snapshots[0].sample_id = 0;
    snapshots[0].valid = 0;
    snapshots[0].miss_count = 0;
    snapshots[1] = snapshots[0];
    Distance = 0;
    Ultrasonic_Valid = 0;
    Ultrasonic_Miss_Count = 0;

    /* Start the first ping immediately; subsequent pings are 65 ms apart. */
    Ultrasonic_Trigger();
}

void Ultrasonic_Service_65ms(void)
{
    u32 ticks = 0;
    u32 distance_mm = 0;
    u8 ready;
    u8 valid = 0;

    /* TIM2 can preempt the main loop. Take and retire the previous result as
       one short transaction so a boundary capture cannot be lost. */
    NVIC_DisableIRQ(TIM2_IRQn);
    ready = capture_ready;
    if (ready)
    {
        ticks = captured_ticks;
        capture_ready = 0;
    }
    measurement_state = 0;
    Ultrasonic_Select_Rising_Edge();
    NVIC_EnableIRQ(TIM2_IRQn);

    if (ready)
    {
        if (ticks > 0 && ticks < ULTRASONIC_ECHO_TIMEOUT_US)
        {
            distance_mm = ticks * 17UL / 100UL;
            if (distance_mm < ULTRASONIC_MIN_DISTANCE_MM)
                distance_mm = ULTRASONIC_MIN_DISTANCE_MM;
            if (distance_mm <= ULTRASONIC_MAX_DISTANCE_MM)
                valid = 1;
        }
    }

    /* No complete capture within one 65 ms service period is one missed ping. */
    Ultrasonic_Publish(distance_mm, valid);
    Ultrasonic_Trigger();
}

void Ultrasonic_GetSnapshot(UltrasonicSnapshot *snapshot)
{
    u8 index = active_snapshot;
    snapshot->distance_mm = snapshots[index].distance_mm;
    snapshot->sample_id = snapshots[index].sample_id;
    snapshot->valid = snapshots[index].valid;
    snapshot->miss_count = snapshots[index].miss_count;
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(ULTRASONIC_TIMER, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(ULTRASONIC_TIMER, TIM_IT_Update);
        if (measurement_state == ULTRASONIC_WAIT_FALL && overflow_count < 63U)
            overflow_count++;
    }

    if (TIM_GetITStatus(ULTRASONIC_TIMER, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(ULTRASONIC_TIMER, TIM_IT_CC1);
        if (measurement_state != 0)
        {
            captured_ticks = ULTRASONIC_ECHO_TIMEOUT_US;
            capture_ready = 1;
            measurement_state = 0;
            Ultrasonic_Select_Rising_Edge();
        }
    }

    if (TIM_GetITStatus(ULTRASONIC_TIMER, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(ULTRASONIC_TIMER, TIM_IT_CC2);

        if (measurement_state == ULTRASONIC_WAIT_RISE)
        {
            measurement_state = ULTRASONIC_WAIT_FALL;
            overflow_count = 0;
            TIM_SetCounter(ULTRASONIC_TIMER, 0);
            TIM_SetCompare1(ULTRASONIC_TIMER, (u16)ULTRASONIC_ECHO_TIMEOUT_US);
            Ultrasonic_Select_Falling_Edge();
        }
        else if (measurement_state == ULTRASONIC_WAIT_FALL)
        {
            captured_ticks = (u32)overflow_count * 65536UL +
                             TIM_GetCapture2(ULTRASONIC_TIMER);
            capture_ready = 1;
            measurement_state = 0;
            Ultrasonic_Select_Rising_Edge();
        }
    }
}
