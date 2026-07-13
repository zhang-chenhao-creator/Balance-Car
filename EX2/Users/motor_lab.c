/**
 * WHEELTEC B585 motor, encoder and incremental PI laboratory firmware.
 * Target: STM32F103RCT6, 72 MHz, HAL library.
 *
 * Hardware mapping (verified against the supplied C10B schematic/resource table):
 *   TIM3 CH1/CH2: PA6/PA7 -> left AT8236 IN1/IN2
 *   TIM3 CH3/CH4: PB0/PB1 -> right AT8236 IN1/IN2
 *   TIM4 CH1/CH2: PB6/PB7 -> left quadrature encoder
 *   TIM8 CH1/CH2: PC6/PC7 -> right quadrature encoder
 *   USART1: PA9/PA10 -> CH9102 USB serial, 115200-8-N-1
 *   PC13: motor enable switch, active low
 *
 * Serial commands must end with CRLF.
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CONTROL_PERIOD_MS                 10U
#define PWM_PERIOD_COUNTS                 7200
#define ENCODER_MOTOR_CPR                 2000L
#define MOTOR_GEAR_RATIO                  30L
#define ENCODER_CPR_DEFAULT               (ENCODER_MOTOR_CPR * MOTOR_GEAR_RATIO)
#define WHEEL_CIRCUMFERENCE_MM_X1000      204204L /* 65 mm wheel; calibrate if needed. */
#define MOTOR_ENABLE_ACTIVE_LEVEL         GPIO_PIN_RESET
#define COMMAND_TIMEOUT_MS                30000U

typedef enum
{
    MODE_STOP = 0,
    MODE_OPEN = 1,
    MODE_SPEED_PI = 2,
    MODE_SWEEP = 3
} lab_mode_t;

static TIM_HandleTypeDef g_tim3_pwm;
static TIM_HandleTypeDef g_tim4_encoder;
static TIM_HandleTypeDef g_tim8_encoder;

static lab_mode_t g_mode = MODE_STOP;
static uint8_t g_armed = 0U;
static uint8_t g_log_enabled = 0U;
static int32_t g_encoder_cpr = ENCODER_CPR_DEFAULT;
static int32_t g_encoder_sign_left = 1;
static int32_t g_encoder_sign_right = 1;

static int32_t g_target_left_mm_s = 0;
static int32_t g_target_right_mm_s = 0;
static int32_t g_speed_left_mm_s = 0;
static int32_t g_speed_right_mm_s = 0;
static int32_t g_pwm_left = 0;
static int32_t g_pwm_right = 0;
static int32_t g_total_left = 0;
static int32_t g_total_right = 0;

/* PI gains are scaled by 1000. Defaults: Kp=3.5, Ki=20.0. */
static int32_t g_kp_x1000 = 3500;
static int32_t g_ki_x1000 = 20000;
static int32_t g_pi_prev_error_left = 0;
static int32_t g_pi_prev_error_right = 0;
static int32_t g_pi_output_left = 0;
static int32_t g_pi_output_right = 0;

static uint32_t g_last_command_ms = 0U;
static uint32_t g_sweep_start_ms = 0U;
static uint8_t g_sweep_step = 0U;

static void gpio_safety_init(void);
static void pwm_tim3_init(void);
static void encoder_tim4_init(void);
static void encoder_tim8_init(void);
static void fatal_error(const char *reason);
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high);
static uint8_t motor_enable_is_active(void);
static void motor_apply(int32_t left, int32_t right);
static void motor_force_stop(void);
static void pi_reset(void);
static int32_t pi_update(int32_t target, int32_t measured, int32_t *previous_error, int32_t *output);
static void control_tick(uint32_t now_ms);
static void process_serial_command(void);
static void handle_command(char *command);
static void print_help(void);
static void print_status(void);

int main(void)
{
    uint32_t next_control_ms;

    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);

    gpio_safety_init();
    pwm_tim3_init();
    encoder_tim4_init();
    encoder_tim8_init();
    motor_force_stop();

    printf("\r\nB585 MOTOR/ENCODER/PI LAB READY\r\n");
    printf("MCU=STM32F103RCT6, PWM=10kHz, sample=10ms, CPR=%ld\r\n", (long)g_encoder_cpr);
    printf("Safety: boot is DISARMED; PC13 enable switch must be active.\r\n");
    printf("Type HELP followed by CRLF.\r\n");
    printf("CSV: D,ms,mode,targetL,targetR,speedL,speedR,pwmL,pwmR,rawL,rawR,totalL,totalR,enable\r\n");

    g_last_command_ms = HAL_GetTick();
    next_control_ms = HAL_GetTick() + CONTROL_PERIOD_MS;

    while (1)
    {
        uint32_t now_ms = HAL_GetTick();

        process_serial_command();

        if ((int32_t)(now_ms - next_control_ms) >= 0)
        {
            next_control_ms += CONTROL_PERIOD_MS;
            control_tick(now_ms);

            /* Recover cleanly if serial printing or a debugger delayed the loop. */
            if ((int32_t)(now_ms - next_control_ms) >= (int32_t)CONTROL_PERIOD_MS)
            {
                next_control_ms = now_ms + CONTROL_PERIOD_MS;
            }
        }
    }
}

static void gpio_safety_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void pwm_tim3_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef output = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_tim3_pwm.Instance = TIM3;
    g_tim3_pwm.Init.Prescaler = 0;
    g_tim3_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim3_pwm.Init.Period = PWM_PERIOD_COUNTS - 1;
    g_tim3_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim3_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&g_tim3_pwm) != HAL_OK)
    {
        fatal_error("TIM3 PWM init");
    }

    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;

    if ((HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm, &output, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm, &output, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm, &output, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm, &output, TIM_CHANNEL_4) != HAL_OK))
    {
        fatal_error("TIM3 channel config");
    }

    HAL_TIM_PWM_Start(&g_tim3_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_tim3_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_tim3_pwm, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&g_tim3_pwm, TIM_CHANNEL_4);
}

static void encoder_common_init(TIM_HandleTypeDef *timer)
{
    TIM_Encoder_InitTypeDef encoder = {0};

    timer->Init.Prescaler = 0;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 0xFFFF;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 6;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 6;

    if (HAL_TIM_Encoder_Init(timer, &encoder) != HAL_OK)
    {
        fatal_error("encoder timer init");
    }

    __HAL_TIM_SET_COUNTER(timer, 0);
    if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK)
    {
        fatal_error("encoder timer start");
    }
}

static void encoder_tim4_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_tim4_encoder.Instance = TIM4;
    encoder_common_init(&g_tim4_encoder);
}

static void encoder_tim8_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    g_tim8_encoder.Instance = TIM8;
    encoder_common_init(&g_tim8_encoder);
}

static void fatal_error(const char *reason)
{
    motor_force_stop();
    printf("FATAL,%s\r\n", reason);
    __disable_irq();
    while (1)
    {
    }
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static uint8_t motor_enable_is_active(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == MOTOR_ENABLE_ACTIVE_LEVEL) ? 1U : 0U;
}

static void motor_apply(int32_t left, int32_t right)
{
    left = clamp_i32(left, -PWM_PERIOD_COUNTS, PWM_PERIOD_COUNTS);
    right = clamp_i32(right, -PWM_PERIOD_COUNTS, PWM_PERIOD_COUNTS);

    if ((g_armed == 0U) || (motor_enable_is_active() == 0U))
    {
        left = 0;
        right = 0;
    }

    if (left >= 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_1, left);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_2, -left);
    }

    if (right >= 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_3, right);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_4, -right);
    }

    g_pwm_left = left;
    g_pwm_right = right;
}

static void motor_force_stop(void)
{
    if (g_tim3_pwm.Instance == TIM3)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm, TIM_CHANNEL_4, 0);
    }
    g_pwm_left = 0;
    g_pwm_right = 0;
}

static void pi_reset(void)
{
    g_pi_prev_error_left = 0;
    g_pi_prev_error_right = 0;
    g_pi_output_left = 0;
    g_pi_output_right = 0;
}

static int32_t pi_update(int32_t target, int32_t measured, int32_t *previous_error, int32_t *output)
{
    int32_t error = target - measured;
    int64_t delta = ((int64_t)g_kp_x1000 * (error - *previous_error)) / 1000;
    delta += ((int64_t)g_ki_x1000 * error) / 100000; /* Ki * error * 0.01 s */
    *output = clamp_i32(*output + (int32_t)delta, -PWM_PERIOD_COUNTS, PWM_PERIOD_COUNTS);
    *previous_error = error;
    return *output;
}

static void control_tick(uint32_t now_ms)
{
    int16_t raw_left = (int16_t)__HAL_TIM_GET_COUNTER(&g_tim4_encoder);
    int16_t raw_right = (int16_t)__HAL_TIM_GET_COUNTER(&g_tim8_encoder);
    int32_t delta_left;
    int32_t delta_right;
    int32_t requested_left = 0;
    int32_t requested_right = 0;

    __HAL_TIM_SET_COUNTER(&g_tim4_encoder, 0);
    __HAL_TIM_SET_COUNTER(&g_tim8_encoder, 0);

    delta_left = (int32_t)raw_left * g_encoder_sign_left;
    delta_right = (int32_t)raw_right * g_encoder_sign_right;
    g_total_left += delta_left;
    g_total_right += delta_right;

    g_speed_left_mm_s = (int32_t)(((int64_t)delta_left * WHEEL_CIRCUMFERENCE_MM_X1000 * 100L) /
                                  ((int64_t)g_encoder_cpr * 1000L));
    g_speed_right_mm_s = (int32_t)(((int64_t)delta_right * WHEEL_CIRCUMFERENCE_MM_X1000 * 100L) /
                                   ((int64_t)g_encoder_cpr * 1000L));

    if ((g_armed != 0U) && ((uint32_t)(now_ms - g_last_command_ms) > COMMAND_TIMEOUT_MS))
    {
        g_armed = 0U;
        g_mode = MODE_STOP;
        pi_reset();
        printf("EVENT,COMMAND_TIMEOUT_STOP\r\n");
    }

    if (g_mode == MODE_OPEN)
    {
        requested_left = g_target_left_mm_s;
        requested_right = g_target_right_mm_s;
    }
    else if (g_mode == MODE_SPEED_PI)
    {
        requested_left = pi_update(g_target_left_mm_s, g_speed_left_mm_s,
                                   &g_pi_prev_error_left, &g_pi_output_left);
        requested_right = pi_update(g_target_right_mm_s, g_speed_right_mm_s,
                                    &g_pi_prev_error_right, &g_pi_output_right);
    }
    else if (g_mode == MODE_SWEEP)
    {
        static const int16_t sweep_pwm[5] = {1200, 1800, 2400, 3200, 4000};
        uint32_t elapsed = now_ms - g_sweep_start_ms;
        uint8_t step = (uint8_t)(elapsed / 3000U);

        if (step >= 5U)
        {
            g_mode = MODE_STOP;
            requested_left = 0;
            requested_right = 0;
            printf("EVENT,SWEEP_COMPLETE\r\n");
        }
        else
        {
            if (step != g_sweep_step)
            {
                g_sweep_step = step;
                printf("EVENT,SWEEP_STEP,%u,%d\r\n", (unsigned)(step + 1U), sweep_pwm[step]);
            }
            requested_left = sweep_pwm[step];
            requested_right = sweep_pwm[step];
        }
    }

    motor_apply(requested_left, requested_right);

    if (g_log_enabled != 0U)
    {
        printf("D,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%ld,%ld,%u\r\n",
               (unsigned long)now_ms, (unsigned)g_mode,
               (long)g_target_left_mm_s, (long)g_target_right_mm_s,
               (long)g_speed_left_mm_s, (long)g_speed_right_mm_s,
               (long)g_pwm_left, (long)g_pwm_right,
               (int)raw_left, (int)raw_right,
               (long)g_total_left, (long)g_total_right,
               (unsigned)motor_enable_is_active());
    }
}

static void process_serial_command(void)
{
    if ((g_usart_rx_sta & 0x8000U) != 0U)
    {
        uint16_t length;
        char command[USART_REC_LEN];

        __disable_irq();
        length = g_usart_rx_sta & 0x3FFFU;
        if (length >= (USART_REC_LEN - 1U))
        {
            length = USART_REC_LEN - 1U;
        }
        memcpy(command, g_usart_rx_buf, length);
        command[length] = '\0';
        g_usart_rx_sta = 0U;
        __enable_irq();

        handle_command(command);
    }
}

static void handle_command(char *command)
{
    long a = 0;
    long b = 0;

    g_last_command_ms = HAL_GetTick();

    if (strcmp(command, "HELP") == 0)
    {
        print_help();
    }
    else if (strcmp(command, "ARM") == 0)
    {
        if (motor_enable_is_active() != 0U)
        {
            g_armed = 1U;
            g_log_enabled = 1U;
            printf("OK,ARMED\r\n");
        }
        else
        {
            printf("ERR,PC13_ENABLE_SWITCH_INACTIVE\r\n");
        }
    }
    else if (strcmp(command, "STOP") == 0)
    {
        g_armed = 0U;
        g_mode = MODE_STOP;
        g_target_left_mm_s = 0;
        g_target_right_mm_s = 0;
        pi_reset();
        motor_force_stop();
        printf("OK,STOPPED_AND_DISARMED\r\n");
    }
    else if (sscanf(command, "OPEN %ld %ld", &a, &b) == 2)
    {
        g_target_left_mm_s = clamp_i32((int32_t)a, -PWM_PERIOD_COUNTS, PWM_PERIOD_COUNTS);
        g_target_right_mm_s = clamp_i32((int32_t)b, -PWM_PERIOD_COUNTS, PWM_PERIOD_COUNTS);
        g_mode = MODE_OPEN;
        pi_reset();
        printf("OK,OPEN,%ld,%ld\r\n", (long)g_target_left_mm_s, (long)g_target_right_mm_s);
    }
    else if (sscanf(command, "SPEED %ld %ld", &a, &b) == 2)
    {
        g_target_left_mm_s = clamp_i32((int32_t)a, -1000, 1000);
        g_target_right_mm_s = clamp_i32((int32_t)b, -1000, 1000);
        g_mode = MODE_SPEED_PI;
        pi_reset();
        printf("OK,SPEED,%ld,%ld\r\n", (long)g_target_left_mm_s, (long)g_target_right_mm_s);
    }
    else if (sscanf(command, "PI %ld %ld", &a, &b) == 2)
    {
        g_kp_x1000 = clamp_i32((int32_t)a, 0, 50000);
        g_ki_x1000 = clamp_i32((int32_t)b, 0, 200000);
        pi_reset();
        printf("OK,PI,Kp_x1000=%ld,Ki_x1000=%ld\r\n", (long)g_kp_x1000, (long)g_ki_x1000);
    }
    else if ((strcmp(command, "P1") == 0) || (strcmp(command, "P2") == 0) || (strcmp(command, "P3") == 0))
    {
        if (command[1] == '1')
        {
            g_kp_x1000 = 2000;
            g_ki_x1000 = 12000;
        }
        else if (command[1] == '2')
        {
            g_kp_x1000 = 3500;
            g_ki_x1000 = 20000;
        }
        else
        {
            g_kp_x1000 = 5000;
            g_ki_x1000 = 30000;
        }
        pi_reset();
        printf("OK,%s,Kp_x1000=%ld,Ki_x1000=%ld\r\n", command, (long)g_kp_x1000, (long)g_ki_x1000);
    }
    else if (sscanf(command, "SIGN %ld %ld", &a, &b) == 2)
    {
        g_encoder_sign_left = (a < 0) ? -1 : 1;
        g_encoder_sign_right = (b < 0) ? -1 : 1;
        printf("OK,SIGN,%ld,%ld\r\n", (long)g_encoder_sign_left, (long)g_encoder_sign_right);
    }
    else if (sscanf(command, "CPR %ld", &a) == 1)
    {
        g_encoder_cpr = clamp_i32((int32_t)a, 100, 120000);
        printf("OK,CPR,%ld\r\n", (long)g_encoder_cpr);
    }
    else if (sscanf(command, "LOG %ld", &a) == 1)
    {
        g_log_enabled = (a != 0) ? 1U : 0U;
        printf("OK,LOG,%u\r\n", (unsigned)g_log_enabled);
    }
    else if (strcmp(command, "ZERO") == 0)
    {
        g_total_left = 0;
        g_total_right = 0;
        __HAL_TIM_SET_COUNTER(&g_tim4_encoder, 0);
        __HAL_TIM_SET_COUNTER(&g_tim8_encoder, 0);
        printf("OK,ENCODER_TOTALS_ZEROED\r\n");
    }
    else if (strcmp(command, "ENC") == 0)
    {
        printf("ENC,totalL=%ld,totalR=%ld,CPR=%ld\r\n",
               (long)g_total_left, (long)g_total_right, (long)g_encoder_cpr);
    }
    else if (strcmp(command, "SWEEP") == 0)
    {
        g_mode = MODE_SWEEP;
        g_sweep_step = 0xFFU;
        g_sweep_start_ms = HAL_GetTick();
        g_log_enabled = 1U;
        pi_reset();
        printf("OK,SWEEP,1200|1800|2400|3200|4000,3s_each\r\n");
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        print_status();
    }
    else
    {
        printf("ERR,UNKNOWN_COMMAND,%s\r\n", command);
    }
}

static void print_help(void)
{
    printf("HELP: commands end with CRLF\r\n");
    printf("ARM | STOP | STATUS | LOG 0|1\r\n");
    printf("OPEN leftPWM rightPWM       (-7200..7200)\r\n");
    printf("SWEEP                       (five PWM levels, 3 s each)\r\n");
    printf("SPEED left_mm_s right_mm_s  (incremental PI)\r\n");
    printf("P1 | P2 | P3                (three preset PI groups)\r\n");
    printf("PI kp_x1000 ki_x1000        (custom gains)\r\n");
    printf("ZERO | ENC | CPR counts | SIGN left right\r\n");
}

static void print_status(void)
{
    printf("STATUS,armed=%u,enable=%u,mode=%u,CPR=%ld,sign=%ld/%ld,Kp_x1000=%ld,Ki_x1000=%ld\r\n",
           (unsigned)g_armed, (unsigned)motor_enable_is_active(), (unsigned)g_mode,
           (long)g_encoder_cpr, (long)g_encoder_sign_left, (long)g_encoder_sign_right,
           (long)g_kp_x1000, (long)g_ki_x1000);
}
