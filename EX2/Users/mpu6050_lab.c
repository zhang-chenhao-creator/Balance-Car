/**
 * WHEELTEC B585 MPU6050 attitude laboratory firmware.
 * Target: STM32F103RCT6, 72 MHz, HAL library.
 *
 * Hardware mapping:
 *   USART1: PA9/PA10 -> CH9102 USB serial, 115200-8-N-1
 *   Software I2C: PB14=SCL, PB15=SDA
 *   MPU6050 INT: PB9
 *
 * Serial commands must end with CRLF.
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "driver_mpu6050_dmp.h"
#include "driver_mpu6050_interface.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MPU_ADDR_WRITE                 0xD0U
#define MPU_REG_SMPLRT_DIV             0x19U
#define MPU_REG_CONFIG                 0x1AU
#define MPU_REG_GYRO_CONFIG            0x1BU
#define MPU_REG_ACCEL_CONFIG           0x1CU
#define MPU_REG_INT_PIN_CFG            0x37U
#define MPU_REG_INT_ENABLE             0x38U
#define MPU_REG_ACCEL_XOUT_H           0x3BU
#define MPU_REG_PWR_MGMT_1             0x6BU
#define MPU_REG_WHO_AM_I               0x75U

#define RAD_TO_DEG                     57.2957795f
#define PITCH_SIGN                     (1.0f)
#define DEFAULT_PERIOD_MS              20U
#define MIN_PERIOD_MS                  10U
#define MAX_PERIOD_MS                  50U

typedef enum
{
    MODE_RAW = 0,
    MODE_DMP = 1,
    MODE_COMP = 2,
    MODE_KALMAN = 3,
    MODE_ALL = 4
} attitude_mode_t;

typedef enum
{
    STATE_STATIC = 0,
    STATE_TILT = 1,
    STATE_SHAKE = 2
} lab_state_t;

typedef struct
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} raw_sample_t;

typedef struct
{
    float angle;
    float bias;
    float p00;
    float p01;
    float p10;
    float p11;
} kalman_1d_t;

static attitude_mode_t g_mode = MODE_ALL;
static lab_state_t g_state = STATE_STATIC;
static uint8_t g_log_enabled = 1U;
static uint8_t g_dmp_ok = 0U;
static uint8_t g_raw_ok = 0U;
static uint8_t g_who_am_i = 0U;
static uint32_t g_period_ms = DEFAULT_PERIOD_MS;
static uint32_t g_last_dmp_window_ms = 0U;
static uint32_t g_dmp_samples_window = 0U;
static uint32_t g_dmp_hz = 0U;

static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;
static float g_pitch_comp = 0.0f;
static float g_pitch_kalman = 0.0f;
static float g_pitch_dmp = 0.0f;
static kalman_1d_t g_kalman = {0};

static void process_serial_command(void);
static void handle_command(char *command);
static void print_help(void);
static void print_status(void);
static void print_csv(uint32_t now_ms, const raw_sample_t *raw, float pitch_acc,
                      float pitch_dmp, float pitch_comp, float pitch_kalman,
                      uint32_t dt_ms, const char *status);
static void print_fixed_3(float value);
static const char *mode_name(attitude_mode_t mode);
static const char *state_name(lab_state_t state);
static uint8_t write_reg(uint8_t reg, uint8_t value);
static uint8_t read_regs(uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t mpu_basic_init(void);
static uint8_t read_raw_sample(raw_sample_t *sample);
static float compute_pitch_acc(const raw_sample_t *sample);
static float complementary_update(float pitch_acc, float gyro_x_dps, float dt_s);
static float kalman_update(kalman_1d_t *k, float measured_angle, float gyro_rate, float dt_s);
static void calibrate_static(void);
static uint8_t init_dmp(void);
static uint8_t read_dmp(float *pitch);

int main(void)
{
    uint32_t next_sample_ms;
    uint32_t last_sample_ms;

    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);

    printf("\r\nB585 MPU6050 ATTITUDE LAB READY\r\n");
    printf("MCU=STM32F103RCT6, UART=USART1 115200, I2C=PB14/PB15, INT=PB9\r\n");
    printf("CSV: A,ms,state,mode,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,");
    printf("ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_acc,pitch_dmp,");
    printf("pitch_comp,pitch_kalman,dmp_hz,dt_ms,status\r\n");

    mpu6050_interface_iic_init();
    g_raw_ok = (mpu_basic_init() == 0U) ? 1U : 0U;
    g_dmp_ok = init_dmp();

    printf("BOOT,WHO_AM_I=0x%02X,RAW=%u,DMP=%u\r\n",
           (unsigned)g_who_am_i, (unsigned)g_raw_ok, (unsigned)g_dmp_ok);

    if (g_raw_ok != 0U)
    {
        calibrate_static();
    }

    print_help();
    g_last_dmp_window_ms = HAL_GetTick();
    last_sample_ms = HAL_GetTick();
    next_sample_ms = last_sample_ms + g_period_ms;

    while (1)
    {
        uint32_t now_ms = HAL_GetTick();

        process_serial_command();

        if ((int32_t)(now_ms - next_sample_ms) >= 0)
        {
            raw_sample_t raw;
            uint32_t dt_ms = now_ms - last_sample_ms;
            float dt_s = (float)dt_ms / 1000.0f;
            float pitch_acc = 0.0f;
            const char *status = "OK";

            last_sample_ms = now_ms;
            next_sample_ms += g_period_ms;
            if ((int32_t)(now_ms - next_sample_ms) >= (int32_t)g_period_ms)
            {
                next_sample_ms = now_ms + g_period_ms;
            }

            if (read_raw_sample(&raw) == 0U)
            {
                pitch_acc = compute_pitch_acc(&raw);
                g_pitch_comp = complementary_update(pitch_acc, raw.gx_dps - g_gyro_bias_x, dt_s);
                g_pitch_kalman = kalman_update(&g_kalman, pitch_acc, raw.gx_dps - g_gyro_bias_x, dt_s);
            }
            else
            {
                memset(&raw, 0, sizeof(raw));
                status = "RAW_FAIL";
            }

            if (g_dmp_ok != 0U)
            {
                if (read_dmp(&g_pitch_dmp) != 0U)
                {
                    status = "DMP_WAIT";
                }
            }
            else if (strcmp(status, "OK") == 0)
            {
                status = "DMP_FAIL";
            }

            if ((now_ms - g_last_dmp_window_ms) >= 1000U)
            {
                g_dmp_hz = (g_dmp_samples_window * 1000U) / (now_ms - g_last_dmp_window_ms);
                g_dmp_samples_window = 0U;
                g_last_dmp_window_ms = now_ms;
            }

            if (g_log_enabled != 0U)
            {
                print_csv(now_ms, &raw, pitch_acc, g_pitch_dmp, g_pitch_comp, g_pitch_kalman,
                          dt_ms, status);
            }
        }
    }
}

static uint8_t write_reg(uint8_t reg, uint8_t value)
{
    return mpu6050_interface_iic_write(MPU_ADDR_WRITE, reg, &value, 1);
}

static uint8_t read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return mpu6050_interface_iic_read(MPU_ADDR_WRITE, reg, buf, len);
}

static uint8_t mpu_basic_init(void)
{
    uint8_t value;

    if (read_regs(MPU_REG_WHO_AM_I, &g_who_am_i, 1) != 0U)
    {
        return 1;
    }

    value = 0x80U;
    if (write_reg(MPU_REG_PWR_MGMT_1, value) != 0U)
    {
        return 1;
    }
    delay_ms(100);

    if ((write_reg(MPU_REG_PWR_MGMT_1, 0x01U) != 0U) ||
        (write_reg(MPU_REG_SMPLRT_DIV, 0x04U) != 0U) ||
        (write_reg(MPU_REG_CONFIG, 0x03U) != 0U) ||
        (write_reg(MPU_REG_GYRO_CONFIG, 0x18U) != 0U) ||
        (write_reg(MPU_REG_ACCEL_CONFIG, 0x00U) != 0U) ||
        (write_reg(MPU_REG_INT_PIN_CFG, 0x10U) != 0U) ||
        (write_reg(MPU_REG_INT_ENABLE, 0x01U) != 0U))
    {
        return 1;
    }
    delay_ms(50);

    return 0;
}

static uint8_t read_raw_sample(raw_sample_t *sample)
{
    uint8_t buf[14];

    if (read_regs(MPU_REG_ACCEL_XOUT_H, buf, sizeof(buf)) != 0U)
    {
        return 1;
    }

    sample->ax_raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    sample->ay_raw = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    sample->az_raw = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    sample->gx_raw = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    sample->gy_raw = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    sample->gz_raw = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    sample->ax_g = (float)sample->ax_raw / 16384.0f;
    sample->ay_g = (float)sample->ay_raw / 16384.0f;
    sample->az_g = (float)sample->az_raw / 16384.0f;
    sample->gx_dps = (float)sample->gx_raw / 16.4f;
    sample->gy_dps = (float)sample->gy_raw / 16.4f;
    sample->gz_dps = (float)sample->gz_raw / 16.4f;

    return 0;
}

static float compute_pitch_acc(const raw_sample_t *sample)
{
    float denom = sqrtf((sample->ax_g * sample->ax_g) + (sample->az_g * sample->az_g));

    if (denom < 0.0001f)
    {
        denom = 0.0001f;
    }

    return PITCH_SIGN * atan2f(sample->ay_g, denom) * RAD_TO_DEG;
}

static float complementary_update(float pitch_acc, float gyro_x_dps, float dt_s)
{
    const float alpha = 0.98f;

    if ((dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        dt_s = (float)g_period_ms / 1000.0f;
    }

    g_pitch_comp = (alpha * (g_pitch_comp + gyro_x_dps * dt_s)) + ((1.0f - alpha) * pitch_acc);

    return g_pitch_comp;
}

static float kalman_update(kalman_1d_t *k, float measured_angle, float gyro_rate, float dt_s)
{
    const float q_angle = 0.001f;
    const float q_bias = 0.003f;
    const float r_measure = 0.5f;
    float rate;
    float s;
    float k0;
    float k1;
    float y;
    float p00_temp;
    float p01_temp;

    if ((dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        dt_s = (float)g_period_ms / 1000.0f;
    }

    rate = gyro_rate - k->bias;
    k->angle += dt_s * rate;

    k->p00 += dt_s * ((dt_s * k->p11) - k->p01 - k->p10 + q_angle);
    k->p01 -= dt_s * k->p11;
    k->p10 -= dt_s * k->p11;
    k->p11 += q_bias * dt_s;

    s = k->p00 + r_measure;
    if (s < 0.0001f)
    {
        s = 0.0001f;
    }
    k0 = k->p00 / s;
    k1 = k->p10 / s;
    y = measured_angle - k->angle;

    k->angle += k0 * y;
    k->bias += k1 * y;

    p00_temp = k->p00;
    p01_temp = k->p01;
    k->p00 -= k0 * p00_temp;
    k->p01 -= k0 * p01_temp;
    k->p10 -= k1 * p00_temp;
    k->p11 -= k1 * p01_temp;

    return k->angle;
}

static void calibrate_static(void)
{
    uint16_t i;
    uint16_t count = 0;
    raw_sample_t sample;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    float sum_pitch = 0.0f;
    float sum_pitch2 = 0.0f;

    printf("CAL,START,keep_static\r\n");
    for (i = 0; i < 200U; i++)
    {
        if (read_raw_sample(&sample) == 0U)
        {
            float pitch = compute_pitch_acc(&sample);
            sum_gx += sample.gx_dps;
            sum_gy += sample.gy_dps;
            sum_gz += sample.gz_dps;
            sum_pitch += pitch;
            sum_pitch2 += pitch * pitch;
            count++;
        }
        delay_ms(5);
    }

    if (count > 0U)
    {
        float mean = sum_pitch / (float)count;
        float var = (sum_pitch2 / (float)count) - (mean * mean);
        float std = (var > 0.0f) ? sqrtf(var) : 0.0f;

        g_gyro_bias_x = sum_gx / (float)count;
        g_gyro_bias_y = sum_gy / (float)count;
        g_gyro_bias_z = sum_gz / (float)count;
        g_pitch_comp = mean;
        memset(&g_kalman, 0, sizeof(g_kalman));
        g_kalman.angle = mean;
        g_pitch_kalman = mean;

        printf("CAL,DONE,count=%u,gx_bias=", (unsigned)count);
        print_fixed_3(g_gyro_bias_x);
        printf(",gy_bias=");
        print_fixed_3(g_gyro_bias_y);
        printf(",gz_bias=");
        print_fixed_3(g_gyro_bias_z);
        printf(",pitch_mean=");
        print_fixed_3(mean);
        printf(",pitch_std=");
        print_fixed_3(std);
        printf("\r\n");
    }
    else
    {
        printf("CAL,FAIL,no_raw_samples\r\n");
    }
}

static void dmp_receive_callback(uint8_t type)
{
    if (type == MPU6050_INTERRUPT_FIFO_OVERFLOW)
    {
        printf("EVENT,DMP_FIFO_OVERFLOW\r\n");
    }
}

static uint8_t init_dmp(void)
{
    if (mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW,
                         dmp_receive_callback,
                         mpu6050_interface_dmp_tap_callback,
                         mpu6050_interface_dmp_orient_callback) != 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t read_dmp(float *pitch)
{
    int16_t accel_raw[8][3];
    float accel_g[8][3];
    int16_t gyro_raw[8][3];
    float gyro_dps[8][3];
    int32_t quat[8][4];
    float pitch_buf[8];
    float roll_buf[8];
    float yaw_buf[8];
    uint16_t len = 8U;

    if (mpu6050_dmp_read_all(accel_raw, accel_g, gyro_raw, gyro_dps,
                             quat, pitch_buf, roll_buf, yaw_buf, &len) != 0U)
    {
        return 1U;
    }

    if (len == 0U)
    {
        return 1U;
    }

    (void)pitch_buf;
    *pitch = PITCH_SIGN * roll_buf[len - 1U];
    g_dmp_samples_window += len;

    return 0U;
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
    long value = 0;

    if (strcmp(command, "HELP") == 0)
    {
        print_help();
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        print_status();
    }
    else if (strcmp(command, "CAL") == 0)
    {
        calibrate_static();
    }
    else if (sscanf(command, "LOG %ld", &value) == 1)
    {
        g_log_enabled = (value != 0) ? 1U : 0U;
        printf("OK,LOG,%u\r\n", (unsigned)g_log_enabled);
    }
    else if (strcmp(command, "MODE RAW") == 0)
    {
        g_mode = MODE_RAW;
        printf("OK,MODE,RAW\r\n");
    }
    else if (strcmp(command, "MODE DMP") == 0)
    {
        g_mode = MODE_DMP;
        printf("OK,MODE,DMP\r\n");
    }
    else if (strcmp(command, "MODE COMP") == 0)
    {
        g_mode = MODE_COMP;
        printf("OK,MODE,COMP\r\n");
    }
    else if (strcmp(command, "MODE KALMAN") == 0)
    {
        g_mode = MODE_KALMAN;
        printf("OK,MODE,KALMAN\r\n");
    }
    else if (strcmp(command, "MODE ALL") == 0)
    {
        g_mode = MODE_ALL;
        printf("OK,MODE,ALL\r\n");
    }
    else if (strcmp(command, "STATE STATIC") == 0)
    {
        g_state = STATE_STATIC;
        printf("OK,STATE,STATIC\r\n");
    }
    else if (strcmp(command, "STATE TILT") == 0)
    {
        g_state = STATE_TILT;
        printf("OK,STATE,TILT\r\n");
    }
    else if (strcmp(command, "STATE SHAKE") == 0)
    {
        g_state = STATE_SHAKE;
        printf("OK,STATE,SHAKE\r\n");
    }
    else if (sscanf(command, "RATE %ld", &value) == 1)
    {
        if (value == 20)
        {
            g_period_ms = 50U;
        }
        else if (value == 50)
        {
            g_period_ms = 20U;
        }
        else if (value == 100)
        {
            g_period_ms = 10U;
        }
        else
        {
            printf("ERR,RATE_EXPECTS_20_50_100\r\n");
            return;
        }

        if (g_period_ms < MIN_PERIOD_MS)
        {
            g_period_ms = MIN_PERIOD_MS;
        }
        if (g_period_ms > MAX_PERIOD_MS)
        {
            g_period_ms = MAX_PERIOD_MS;
        }
        printf("OK,RATE,%ld,period_ms=%lu\r\n", value, (unsigned long)g_period_ms);
    }
    else
    {
        printf("ERR,UNKNOWN_COMMAND,%s\r\n", command);
    }
}

static void print_help(void)
{
    printf("HELP: commands end with CRLF\r\n");
    printf("STATUS | CAL | LOG 0|1\r\n");
    printf("MODE RAW|DMP|COMP|KALMAN|ALL\r\n");
    printf("STATE STATIC|TILT|SHAKE\r\n");
    printf("RATE 20|50|100\r\n");
}

static void print_status(void)
{
    printf("STATUS,WHO_AM_I=0x%02X,raw=%u,dmp=%u,log=%u,mode=%s,state=%s,rate_hz=%lu,dmp_hz=%lu\r\n",
           (unsigned)g_who_am_i, (unsigned)g_raw_ok, (unsigned)g_dmp_ok,
           (unsigned)g_log_enabled, mode_name(g_mode), state_name(g_state),
           (unsigned long)(1000U / g_period_ms), (unsigned long)g_dmp_hz);
}

static const char *mode_name(attitude_mode_t mode)
{
    switch (mode)
    {
        case MODE_RAW: return "RAW";
        case MODE_DMP: return "DMP";
        case MODE_COMP: return "COMP";
        case MODE_KALMAN: return "KALMAN";
        case MODE_ALL: return "ALL";
        default: return "UNKNOWN";
    }
}

static const char *state_name(lab_state_t state)
{
    switch (state)
    {
        case STATE_STATIC: return "STATIC";
        case STATE_TILT: return "TILT";
        case STATE_SHAKE: return "SHAKE";
        default: return "UNKNOWN";
    }
}

static void print_fixed_3(float value)
{
    int32_t scaled;

    if (value >= 0.0f)
    {
        scaled = (int32_t)((value * 1000.0f) + 0.5f);
    }
    else
    {
        scaled = (int32_t)((value * 1000.0f) - 0.5f);
    }

    if (scaled < 0)
    {
        printf("-");
        scaled = -scaled;
    }

    printf("%ld.%03ld", (long)(scaled / 1000), (long)(scaled % 1000));
}

static void print_csv(uint32_t now_ms, const raw_sample_t *raw, float pitch_acc,
                      float pitch_dmp, float pitch_comp, float pitch_kalman,
                      uint32_t dt_ms, const char *status)
{
    printf("A,%lu,%s,%s,%d,%d,%d,%d,%d,%d,",
           (unsigned long)now_ms, state_name(g_state), mode_name(g_mode),
           (int)raw->ax_raw, (int)raw->ay_raw, (int)raw->az_raw,
           (int)raw->gx_raw, (int)raw->gy_raw, (int)raw->gz_raw);

    print_fixed_3(raw->ax_g);
    printf(",");
    print_fixed_3(raw->ay_g);
    printf(",");
    print_fixed_3(raw->az_g);
    printf(",");
    print_fixed_3(raw->gx_dps);
    printf(",");
    print_fixed_3(raw->gy_dps);
    printf(",");
    print_fixed_3(raw->gz_dps);
    printf(",");
    print_fixed_3(pitch_acc);
    printf(",");
    print_fixed_3(pitch_dmp);
    printf(",");
    print_fixed_3(pitch_comp);
    printf(",");
    print_fixed_3(pitch_kalman);
    printf(",%lu,%lu,%s\r\n", (unsigned long)g_dmp_hz, (unsigned long)dt_ms, status);
}
