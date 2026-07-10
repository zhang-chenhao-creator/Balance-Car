/**
 * Board interface for LibDriver MPU6050 on WHEELTEC B585/C10B.
 *
 * C10B MPU6050 module header:
 *   PB14 -> SCL
 *   PB15 -> SDA
 *   PB9  -> INT
 */

#include "driver_mpu6050_interface.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include <stdarg.h>
#include <stdio.h>

#define MPU_I2C_PORT                 GPIOB
#define MPU_I2C_SCL_PIN              GPIO_PIN_14
#define MPU_I2C_SDA_PIN              GPIO_PIN_15
#define MPU_I2C_INT_PIN              GPIO_PIN_9

static void i2c_delay(void)
{
    volatile uint32_t i;

    for (i = 0; i < 42U; i++)
    {
        __NOP();
    }
}

static void scl_high(void)
{
    HAL_GPIO_WritePin(MPU_I2C_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void scl_low(void)
{
    HAL_GPIO_WritePin(MPU_I2C_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void sda_high(void)
{
    HAL_GPIO_WritePin(MPU_I2C_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void sda_low(void)
{
    HAL_GPIO_WritePin(MPU_I2C_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static uint8_t sda_read(void)
{
    return (HAL_GPIO_ReadPin(MPU_I2C_PORT, MPU_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t data)
{
    uint8_t i;
    uint8_t nack;

    for (i = 0; i < 8U; i++)
    {
        if ((data & 0x80U) != 0U)
        {
            sda_high();
        }
        else
        {
            sda_low();
        }
        i2c_delay();
        scl_high();
        i2c_delay();
        scl_low();
        data <<= 1;
    }

    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();
    nack = sda_read();
    scl_low();
    i2c_delay();

    return nack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    sda_high();
    for (i = 0; i < 8U; i++)
    {
        data <<= 1;
        scl_high();
        i2c_delay();
        if (sda_read() != 0U)
        {
            data |= 1U;
        }
        scl_low();
        i2c_delay();
    }

    if (ack != 0U)
    {
        sda_low();
    }
    else
    {
        sda_high();
    }
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    sda_high();
    i2c_delay();

    return data;
}

uint8_t mpu6050_interface_iic_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = MPU_I2C_SCL_PIN | MPU_I2C_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU_I2C_PORT, &gpio);

    gpio.Pin = MPU_I2C_INT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MPU_I2C_PORT, &gpio);

    sda_high();
    scl_high();

    return 0;
}

uint8_t mpu6050_interface_iic_deinit(void)
{
    return 0;
}

uint8_t mpu6050_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return 1;
    }

    i2c_start();
    if (i2c_write_byte(addr) != 0U)
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write_byte(reg) != 0U)
    {
        i2c_stop();
        return 1;
    }
    i2c_start();
    if (i2c_write_byte(addr | 0x01U) != 0U)
    {
        i2c_stop();
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        buf[i] = i2c_read_byte((i + 1U) < len);
    }
    i2c_stop();

    return 0;
}

uint8_t mpu6050_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    i2c_start();
    if (i2c_write_byte(addr) != 0U)
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write_byte(reg) != 0U)
    {
        i2c_stop();
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        if (i2c_write_byte(buf[i]) != 0U)
        {
            i2c_stop();
            return 1;
        }
    }
    i2c_stop();

    return 0;
}

void mpu6050_interface_delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

void mpu6050_interface_debug_print(const char *const fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void mpu6050_interface_receive_callback(uint8_t type)
{
    (void)type;
}

void mpu6050_interface_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    printf("EVENT,DMP_TAP,%u,%u\r\n", (unsigned)count, (unsigned)direction);
}

void mpu6050_interface_dmp_orient_callback(uint8_t orientation)
{
    printf("EVENT,DMP_ORIENT,%u\r\n", (unsigned)orientation);
}
