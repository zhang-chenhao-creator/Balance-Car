/**
 ******************************************************************************
 * @file     main.c
 * @author   ÕýµãÔ­×ÓÍÅ¶Ó(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    ÐÂ½¨¹¤³ÌÊµÑé-HAL¿â°æ±¾ ÊµÑé
 * @license  Copyright (c) 2020-2032, ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾
 ******************************************************************************
 * @attention
 * 
 * ÊµÑéÆ½Ì¨:ÕýµãÔ­×Ó STM32F103 ¿ª·¢°å
 * ÔÚÏßÊÓÆµ:www.yuanzige.com
 * ¼¼ÊõÂÛÌ³:www.openedv.com
 * ¹«Ë¾ÍøÖ·:www.alientek.com
 * ¹ºÂòµØÖ·:openedv.taobao.com
 ******************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

void led_init(void);                       /* LED³õÊ¼»¯º¯ÊýÉùÃ÷ */

int main(void)
{
    HAL_Init();                              /* ³õÊ¼»¯HAL¿â */
    sys_stm32_clock_init(RCC_PLL_MUL9);      /* ÉèÖÃÊ±ÖÓ, 72Mhz */
    delay_init(72);                          /* ÑÓÊ±³õÊ¼»¯ */
    led_init();                              /* LED³õÊ¼»¯ */
    
    while(1)
    { 
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);    /* PB5ÖÃ1 */ 
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);  /* PE5ÖÃ0 */ 
        delay_ms(500);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);  /* PB5ÖÃ0 */
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);    /* PE5ÖÃ1 */
        delay_ms(500); 
    }
}

/**
 * @brief       ³õÊ¼»¯LEDÏà¹ØIO¿Ú, ²¢Ê¹ÄÜÊ±ÖÓ
 * @param       ÎÞ
 * @retval      ÎÞ
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();                          /* IO¿ÚPBÊ±ÖÓÊ¹ÄÜ */
    __HAL_RCC_GPIOE_CLK_ENABLE();                          /* IO¿ÚPEÊ±ÖÓÊ¹ÄÜ */

    gpio_initstruct.Pin = GPIO_PIN_5;                      /* LED0Òý½Å */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* ÍÆÍìÊä³ö */
    gpio_initstruct.Pull = GPIO_PULLUP;                    /* ÉÏÀ­ */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ¸ßËÙ */
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);                /* ³õÊ¼»¯LED0Òý½Å */

    gpio_initstruct.Pin = GPIO_PIN_5;                      /* LED1Òý½Å */
    HAL_GPIO_Init(GPIOE, &gpio_initstruct);                /* ³õÊ¼»¯LED1Òý½Å */
}
