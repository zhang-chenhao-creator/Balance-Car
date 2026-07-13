/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

//这里是电机输出的PWM          //PWMX_IN1 为PWM输入时，PWMX_IN2 没有PWM输入时，车轮正转快衰竭
                               //PWMX_IN1 为1输入时，PWMX_IN2 有PWM输入时，车轮正转慢衰竭
#define PWMA_IN1 TIM3->CCR1   
#define PWMA_IN2 TIM3->CCR2   
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4



void MiniBalance_PWM_Init(u16 arr,u16 psc);
#endif
