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



#ifndef __ELE_CCD_H
#define	__ELE_CCD_H

#include "sys.h"

/*******************************电磁巡线ADC**************************/
//PA4，PA5，PC5--IN4，IN5，IN15

#define    ELE_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
#define    ELE_ADC                          	 ADC1
#define    ELE_ADC_CLK                      	 RCC_APB2Periph_ADC1
#define    ELE_ADC_GPIO_APBxClock_FUN       	 RCC_APB2PeriphClockCmd




/********************************CCD巡线***************************/
//PA4--TSL_SI；PA5--TSL_CLK;PC5--ADC

//TSL_SI
#define    TSL_SI_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
#define    TSL_SI_PORT                     	 	GPIOA
#define    TSL_SI_PIN                       	GPIO_PIN_4


//TSL_CLK
#define    TSL_CLK_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
#define    TSL_CLK_PORT                     	GPIOA
#define    TSL_CLK_PIN                       	GPIO_PIN_5

//CCD_ADC
#define    CCD_ADC_GPIO_CLK                	 	RCC_APB2Periph_GPIOC  
#define    CCD_ADC_PORT                     	GPIOC
#define    CCD_ADC_PIN                       	GPIO_PIN_5

#define    CCD_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
#define    CCD_ADC                          	 ADC1
#define    CCD_ADC_CLK                      	 RCC_APB2Periph_ADC1



#define TSL_SI_HIGH								PAout(4) = 1
#define TSL_SI_LOW								PAout(4) = 0
#define TSL_CLK_HIGH							PAout(5) = 1
#define TSL_CLK_LOW								PAout(5) = 0



#define Barrier_Detected						1
#define No_Barrier								0

#define tracking_speed 300      //给小车一个大概300mm/s的速度
#define Detect_distance 700//检测距离为700mm


extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
extern u16  CCD_ADV[128];
extern u8 CCD_Median,CCD_Threshold;             


void ELE_ADC_Init(void);
void ELE_Mode(void);
void CCD_Init(void);
void Dly_us(void);
void RD_TSL(void);
void  Find_CCD_Median(void);
void CCD_Mode(void);
int CCD_turn(u8 CCD,float gyro);//CCD模式转向控制
int ELE_turn(int encoder_left,int encoder_right,float gyro);//ELE模式转向控制

u8 Detect_Barrier(void);


#endif
