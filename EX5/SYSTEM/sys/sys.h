/***********************************************
锟斤拷司锟斤拷锟斤拷趣锟狡硷拷(锟斤拷莞)锟斤拷锟睫癸拷司
品锟狡ｏ拷WHEELTEC
锟斤拷锟斤拷锟斤拷wheeltec.net
锟皆憋拷锟斤拷锟教ｏ拷shop114407458.taobao.com 
锟斤拷锟斤拷通: https://minibalance.aliexpress.com/store/4455017
锟芥本锟斤拷V1.0
锟睫革拷时锟戒：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update锟斤拷2022-09-05

All rights reserved
***********************************************/
#ifndef __SYS_H
#define __SYS_H	  
#include <stm32f10x.h>   
//0,锟斤拷支锟斤拷ucos
//1,支锟斤拷ucos
#define SYSTEM_SUPPORT_UCOS		0		//锟斤拷锟斤拷系统锟侥硷拷锟斤拷锟角凤拷支锟斤拷UCOS
																	    
	 
//位锟斤拷锟斤拷锟斤拷,实锟斤拷51锟斤拷锟狡碉拷GPIO锟斤拷锟狡癸拷锟斤拷
//锟斤拷锟斤拷实锟斤拷思锟斤拷,锟轿匡拷<<CM3权锟斤拷指锟斤拷>>锟斤拷锟斤拷锟斤拷(87页~92页).
//IO锟节诧拷锟斤拷锟疥定锟斤拷
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO锟节碉拷址映锟斤拷
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO锟节诧拷锟斤拷,只锟皆碉拷一锟斤拷IO锟斤拷!
//确锟斤拷n锟斤拷值小锟斤拷16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //锟斤拷锟?
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //锟斤拷锟斤拷 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //锟斤拷锟?
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //锟斤拷锟斤拷 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //锟斤拷锟?
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //锟斤拷锟斤拷 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //锟斤拷锟?
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //锟斤拷锟斤拷 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //锟斤拷锟?
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //锟斤拷锟斤拷

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //锟斤拷锟?
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //锟斤拷锟斤拷

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //锟斤拷锟?
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //锟斤拷锟斤拷
/////////////////////////////////////////////////////////////////
//Ex_NVIC_Config专锟矫讹拷锟斤拷
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1  //锟铰斤拷锟截达拷锟斤拷
#define RTIR   2  //锟斤拷锟斤拷锟截达拷锟斤拷
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "usart3.h"
#include "oled.h"
#include "oled_status.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "ioi2c.h"
#include "ultrasonic.h"
#include "mpu6050.h"
#include "exti.h"
#include "control.h"
#include "filter.h"
////JTAG模式锟斤拷锟矫讹拷锟斤拷
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

/* 直锟接诧拷锟斤拷锟侥达拷锟斤拷锟侥凤拷锟斤拷锟斤拷锟斤拷IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 	//锟斤拷锟轿拷叩锟狡?	
#define digitalLo(p,i)		 {p->BRR=i;}	 	//锟斤拷锟斤拷偷锟狡?
#define digitalToggle(p,i) {p->ODR ^=i;} 		//锟斤拷锟斤拷锟阶刺?
extern u8 Pick_up_stop;
extern int Middle_angle;
extern float RC_Velocity, RC_Turn_Velocity;
extern u8 Way_Angle;
extern int Motor_Left, Motor_Right;
extern volatile u16 Flag_front, Flag_back, Flag_Left, Flag_Right, Flag_velocity, Target_Velocity;
extern volatile u8 Flag_Stop;
extern volatile u8 Safety_Stop_Reason;
extern int Voltage;
extern float Angle_Balance, Gyro_Balance, Gyro_Turn;
extern int Temperature;
extern u32 Distance;
extern volatile u8 Ultrasonic_Valid, Ultrasonic_Obstacle;
extern volatile u8 Ultrasonic_Avoid_Enable, Ultrasonic_Avoid_Action;
extern volatile u8 Ultrasonic_Miss_Count;
extern u16 determine;
extern int Encoder_Left, Encoder_Right;
extern u8 delay_65, PID_Send;
extern volatile u8 delay_flag;
extern float Acceleration_Z;
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
/////////////////////////////////////////////////////////////////  
void Stm32_Clock_Init(u8 PLL);  //时锟接筹拷始锟斤拷  
void Sys_Soft_Reset(void);      //系统锟斤拷锟斤拷位
void Sys_Standby(void);         //锟斤拷锟斤拷模式 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);//锟斤拷锟斤拷偏锟狡碉拷址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);//锟斤拷锟斤拷NVIC锟斤拷锟斤拷
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//锟斤拷锟斤拷锟叫讹拷
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);//锟解部锟叫讹拷锟斤拷锟矫猴拷锟斤拷(只锟斤拷GPIOA~G)
void JTAG_Set(u8 mode);
//////////////////////////////////////////////////////////////////////////////
//锟斤拷锟斤拷为锟斤拷嗪拷锟?
void WFI_SET(void);		  //执锟斤拷WFI指锟斤拷
void INTX_DISABLE(void);//锟截憋拷锟斤拷锟斤拷锟叫讹拷
void INTX_ENABLE(void);	//锟斤拷锟斤拷锟斤拷锟斤拷锟叫讹拷
void MSR_MSP(u32 addr);	//锟斤拷锟矫讹拷栈锟斤拷址
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#endif















