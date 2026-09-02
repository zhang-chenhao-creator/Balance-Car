/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/

#ifndef __CAPTURE_H
#define	__CAPTURE_H



/*-------------超声波测距程序使用-----------
超声波初始化之后，在control.c的中断服务函数里面可以直接通过Read_Distane函数读取
放在下列的变量里面
Distance_1,Distance_2,Distance_3,Distance_4;//超声波相关变量 
-----------超声波测距程序使用-----------*/

/*-------------航模遥控程序使用和接线-----------
代码仅仅是计算PWM的高电平时间，具体需要如使用这个指令可以编程设定
-------------航模遥控程序使用和接线-----------*/


#include "control.h"





//超声波和航模遥控的宏定义，只能使用其一
//选择不需要使用的注释掉
//#define Distance_Capture
#define PWM_Capture


//超声波触发引脚
//使用端口PC15触发（普通IO）,超声波模块捕获是定时器2通道2//超声波模块1
#define 			CAPTURE_TRIG_GPIO_CLK1			RCC_APB2Periph_GPIOC
#define            	CAPTURE_TRIG_PORT1         		GPIOC
#define            	CAPTURE_TRIG_PIN1           	GPIO_Pin_15

#define 			TRIG_HIGH1						PCout(15) = 1
#define 			TRIG_LOW1						PCout(15) = 0

//使用端口PA12触发（普通IO）,超声波模块捕获是定时器2通道3//超声波模块2
#define 			CAPTURE_TRIG_GPIO_CLK2			RCC_APB2Periph_GPIOA
#define            	CAPTURE_TRIG_PORT2          	GPIOA
#define            	CAPTURE_TRIG_PIN2          		GPIO_Pin_12

#define 			TRIG_HIGH2						PAout(12) = 1
#define 			TRIG_LOW2						PAout(12) = 0

//使用端口PB13触发（普通IO）,超声波模块捕获是定时器2通道4//超声波模块3
#define 			CAPTURE_TRIG_GPIO_CLK3			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT3          	GPIOB
#define            	CAPTURE_TRIG_PIN3          		GPIO_Pin_13

#define 			TRIG_HIGH3						PBout(13) = 1
#define 			TRIG_LOW3						PBout(13) = 0

//使用端口PB12触发（普通IO）,超声波模块捕获是定时器1通道4//超声波模块4
#define 			CAPTURE_TRIG_GPIO_CLK4			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT4         		GPIOB
#define            	CAPTURE_TRIG_PIN4          		GPIO_Pin_12

#define 			TRIG_HIGH4						PBout(12) = 1
#define 			TRIG_LOW4						PBout(12) = 0

//使用高电平捕获功能
//定时器TIM2—CH2,CH3,Ch4
#define 			CAPTURE_TIM2_CH2_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH2_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH2_PIN           	GPIO_Pin_1

#define 			CAPTURE_TIM2_CH3_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH3_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH3_PIN           	GPIO_Pin_2

#define 			CAPTURE_TIM2_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH4_PIN           	GPIO_Pin_3


#define 			CAPTURE_TIM2					TIM2
#define            	CAPTURE_TIM2_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	CAPTURE_TIM2_CLK               	RCC_APB1Periph_TIM2
#define            	CAPTURE_TIM2_IRQ               	TIM2_IRQn
#define           	CAPTURE_TIM2_IRQHandler       	TIM2_IRQHandler
//#define 			CAPTURE_TIM2_CHx					TIM_Channel_2
//#define 			CAPTURE_TIM_IT_CCX 				TIM_IT_CC2//捕获通道

//使用TIM1_CH4
#define 			CAPTURE_TIM1_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM1_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM1_CH4_PIN           	GPIO_Pin_11


#define 			CAPTURE_TIM1					TIM1
#define            	CAPTURE_TIM1_APBxClock_FUN     	RCC_APB2PeriphClockCmd
#define            	CAPTURE_TIM1_CLK               	RCC_APB2Periph_TIM1
#define            	CAPTURE_TIM1_CC_IRQn             TIM1_CC_IRQn
#define            	CAPTURE_TIM1_UP_IRQn            TIM1_UP_IRQn
#define           	CAPTURE_TIM1_CC_IRQHandler      TIM1_CC_IRQHandler
#define           	CAPTURE_TIM1_UP_IRQHandler      TIM1_UP_IRQHandler






//航模遥控初始化
//使用TIM2_CH4,CH3,TIM1_CH4,CH1分别是4路航模
//捕获PWM的高电平
#define 			PWM_TIM2_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM2_CH4_PORT          	GPIOA
#define            	PWM_TIM2_CH4_PIN           	GPIO_Pin_3

#define 			PWM_TIM2_CH3_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM2_CH3_PORT          	GPIOA
#define            	PWM_TIM2_CH3_PIN           	GPIO_Pin_2

#define 			PWM_TIM2					TIM2
#define            	PWM_TIM2_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	PWM_TIM2_CLK               	RCC_APB1Periph_TIM2
#define            	PWM_TIM2_IRQ               	TIM2_IRQn
#define           	PWM_TIM2_IRQHandler       	TIM2_IRQHandler

#define 			PWM_TIM1_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM1_CH4_PORT          	GPIOA
#define            	PWM_TIM1_CH4_PIN           	GPIO_Pin_11

#define 			PWM_TIM1_CH1_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM1_CH1_PORT          	GPIOA
#define            	PWM_TIM1_CH1_PIN           	GPIO_Pin_8

#define 			PWM_TIM1					TIM1
#define            	PWM_TIM1_APBxClock_FUN     	RCC_APB2PeriphClockCmd
#define            	PWM_TIM1_CLK               	RCC_APB2Periph_TIM1
#define            	PWM_TIM1_CC_IRQn             TIM1_CC_IRQn
#define            	PWM_TIM1_UP_IRQn            TIM1_UP_IRQn
#define           	PWM_TIM1_CC_IRQHandler      TIM1_CC_IRQHandler
#define           	PWM_TIM1_UP_IRQHandler      TIM1_UP_IRQHandler



//// 获取捕获寄存器值函数宏定义
//#define            CAPTURE_TIM_GetCapturex_FUN                 TIM_GetCapture2
//// 捕获信号极性函数宏定义
//#define            CAPTURE_TIM_OCxPolarityConfig_FUN           TIM_OC2PolarityConfig
// 测量的起始边沿
#define            CAPTURE_TIM_STRAT_ICPolarity                TIM_ICPolarity_Rising
// 测量的结束边沿
#define            CAPTURE_TIM_END_ICPolarity                  TIM_ICPolarity_Falling
//光速宏定义
#define 			Light_Speed								   340

// 定时器输入捕获用户自定义变量结构体声明
typedef struct
{   
	uint8_t   Capture_FinishFlag;   // 捕获结束标志位
	uint8_t   Capture_StartFlag;    // 捕获开始标志位
	int  	  Capture_CcrValue;     // 捕获寄存器的值
	uint16_t  Capture_Period;       // 自动重装载寄存器更新标志 
}TIM_ICUserValueTypeDef;

//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
extern int Remoter_Ch1,Remoter_Ch2,Remoter_Ch3;
//Model aircraft remote control receiver variable
//航模遥控接收变量
extern int L_Remoter_Ch1,L_Remoter_Ch2,L_Remoter_Ch2;  
extern u16 Distance1,Distance2,Distance3,Distance4;	

void Read_Distane(void);        
void Distance_Cap_Init(u16 arr,u16 psc);
void PWM_Cap_Init(u16 arr,u16 psc);
void TIM1_Init(u16 arr,u16 psc);
#endif
