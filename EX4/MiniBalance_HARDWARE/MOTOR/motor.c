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
#include "motor.h"
/**************************************************************************
Function: PWM_OutPut_TIM_GPIO_Config
Input   : none
Output  : none
函数功能：配置PWM输出端口，用于控制电机
入口参数: 无 
返回  值：无
**************************************************************************/	 	
static void PWM_OutPut_TIM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 输出比较通道1 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 输出比较通道2 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 输出比较通道3 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 输出比较通道4 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


///*
// * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
// * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
// * 另外三个成员是通用定时器和高级定时器才有.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            都有
// *	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
// *  TIM_Period               都有
// *  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)

/**************************************************************************
Function: PWM_OutPut_TIM_Mode_Config
Input   : none
Output  : none
函数功能：配置PWM输出模式，用于控制电机
入口参数: 无 
返回  值：无
**************************************************************************/	 	
static void PWM_OutPut_TIM_Mode_Config(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

  // 开启定时器时钟,即内部时钟CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	/*--------------------时基结构体初始化-------------------------*/

	TIM_TimeBaseInitStruct.TIM_Period = arr;              			//设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          			//设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        //设置时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);      	//初始化定时器

	
	/*--------------------输出比较结构体初始化-------------------*/	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);                	//初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC2Init(TIM3,&TIM_OCInitStruct);                 	//初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC3Init(TIM3,&TIM_OCInitStruct);                  //初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC4Init(TIM3,&TIM_OCInitStruct);                  //初始化输出比较参数

	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH1使能预装载寄存器
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH2使能预装载寄存器
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH3使能预装载寄存器
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH4使能预装载寄存器

	TIM_ARRPreloadConfig(TIM3, ENABLE);                	//使定时器3在ARR上的预装载寄存器

	TIM_Cmd(TIM3,ENABLE);                              	//使能定时器3
}

void MiniBalance_PWM_Init(u16 arr,u16 psc)
{
	PWM_OutPut_TIM_GPIO_Config();			//GPIO配置
	PWM_OutPut_TIM_Mode_Config(arr,psc);	//模式配置
}
