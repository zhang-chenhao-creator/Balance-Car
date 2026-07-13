#include "encoder.h"
#include "stm32f10x_gpio.h"




void Encoder_Init_TIM8(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	//NVIC_InitTypeDef NVIC_InitStruct;
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能定时器8的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能PB端口时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;	//滤波10
  TIM_ICInit(TIM8, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
  TIM_ClearFlag(TIM8, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM8,0);
  TIM_Cmd(TIM8, ENABLE); 
	

//	NVIC_InitStruct.NVIC_IRQChannel =TIM8_UP_IRQn;  		//定时器8中断
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  			//使能IRQ通道
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级1 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;       	//响应优先级3
//	NVIC_Init(&NVIC_InitStruct);
	
}
/**************************************************************************
Function: Initialize TIM4 to encoder interface mode
Input   : none
Output  : none
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	//NVIC_InitTypeDef NVIC_InitStruct;	
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
 
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
	


//	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;  		//定时器4中断
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  			//使能IRQ通道
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级1 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;       	//响应优先级3
//	NVIC_Init(&NVIC_InitStruct);

}
/**************************************************************************
Function: Read encoder count per unit time
Input   : TIMX：Timer
Output  : none
函数功能：单位时间读取编码器计数
入口参数：TIMX：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 case 8:  Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0;break;
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/**************************************************************************
Function: TIM4 interrupt service function
Input   : none
Output  : none
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//溢出中断
	{
	 
	} 
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 	//清除中断标志位 	    
}
/**************************************************************************
Function: TIM2 interrupt service function
Input   : none
Output  : none
函数功能：TIM8中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM8_UP_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetFlagStatus(TIM8,TIM_FLAG_Update)==SET)//溢出中断
	{
	 
	} 
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update); 	//清除中断标志位 	    
}



