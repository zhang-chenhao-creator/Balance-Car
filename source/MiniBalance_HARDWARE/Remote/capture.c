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

//这个模块默认不使用
//4路超声波模块
//或 4路航模遥控
#include "capture.h"
#include "tim.h"
//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500;
//Model aircraft remote control receiver variable
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500;  
u16 TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL;

TIM_ICUserValueTypeDef PWM_TIM2_CH4_ICUserValueStructure = {0,0,0,0};//航模遥控第一路
TIM_ICUserValueTypeDef PWM_TIM2_CH3_ICUserValueStructure = {0,0,0,0};//航模遥控第二路
TIM_ICUserValueTypeDef PWM_TIM1_CH4_ICUserValueStructure = {0,0,0,0};//航模遥控第三路
TIM_ICUserValueTypeDef PWM_TIM1_CH1_ICUserValueStructure = {0,0,0,0};//航模遥控第四路

//使用航模遥控
#ifdef PWM_Capture
/**************************************************************************
Function: HAL_TIM_IC_CaptureCallback
Input   : none
Output  : none
函数功能：高电平捕获中断回调函数
入口参数: 无
返回  值：无
**************************************************************************/	
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{ 	
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	u16 tsr;
	tsr=TIM2->SR;
	//连接航模遥遥控器后，需要推下前进杆，才可以正式航模控制小车
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
  if(Remoter_Ch2>1600&&Remote_ON_Flag==0)
  {
		//Model aircraft remote control mark position 1, other marks position 0
		//航模遥控标志位置1，其它标志位置0
		Remote_ON_Flag=1;
		PS2_ON_Flag=0;
	}
	// 当要被捕获的信号的周期大于定时器的最长定时时，定时器就会溢出，产生更新中断
	// 这个时候我们需要把这个最长的定时周期加到捕获信号的时间里面去
	if(htim == &htim2)
	{
		/*************************************超声波通道2*******************************************/
		 if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
		{
		    if((TIM2CH2_CAPTURE_STA&0X80)==0)//还未成功捕获
		    {
				if(TIM2CH2_CAPTURE_STA&0X40)  //捕获到一个下降沿   
				{      
				 TIM2CH2_CAPTURE_STA|=0X80;  //标记成功捕获到一次高电平脉宽
				 TIM2CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//获取当前的捕获值.
				 __HAL_TIM_DISABLE(&htim2);
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM2通道2上升沿捕获
				 __HAL_TIM_ENABLE(&htim2);//使能定时器2
				}
				else          //还未开始,第一次捕获上升沿
				{
				 TIM2CH2_CAPTURE_STA=0;   //清空
				 TIM2CH2_CAPTURE_VAL=0;
				 TIM2CH2_CAPTURE_STA|=0X40;  //标记捕获到了上升沿
				 //配置tim前一定要先关闭tim，配置完以后再使能
				 __HAL_TIM_DISABLE(&htim2);        //关闭定时器2
				 __HAL_TIM_SET_COUNTER(&htim2,0);  //计数器CNT置0
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时器3通道3设置为下降沿捕获
				 __HAL_TIM_ENABLE(&htim2);//使能定时器2
				}      
		   }  
	   } 
	/*************************************通道3*******************************************/
	    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if( PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag == 0)// 第一次捕获
			{
			  PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//第一次捕获时，把捕获值储存起来
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿									
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 1;	// 开始捕获标志位置1		
			}
			else// 下降沿捕获中断,第二次捕获
			{
				// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
				Remoter_Ch1 = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3)-PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue;
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if(ch1_filter_times<=5) Remoter_Ch1 = L_Remoter_Ch1;
					else ch1_filter_times = 0;
				}
				else
					ch1_filter_times=0;
				L_Remoter_Ch1 = Remoter_Ch1;
			  // 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);
				// 开始捕获标志清0		
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 0;
			}
		}
	/*************************************通道4*******************************************/
	    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag==0)// 第一次捕获
			{
                PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//第一次捕获时，把捕获值储存起来
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 1; // 开始捕获标志位置1		
			}
			else
			{
				// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
				Remoter_Ch2=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4)-PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue;
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if(ch2_filter_times<=5) Remoter_Ch2 = L_Remoter_Ch2;
					else ch2_filter_times = 0;
				}
				else
					ch2_filter_times=0;
				L_Remoter_Ch2 = Remoter_Ch2;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 0;// 开始捕获标志清0		
			}
		}
	}
}

#endif

/**************************************************************************
Function: Ultrasonic receiving echo function
Input   : none
Output  : none
函数功能：超声波接收回波函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Read_Distane(void)        
{   
	 PCout(15)=1;         
	 delay_us(15);  
	 PCout(15)=0;	
	 if(TIM2CH2_CAPTURE_STA&0X80)//成功捕获到了一次高电平
	 {
		 Distance=TIM2CH2_CAPTURE_STA&0X3F; 
		 Distance*=65536;					        //溢出时间总和
		 Distance+=TIM2CH2_CAPTURE_VAL;		//得到总的高电平时间
		 Distance=Distance*170/1000;      //时间*声速/2（来回） 一个计数0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//开启下一次捕获
	 }				
}


