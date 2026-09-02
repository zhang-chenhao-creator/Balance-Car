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


#include "ELE_CCD.h"

int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
u16  CCD_ADV[128]={0};
u8 CCD_Median,CCD_Threshold;                 //线性CCD相关

/**************************************************************************
Function: ELE_ADC_Mode_Config
Input   : none
Output  : none
函数功能：初始化电磁巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	

void ELE_ADC_Mode_Config(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ELE_ADC_L_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	
	sConfig.Channel = ELE_ADC_M_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	
	sConfig.Channel = ELE_ADC_R_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); //开启adc校准
}

/**************************************************************************
Function: ELE_ADC_Init
Input   : none
Output  : none
函数功能：初始化电磁巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	
//电磁巡线初始化
void ELE_ADC_Init(void)
{
	ELE_ADC_Mode_Config();
}

/**************************************************************************
Function: CCD_ADC_Mode_Config
Input   : none
Output  : none
函数功能：初始化CCD巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_ADC_Mode_Config(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = CCD_ADC_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); //开启adc校准

}

/**************************************************************************
Function: CCD_GPIO_Config
Input   : none
Output  : none
函数功能：初始化CCD巡线GPIO
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// 打开CCD端口时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	// CLK,SI配置为输出	
	GPIO_InitStruct.Pin = TSL_SI_PIN;				//PA4
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TSL_SI_PORT, &GPIO_InitStruct);	


	GPIO_InitStruct.Pin = TSL_CLK_PIN;				//PA5
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TSL_CLK_PORT, &GPIO_InitStruct);				
	
}


/**************************************************************************
Function: CCD_Init
Input   : none
Output  : none
函数功能：初始化CCD巡线
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_Init(void)
{
	CCD_GPIO_Config();
	CCD_ADC_Mode_Config();

}

/**************************************************************************
Function: ELE_Mode
Input   : none
Output  : none
函数功能：电磁巡线模式运行
入口参数: 无
返回  值：无
**************************************************************************/	 	
void ELE_Mode(void)
{
	if(Mode == ELE_Line_Patrol_Mode && Flag_Left!=1 &&Flag_Right!=1)
	{
		int Sum = 0;
		Sensor_Left = Get_Adc(ELE_ADC_L_CHANNEL);
		Sensor_Middle = Get_Adc(ELE_ADC_M_CHANNEL);
		Sensor_Right = Get_Adc(ELE_ADC_R_CHANNEL);
		Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
		Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
		if(Detect_Barrier() == No_Barrier)		//检测到无障碍物
		{
			Move_X=tracking_speed;
			Buzzer_Alarm(0);
		}
		else									//有障碍物
		{
			if(!Flag_Stop)
				Buzzer_Alarm(100);				//当电机使能的时候，有障碍物则蜂鸣器报警
			else 
				Buzzer_Alarm(0);
			Move_X = 0;
		}	
	}
	
}

/**************************************************************************
Function: Detect_Barrier
Input   : none
Output  : 1or0(Barrier_Detected or No_Barrier)
函数功能：电磁巡线模式雷达检测障碍物
入口参数: 无
返回  值：1或0(检测到障碍物或无障碍物)
**************************************************************************/	 	
//检测障碍物
u8 Detect_Barrier(void)
{
	u8 i;
	u8 point_count = 0;
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<225;i++)	//检测是否有障碍物
		{
			if(Dataprocess[i].angle>340 || Dataprocess[i].angle <20)
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<Detect_distance)//700mm内是否有障碍物
				point_count++,Distance = Dataprocess[i].distance;
			}
		}
		if(point_count > 3)//有障碍物
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}
/**************************************************************************
函数功能：软件延时
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

/**************************************************************************
Function: Read_TSL
Input   : none
Output  : none
函数功能：读取CCD模块的数据
入口参数: 无
返回  值：无
**************************************************************************/	 	
//读取CCD模块的数据
void RD_TSL(void) 
{
	u8 i=0,tslp=0;
	
	TSL_CLK_HIGH;
	TSL_SI_LOW;
	Dly_us();
	Dly_us();

	TSL_CLK_LOW;
	TSL_SI_LOW;
	Dly_us();
	Dly_us();

	TSL_CLK_LOW;
	TSL_SI_HIGH;
	Dly_us();
	Dly_us();

	TSL_CLK_HIGH;
	TSL_SI_HIGH;
	Dly_us();
	Dly_us();

	TSL_CLK_HIGH;
	TSL_SI_LOW;
	Dly_us();
	Dly_us();
	
	for(i=0;i<128;i++)
	{ 
		TSL_CLK_LOW; 
		Dly_us();  //调节曝光时间
		Dly_us();
		CCD_ADV[tslp]=(Get_Adc(CCD_ADC_CHANNEL))>>4;		
		++tslp;
		TSL_CLK_HIGH;
		Dly_us();  
	}
}


/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Median(void)
{ 
	static u8 i,j,Left,Right,Last_CCD_Zhongzhi;
	static u16 value1_max,value1_min;

	value1_max=CCD_ADV[0];  //动态阈值算法，读取最大和最小值
	for(i=15;i<123;i++)   //两边各去掉15个点
	{
		if(value1_max<=CCD_ADV[i])
		value1_max=CCD_ADV[i];
	}
	value1_min=CCD_ADV[0];  //最小值
	for(i=15;i<123;i++) 
	{
		if(value1_min>=CCD_ADV[i])
		value1_min=CCD_ADV[i];
	}
	CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	for(i = 15;i<118; i++)   //寻找左边跳变沿
	{
		if(CCD_ADV[i]>CCD_Yuzhi&&CCD_ADV[i+1]>CCD_Yuzhi&&CCD_ADV[i+2]>CCD_Yuzhi&&CCD_ADV[i+3]<CCD_Yuzhi&&CCD_ADV[i+4]<CCD_Yuzhi&&CCD_ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	for(j = 118;j>15; j--)//寻找右边跳变沿
	{
		if(CCD_ADV[j]<CCD_Yuzhi&&CCD_ADV[j+1]<CCD_Yuzhi&&CCD_ADV[j+2]<CCD_Yuzhi&&CCD_ADV[j+3]>CCD_Yuzhi&&CCD_ADV[j+4]>CCD_Yuzhi&&CCD_ADV[j+5]>CCD_Yuzhi)
		{	
			Right=j;
			break;	
		}
	}
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
		CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差

}	
/**************************************************************************
Function: CCD_Mode
Input   : none
Output  : none
函数功能：CCD巡线模式运行
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_Mode(void)
{
	static u8 Count_CCD = 0;								//调节CCD控制频率
	if(Mode == CCD_Line_Patrol_Mode && Flag_Left !=1 &&Flag_Right !=1)
	{
		if(++Count_CCD == 4)									//调节控制频率，4*5 = 20ms控制一次
		{
			RD_TSL(); 
      Find_CCD_Median();	//找中值					
      Count_CCD = 0;			
		}
		else if(Count_CCD>4)  Count_CCD = 0;
		
	 if(Detect_Barrier() == No_Barrier)		//检测到无障碍物
		{
			Move_X=tracking_speed;
			Buzzer_Alarm(0);
		}
		else									//有障碍物
		{
			if(!Flag_Stop)
				Buzzer_Alarm(100);				//当电机使能的时候，有障碍物则蜂鸣器报警
			else 
				Buzzer_Alarm(0);
			Move_X = 0;
		}	
		
	}
}	


/**************************************************************************
函数功能：CCD模式转向控制  巡线
入口参数：CCD提取的中线 Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int CCD_turn(u8 CCD,float gyro)//转向控制
{
	  float Turn;     
    float Bias,kp=30,Kd=0.12;	  
	  Bias=CCD-64;
	  Turn=Bias*kp+gyro*Kd;
	  if(Detect_Barrier() == Barrier_Detected)		//检测到有障碍物
	  {
		  if(!Flag_Stop)
			  Turn = 0;
		 }	
	  return Turn;
}

/**************************************************************************
函数功能：ELE模式转向控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int ELE_turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	float Turn;     
	float Bias,kp=60,Kd=0.2;	  
	Bias=Sensor-100;
	Turn=-Bias*kp-gyro*Kd;
	if(Detect_Barrier() == Barrier_Detected)		//检测到有障碍物
	{
		if(!Flag_Stop)
			Turn = 0;
		}	
	  return Turn;
}
