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
#include "show.h"
#include "TrackModule.h"	//红外巡线(4路寻迹)显示
float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
/**************************************************************************
Function: Track_OLED_Show_State
Input   : none
Output  : none
函数功能：显示红外巡线当前识别到的状态(第0行左侧,8字符定宽,自动清旧字)
入口参数：无
返回  值：无
**************************************************************************/
static void Track_OLED_Show_State(void)
{
	const u8 *name;
	switch(Track_state)
	{
		case STATE_STRAIGHT:    name = "STRAIGHT"; break;
		case STATE_CROSS:       name = "CROSS   "; break;
		case STATE_LEFT_90_A:   case STATE_LEFT_90_B:  name = "L_90    "; break;
		case STATE_RIGHT_90_A:  case STATE_RIGHT_90_B: name = "R_90    "; break;
		case STATE_LEFT_BIG:    name = "L_BIG   "; break;
		case STATE_RIGHT_BIG:   name = "R_BIG   "; break;
		case STATE_LEFT_SMALL:  name = "L_SML   "; break;
		case STATE_RIGHT_SMALL: name = "R_SML   "; break;
		case STATE_LOST:        name = "LOST    "; break;
		default:                name = "UNKNOWN "; break;
	}
	OLED_ShowString(0,0,name);
}
void oled_show(void)
{
	 memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏
		//=============第一行显示小车模式=======================//	
		if((Mode!=CCD_Line_Patrol_Mode)&&(Mode!=Track_Line_Patrol_Mode))
		{			
			if(Way_Angle==1)	OLED_ShowString(0,0,"DMP");
			else if(Way_Angle==2)	OLED_ShowString(0,0,"Kalman");
			else if(Way_Angle==3)	OLED_ShowString(0,0,"C F");
		}

		if(Mode==Lidar_Follow_Mode) OLED_ShowString(60,0,"Follow  ");
		else if(Mode == ROS_Mode)   OLED_ShowString(60,0,"ROS   ");
		else if(Mode == Lidar_Avoid_Mode)   OLED_ShowString(60,0,"Avoid   ");
		else if(Mode == Lidar_Straight_Mode)  OLED_ShowString(60,0,"Straight");
		else if(Mode ==ELE_Line_Patrol_Mode)  
		{
		    OLED_ShowNumber(0,0,Sensor_Left,5,12);	
			OLED_ShowNumber(30,0,Sensor_Middle,4,12);
			OLED_ShowNumber(60,0,Sensor_Right,4,12);
			OLED_ShowNumber(90,0,Sensor,4,12);		
		}
		else if(Mode ==CCD_Line_Patrol_Mode)  OLED_Show_CCD();
		else if(Mode == Track_Line_Patrol_Mode)	//红外巡线: 识别状态+IRDM+DH1~DH4
		{
			Track_OLED_Show_State();			//第0行左侧: 当前识别状态(8字符定宽)
			OLED_ShowString(66,0,"IRDM");
			OLED_ShowNumber(98,0,DH1?1:0,1,12);
			OLED_ShowNumber(105,0,DH2?1:0,1,12);
			OLED_ShowNumber(112,0,DH3?1:0,1,12);
			OLED_ShowNumber(119,0,DH4?1:0,1,12);
		}
		else if(Mode == Ultrasonic_Avoid_Mode) OLED_ShowString(60,0,"U_Avoid");
		else if(Mode == Ultrasonic_Follow_Mode) OLED_ShowString(60,0,"U_Follow");
		else               OLED_ShowString(60,0,"Normal  ");
		//=============第二行显示角度=======================//	
		                      OLED_ShowString(00,10,"Angle");
		if(PS2_ON_Flag==RC_ON)	OLED_ShowString(82,10,"PS2 ");	//PS2手柄
		else if(Remote_ON_Flag==RC_ON) OLED_ShowString(82,10,"R-C");
		if((((Mode==ELE_Line_Patrol_Mode)||(Mode==CCD_Line_Patrol_Mode))&&(Lidar_Detect==1))||(((Mode==Lidar_Avoid_Mode)||(Mode==Lidar_Follow_Mode)||(Mode==Lidar_Straight_Mode))&&Lidar_flag==1)) 
			                    OLED_ShowString(82,10,"Lidar");//ELE\CCD巡线模式下接入雷达的显示
		if( Angle_Balance<0)	OLED_ShowString(48,10,"-");
		if(Angle_Balance>=0)	OLED_ShowString(48,10,"+");
		                      OLED_ShowNumber(56,10, myabs((int)Angle_Balance),3,12);
	  //=============第三行显示角速度与距离===============//
    if(Mode==CCD_Line_Patrol_Mode)
    {
				//=============第三行显示编码器1=======================//	
														OLED_ShowString(00,20,"Z");
			if( CCD_Zhongzhi<0)		OLED_ShowString(10,20,"-"),
														OLED_ShowNumber(25,20,-CCD_Zhongzhi,3,12);
			else                 	OLED_ShowString(10,20,"+"),
														OLED_ShowNumber(25,20, CCD_Zhongzhi,3,12);
														OLED_ShowString(70,20,"Y");
			if( CCD_Yuzhi<0)		OLED_ShowString(80,20,"-"),
														OLED_ShowNumber(95,20,-CCD_Yuzhi,3,12);
			else                 	OLED_ShowString(80,20,"+"),
														OLED_ShowNumber(95,20, CCD_Yuzhi,3,12);
		}			
		else
		{
														OLED_ShowString(0,20,"Gyrox");
			if(Gyro_Balance<0)	  OLED_ShowString(42,20,"-");
			if(Gyro_Balance>=0)	  OLED_ShowString(42,20,"+");
														OLED_ShowNumber(50,20, myabs((int)Gyro_Balance),4,12);
														
														OLED_ShowNumber(82,20,(u16)Distance,5,12);
														OLED_ShowString(114,20,"mm");
		}

		//=============第四行显示左编码器PWM与读数=======================//	
		                      OLED_ShowString(00,30,"L");
		if(Motor_Left<0)		  OLED_ShowString(16,30,"-"),
													OLED_ShowNumber(26,30,myabs((int)Motor_Left),4,12);
		if(Motor_Left>=0)	    OLED_ShowString(16,30,"+"),
		                      OLED_ShowNumber(26,30,myabs((int)Motor_Left),4,12);
													
		if(Velocity_Left<0)	  OLED_ShowString(60,30,"-");
		if(Velocity_Left>=0)	OLED_ShowString(60,30,"+");
		                      OLED_ShowNumber(68,30,myabs((int)Velocity_Left),4,12);
													OLED_ShowString(96,30,"mm/s");
	
		//=============第五行显示右编码器PWM与读数=======================//		
		                      OLED_ShowString(00,40,"R");
		if(Motor_Right<0)		  OLED_ShowString(16,40,"-"),
													OLED_ShowNumber(26,40,myabs((int)Motor_Right),4,12);
		if(Motor_Right>=0)	  OLED_ShowString(16,40,"+"),
		                      OLED_ShowNumber(26,40,myabs((int)Motor_Right),4,12);
													
		if(Velocity_Right<0)	OLED_ShowString(60,40,"-");
		if(Velocity_Right>=0)	OLED_ShowString(60,40,"+");
		                      OLED_ShowNumber(68,40,myabs((int)Velocity_Right),4,12);
													OLED_ShowString(96,40,"mm/s");

		//=============第六行显示电压与电机开关=======================//
		                      OLED_ShowString(0,50,"V");
													OLED_ShowString(30,50,".");
													OLED_ShowString(64,50,"V");
													OLED_ShowNumber(19,50,Voltage/100,2,12);
													OLED_ShowNumber(42,50,Voltage/10%10,1,12);
													OLED_ShowNumber(50,50,Voltage%10,1,12);
		if(Flag_Stop)         OLED_ShowString(95,50,"OFF");
		if(!Flag_Stop)        OLED_ShowString(95,50,"ON ");
											
		//=============刷新=======================//
		OLED_Refresh_Gram();	
}
/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
  static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;		if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
	Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;			  //对编码器数据就行数据处理便于图形化
	Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//发送PID参数,在APP调参界面显示
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Target_Velocity,(int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,(int)Distance_KP,(int)Distance_KD);//打印到APP上面	
		PID_Send=0;	
	}	
   else	if(flag==0)		// 发送电池电压，速度，角度等参数，在APP首页显示
	 {
		 printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance); //打印到APP上面
	 }
		
	 else								//发送小车姿态角，在波形界面显示
	   printf("{B%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw); //x，y，z轴角度 在APP上面显示波形
																													//可按格式自行增加显示波形，最多可显示五个
}
/**************************************************************************
Function: Virtual oscilloscope sends data to upper computer
Input   : none
Output  : none
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{   
	u8 i;//计数变量
	float Vol;								//电压变量
	unsigned char Send_Count; //串口需要发送的数据个数
	Vol=(float)Voltage/100;
	DataScope_Get_Channel_Data( Angle_Balance, 1 );       //显示角度 单位：度（°）
	DataScope_Get_Channel_Data( Distance/10, 2 );         //显示超声波测量的距离 单位：CM 
	DataScope_Get_Channel_Data( Vol, 3 );                 //显示电池电压 单位：V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(3);
	for(i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
}

/**************************************************************************
Function: OLED_Show_CCD
Input   : none
Output  : none
函数功能：CCD模式显示函数，画点
入口参数: 无 
返回  值：无
**************************************************************************/	 	

void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	u8 i=0;
	OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	for(i = 0;i<8; i++)
	{
		OLED_DrawPoint(x,y+i,t);
	}
}

void OLED_Show_CCD(void)
{ 
	u8 i,t;
	for(i = 0;i<128; i++)
	{
		if(CCD_ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
	}
}
