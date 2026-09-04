/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-09-05

All rights reserved
***********************************************/
#include "show.h"
#include "TrackModule.h"	//����Ѳ��(4·Ѱ��)��ʾ
#include "avoid_routine.h"
float Velocity_Left,Velocity_Right;	//�����ٶ�(mm/s)
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
/**************************************************************************
Function: Track_OLED_Show_State
Input   : none
Output  : none
�������ܣ���ʾ����Ѳ�ߵ�ǰʶ�𵽵�״̬(��0�����,8�ַ�����,�Զ������)
��ڲ�������
����  ֵ����
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
	 memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM���㵫������ˢ�£���ֹ����
		//=============��һ����ʾС��ģʽ=======================//	
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
		else if(Mode == Track_Line_Patrol_Mode)	//����Ѳ��: ʶ��״̬+IRDM+DH1~DH4
		{
			Track_OLED_Show_State();			//��0�����: ��ǰʶ��״̬(8�ַ�����)
			OLED_ShowString(66,0,"IRDM");
			OLED_ShowNumber(98,0,DH1?1:0,1,12);
			OLED_ShowNumber(105,0,DH2?1:0,1,12);
			OLED_ShowNumber(112,0,DH3?1:0,1,12);
			OLED_ShowNumber(119,0,DH4?1:0,1,12);
			if(Guard_State==OBSTACLE_GUARD_DISABLED)     OLED_ShowString(82,10,"O");
			else if(Guard_State==OBSTACLE_GUARD_CLEAR)   OLED_ShowString(82,10,"C");
			else if(Guard_State==OBSTACLE_GUARD_SLOW)    OLED_ShowString(82,10,"S");
			else if(Guard_State==OBSTACLE_GUARD_BLOCKED) OLED_ShowString(82,10,"B");
			else                                         OLED_ShowString(82,10,"D");
			OLED_ShowNumber(90,10,Avoid_State,2,12);
		}
		else if(Mode == Ultrasonic_Avoid_Mode) OLED_ShowString(60,0,"U_Avoid");
		else if(Mode == Ultrasonic_Follow_Mode) OLED_ShowString(60,0,"U_Follow");
		else               OLED_ShowString(60,0,"Normal  ");
		//=============�ڶ�����ʾ�Ƕ�=======================//	
		                      OLED_ShowString(00,10,"Angle");
		if(PS2_ON_Flag==RC_ON)	OLED_ShowString(82,10,"PS2 ");	//PS2�ֱ�
		else if(Remote_ON_Flag==RC_ON) OLED_ShowString(82,10,"R-C");
		if((((Mode==ELE_Line_Patrol_Mode)||(Mode==CCD_Line_Patrol_Mode))&&(Lidar_Detect==1))||(((Mode==Lidar_Avoid_Mode)||(Mode==Lidar_Follow_Mode)||(Mode==Lidar_Straight_Mode))&&Lidar_flag==1)) 
			                    OLED_ShowString(82,10,"Lidar");//ELE\CCDѲ��ģʽ�½����״����ʾ
		if( Angle_Balance<0)	OLED_ShowString(48,10,"-");
		if(Angle_Balance>=0)	OLED_ShowString(48,10,"+");
		                      OLED_ShowNumber(56,10, myabs((int)Angle_Balance),3,12);
	  //=============��������ʾ���ٶ������===============//
    if(Mode==CCD_Line_Patrol_Mode)
    {
				//=============��������ʾ������1=======================//	
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

		//=============��������ʾ�������PWM�����=======================//	
		                      OLED_ShowString(00,30,"L");
		if(Motor_Left<0)		  OLED_ShowString(16,30,"-"),
													OLED_ShowNumber(26,30,myabs((int)Motor_Left),4,12);
		if(Motor_Left>=0)	    OLED_ShowString(16,30,"+"),
		                      OLED_ShowNumber(26,30,myabs((int)Motor_Left),4,12);
													
		if(Velocity_Left<0)	  OLED_ShowString(60,30,"-");
		if(Velocity_Left>=0)	OLED_ShowString(60,30,"+");
		                      OLED_ShowNumber(68,30,myabs((int)Velocity_Left),4,12);
													OLED_ShowString(96,30,"mm/s");
	
		//=============��������ʾ�ұ�����PWM�����=======================//		
		                      OLED_ShowString(00,40,"R");
		if(Motor_Right<0)		  OLED_ShowString(16,40,"-"),
													OLED_ShowNumber(26,40,myabs((int)Motor_Right),4,12);
		if(Motor_Right>=0)	  OLED_ShowString(16,40,"+"),
		                      OLED_ShowNumber(26,40,myabs((int)Motor_Right),4,12);
													
		if(Velocity_Right<0)	OLED_ShowString(60,40,"-");
		if(Velocity_Right>=0)	OLED_ShowString(60,40,"+");
		                      OLED_ShowNumber(68,40,myabs((int)Velocity_Right),4,12);
													OLED_ShowString(96,40,"mm/s");

		//=============��������ʾ��ѹ��������=======================//
		                      OLED_ShowString(0,50,"V");
													OLED_ShowString(30,50,".");
													OLED_ShowString(64,50,"V");
													OLED_ShowNumber(19,50,Voltage/100,2,12);
													OLED_ShowNumber(42,50,Voltage/10%10,1,12);
													OLED_ShowNumber(50,50,Voltage%10,1,12);
		if(Flag_Stop)         OLED_ShowString(95,50,"OFF");
		if(!Flag_Stop)        OLED_ShowString(95,50,"ON ");
											
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
}
/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
  static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;		if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //�Ե�ѹ���ݽ��д���
	Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;			  //�Ա��������ݾ������ݴ�������ͼ�λ�
	Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//����PID����,��APP���ν�����ʾ
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Target_Velocity,(int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,(int)Distance_KP,(int)Distance_KD);//��ӡ��APP����	
		PID_Send=0;	
	}	
   else	if(flag==0)		// ���͵�ص�ѹ���ٶȣ��ǶȵȲ�������APP��ҳ��ʾ
	 {
		 printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance); //��ӡ��APP����
	 }
		
	 else								//����С����̬�ǣ��ڲ��ν�����ʾ
	   printf("{B%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw); //x��y��z��Ƕ� ��APP������ʾ����
																													//�ɰ���ʽ����������ʾ���Σ�������ʾ���
}
/**************************************************************************
Function: Virtual oscilloscope sends data to upper computer
Input   : none
Output  : none
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
**************************************************************************/
void DataScope(void)
{   
	u8 i;//��������
	float Vol;								//��ѹ����
	unsigned char Send_Count; //������Ҫ���͵����ݸ���
	Vol=(float)Voltage/100;
	DataScope_Get_Channel_Data( Angle_Balance, 1 );       //��ʾ�Ƕ� ��λ���ȣ��㣩
	DataScope_Get_Channel_Data( Distance/10, 2 );         //��ʾ�����������ľ��� ��λ��CM 
	DataScope_Get_Channel_Data( Vol, 3 );                 //��ʾ��ص�ѹ ��λ��V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
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
�������ܣ�CCDģʽ��ʾ����������
��ڲ���: �� 
����  ֵ����
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
