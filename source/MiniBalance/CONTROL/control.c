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
#include "control.h"	//���ƺ���
#include "TrackModule.h"	//����Ѳ��ģ��(4·������Ѱ��,��PS2������չ�ӿ�)
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
�������ܣ����еĿ��ƴ��붼��������
         5ms�ⲿ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ�����ʱ��ͬ��	
��ڲ�������
����  ֵ����				 
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{ 		
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//��ѹ������ر���
	static u8 Flag_Target;																//���ƺ�����ر������ṩ10ms��׼
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM��
	if(GPIO_Pin==GPIO_PIN_9)
	{
		if(Time_count < START_DELAY)   //����ǰ3��(600*5ms): MPU6050���ݲ��ȶ�, �������������
		{
			Time_count++;
			if(delay_flag==1)          //ά����ѭ��50ms����(DataScopeʾ������),��������
			{
				if(++delay_50==10) delay_50=0,delay_flag=0;
			}
			Set_Pwm(0,0);                //�ȶ����ڵ��ǿ��ͣת(���PWM=7200/7200)
			return;
		}
		Flag_Target=!Flag_Target;
		Encoder_Left=Read_Encoder(4);            					  //��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Encoder_Right=-Read_Encoder(8);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
																												//����A���TIM4_CH1,����A���TIM8_CH1,�����������������ļ��Բ���ͬ
		Get_Angle(Way_Angle);                     					//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		Mode_Choose();                                      //С��ģʽ��ѡ��
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//����������ת�ٶȣ�mm/s��
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//���������ṩ50ms�ľ�׼��ʱ��ʾ������Ҫ50ms�߾�����ʱ
		}
		if(++Ros_count == 10)
		{	
            Lidar_flag=0;			
			Ros_send_flag=1;
			Ros_count=0;
		}
		if(Flag_Target==1)                        					//10ms����һ��
		{		
			Voltage_Temp=Get_battery_volt();		    					//��ȡ��ص�ѹ		
			Voltage_Count++;                       						//ƽ��ֵ������
			Voltage_All+=Voltage_Temp;              					//��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ			                                               
		}                                         					//10ms����һ��
        if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)		
	       Read_Distane();                                  //��������ȡ����   
		Select_Zhongzhi();                                  //��е��ֵ��ѡ��
		Normal();                                           //��ͨģʽ
		Lidar_Avoid();                                      //�״����ģʽ
		Lidar_Follow();                                     //�״����ģʽ
		Lidar_Straight();                                   //�״���ֱ��ģʽ 
		ELE_Mode();                                         //���ѭ��ģʽ
		CCD_Mode();                                         //CCDѲ��ģʽ����
		if(Mode==Track_Line_Patrol_Mode)  IRDM_line_inspection();//����Ѳ��(4·Ѱ��ģ��)
		if(Mode==Normal_Mode)	Led_Flash(100);               //LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
		else Led_Flash(0);                                  //LED����;����ģʽ	
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽ��PID���� Gyro_Balanceƽ����ٶȼ��ԣ�ǰ��Ϊ��������Ϊ��
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //�ٶȻ�PID����	��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		if(Mode ==CCD_Line_Patrol_Mode)                     //CCDѭ���µ�ת�򻷿��� 
			Turn_Pwm=CCD_turn(CCD_Zhongzhi,Gyro_Turn);
		else if(Mode==ELE_Line_Patrol_Mode)
			Turn_Pwm=ELE_turn(Encoder_Left,Encoder_Right,Gyro_Turn);
		else
		  Turn_Pwm=Turn(Gyro_Turn);													//ת��PID����     
		

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //�������ֵ������PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //�������ֵ������PWM
																												//PWMֵ����ʹС��ǰ��������ʹС������
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM�޷�

		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Pick_up_stop=1;	                           					//���������͹رյ��
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Pick_up_stop=0;	                           					//��������¾��������
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//����������쳣
			Set_Pwm(Motor_Left,Motor_Right);         					//��ֵ��PWM�Ĵ���  
	 }       	  
} 

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle��Gyro��angular velocity
Output  : balance��Vertical control PWM
�������ܣ�ֱ��PD����		
��ڲ�����Angle:�Ƕȣ�Gyro�����ٶ�
����  ֵ��balance��ֱ������PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//���ƽ��ĽǶ���ֵ �ͻ�е���
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left��Left wheel encoder reading��encoder_right��Right wheel encoder reading
Output  : Speed control PWM
�������ܣ��ٶȿ���PWM		
��ڲ�����encoder_left�����ֱ�����������encoder_right�����ֱ���������
����  ֵ���ٶȿ���PWM
**************************************************************************/
//�޸�ǰ�������ٶȣ����޸�Target_Velocity�����磬�ĳ�60�ͱȽ�����
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral;
	  //================ң��ǰ�����˲���====================// 
    if(Mode==Track_Line_Patrol_Mode)
		{
			Movement=base_speed_mm/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //����Ѳ��: Ŀ���ٶ�=Ѳ��ģ��base_speed_mm(mm/s)
			if(Track_InPlaceTurn) Movement=0;  // ԭ��ת����ǰ������
		}
		else if(Flag_front==1)		Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //�յ�ǰ���ź�
		else if(Flag_back==1)	Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //�յ������ź�
	  else  Movement=Move_X/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;//�������ٶ�ת��Ϊ����ı�����������λ;
          //Movement=Move_X/�ܳ�/��������ȡƵ��*Ƶ��*���ٱ�*����*2;	
	  if(Movement>2400)  Movement=2400;                     //����ң���ٶȵ����ƣ������ƻ�С����ƽ��
	  
	//=============���������ܣ�����/���ϣ�==================// 
	  if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //����
			 Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //�յ�ǰ���ź�
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //�յ�ǰ���ź�
		if(Mode==Ultrasonic_Avoid_Mode&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //����������
			 Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //�յ�ǰ���ź�
		
   //================�ٶ�PI������=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ� 
		Encoder_bias *= 0.84;		                                          //һ�׵�ͨ�˲���       
		Encoder_bias += Encoder_Least*0.16;	                              //һ�׵�ͨ�˲����������ٶȱ仯 
		Encoder_Integral +=Encoder_bias;                                  //���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //����ң�������ݣ�����ǰ������
		if(Encoder_Integral>200000)  	Encoder_Integral=200000;            //�����޷�
		if(Encoder_Integral<-200000)	  Encoder_Integral=-200000;         //�����޷�	
		if(Mode==Track_Line_Patrol_Mode && Track_InPlaceTurn)
			Encoder_Integral=0;      // ����ǰ�����֣�����һ����һ����
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //�ٶȿ���
    if(Mode == ROS_Mode)
		{ 
			if(++Ros_Rate>=100) Ros_Rate=0,Move_X=0;//���ros��200ms��û�з������ݹ�����Move_Z��0������Ϊ��ros�˿���С���ĸ���˳��
		}
		else Move_X	=0;	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//����رպ��������
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
�������ܣ�ת����� 
��ڲ�����Z��������
����  ֵ��ת�����PWM
��    �ߣ���Ȥ�Ƽ�����ݸ�����޹�˾
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//�޸�ת���ٶȣ����޸�Turn_Amplitude����
	//===================ң��������ת����=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else if(Mode==Track_Line_Patrol_Mode)
	 {
		 if(Track_InPlaceTurn && (turn_diff>0.5f || turn_diff<-0.5f))
			 Turn_Target=(turn_diff>0)?Turn_Amplitude:(-Turn_Amplitude); // ԭ��ת����ң�������ȣ�һ����һ����
		 else
			 Turn_Target=turn_diff*Track_Turn_Scale; //����Ѳ��: ת�����(mm/s)*ϵ��
	 }
		 else Turn_Target=0;
		 if(1==Flag_front||1==Flag_back||Mode==Track_Line_Patrol_Mode)  Kd=Turn_Kd; //����Ѳ��: ��������������
	 else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  //===================ת��PD������=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//���Z�������ǽ���PD����
   if(Mode == ROS_Mode)
	 {
		 if(++Ros_Rate>=40) Ros_Rate=0,Move_Z=0;//���ros��200msû�з������ݹ�����Move_Z��0������Ϊ��ros�˿���С���ĸ���˳��
	 }
	 else Move_Z=0;
	 return turn;								 				 //ת��PWM��תΪ������תΪ��
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)		
	{
		PWMA_IN1=7200;
		PWMA_IN2=7200-motor_left;//����ǰ��
	}		
	else 
	{
		PWMA_IN1=7200+motor_left;
		PWMA_IN2=7200;
	} //���ֺ���
  if(motor_right>0)			
	{
		PWMB_IN1=7200-motor_right;
		PWMB_IN2=7200;
	}		//����ǰ��
	else 	        			  
	{
		PWMB_IN1=7200;
		PWMB_IN2=7200+motor_right;

	}//���ֺ���
	
}
/**************************************************************************
Function: PWM limiting range
Input   : IN��Input  max��Maximum value  min��Minimum value
Output  : Output
�������ܣ�����PWM��ֵ 
��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
����  ֵ���޷����ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}


/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle��Car inclination��voltage��Voltage
Output  : 1��abnormal��0��normal
�������ܣ��쳣�رյ��		
��ڲ�����angle��С����ǣ�voltage����ѹ
����  ֵ��1���쳣  0������
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	Flag_Stop = KEY2_STATE;
	if(KEY2_STATE==1) Pick_up_stop=0;                  //key2�رգ�Pick_up_stop�ָ�Ϊ0
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000||Pick_up_stop==1)//��ص�ѹ����11.1V�رյ��
	{	                                                 //��Ǵ���40�ȹرյ��
		temp=1;                                          //Flag_Stop��1�����������ƹرյ��
		PWMA_IN1=0;                                      //Pick_up_stop��1����С��������ֹ����0����������С�� 
		PWMA_IN2=0;
		PWMB_IN1=0;
		PWMB_IN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
  float gyro_x,gyro_y;
	Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn=gyro[2];                 //����ת����ٶ�
		Acceleration_Z=accel[2];           //����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת��
//		if(Accel_X>32768) Accel_X-=65536;                //��������ת��
//		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת��
//		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת��
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ�
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��
		gyro_x=Gyro_X/16.4;                              //����������ת�������̡�2000��/s��Ӧ������16.4���ɲ��ֲ�
		gyro_y=Gyro_Y/16.4;                              //����������ת��	
		if(Way_Angle==2)		  	
		{
			 Pitch = -Kalman_Filter_x(Accel_Angle_x,gyro_x);//�������˲�
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//�����˲�
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //����ƽ�����
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ�
	}
	Angle_Balance += ANGLE_ZERO_OFFSET;            //��㲹��: ƽ����̬��������(���control.h, ����-22��->0��)
}
/**************************************************************************
Function: Absolute value function
Input   : a��Number to be converted
Output  : unsigned int
�������ܣ�����ֵ����
��ڲ�����a����Ҫ�������ֵ����
����  ֵ���޷�������
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration��Z-axis acceleration��Angle��The angle of balance��encoder_left��Left encoder count��encoder_right��Right encoder count
Output  : 1��picked up  0��No action
�������ܣ����С���Ƿ�����
��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
����  ֵ��1:С��������  0��С��δ������
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //��һ��
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<300)               //����1��С���ӽ���ֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //����ڶ���
	 {
			if(++count1>200)       count1=0,flag=0;                       //��ʱ���ٵȴ�2000ms�����ص�һ��
			if(Acceleration>30000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //����2��С������0�ȸ���������
				flag=2;			
	 } 
	  if(flag == 2)
	 {
		  if(++count2>100)       count2=0,flag=0;                    //��ʱ���ٵȴ�1000ms
		  if(myabs(encoder_left+encoder_right)>3000)                   //����3��С������̥��Ϊ�������ﵽ����ת��
			{
				flag=0;
				return 1;                                                //��⵽С��������
			}
	 }		
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance��Left encoder count��Right encoder count
Output  : 1��put down  0��No action
�������ܣ����С���Ƿ񱻷���
��ڲ�����ƽ��Ƕȣ���������������ұ���������
����  ֵ��1��С������   0��С��δ����
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Pick_up_stop==0)                     //��ֹ���      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //����1��С������0�ȸ�����
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<100&&encoder_right<100) //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
			{ 
				flag=0;
				flag=0;
				return 1;                        //��⵽С�������� 
			}				                        
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
}

/**************************************************************************
Function: Normal
Input   : none
Output  : none
�������ܣ��״�ǰ������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Normal(void)
{
	u8 j;
		if(Mode == Normal_Mode)									  //��ͨ�Ŀ���ģʽ�ɽ����ֱ�����
	{				
		for(j=0;j<225;j++) 	
	 {
		 Distance = Dataprocess[j].distance;           //����ģʽ����OLED��ʾ����
	 }
	 if(PS2_ON_Flag == RC_ON)		  					 //�����ֱ�����ʱ�����Ȱ�start������Ȼ��������ҡ��ֱ������ PS2 ����
			PS2_Control();
	 else if(Remote_ON_Flag==RC_ON)
		 Remote_Control();
	}
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
�������ܣ��״�ǰ������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i;
	u8 avoid_Num=0;//��Ҫ���ϵĵ�
	float Angle_Sum=0;//ȷ���ϰ�������һ����ı���
	u8 too_close = 0;//�ж��ϰ����Ƿ�̫���ı���
	if(Mode==Lidar_Avoid_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			if((Dataprocess[i].angle<avoid_Angle1)||(Dataprocess[i].angle>avoid_Angle2))//С��ǰ������100�ȷ�Χ
			{		
				if((Dataprocess[i].distance>0)&&(Dataprocess[i].distance<avoid_Distance))//����С��300mm��Ҫ����
				{
					Distance=Dataprocess[i].distance;
					avoid_Num++;
					if(Dataprocess[i].angle>310) Angle_Sum += (Dataprocess[i].angle-360);
					else if(Dataprocess[i].angle<50) Angle_Sum+= Dataprocess[i].angle;
					if(Dataprocess[i].distance<150)			too_close++;//����̫������Ҫ����
				}
			}
		}
		if(avoid_Num<8)
		{
		  Move_X=avoid_speed;                                           //��С��һ��200mm/s���ٶȣ���Ҫ����800
			Move_Z=0;
		}
		else if(avoid_Num>8)
		{
			 Move_X=0;
	    	if(too_close>10) Move_X=-avoid_speed,Move_Z=0;      //����̫��������һ��
				else
				{
					if(Angle_Sum>0)      
					{
						Move_Z=-turn_speed;//�ϰ��￿�ң���ת
					}
					else   Move_Z=turn_speed; //�ϰ��￿����ת
				}					
		}
 }
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
�������ܣ��״����ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Follow(void)
{
	u8 i;
	u8 follow_num=0;                //�жϸ���ĵ�
	u16 mini_distance = 65535;      //Ҫ����ľ��룬������С�����ľ���
	static float angle =0;                 //�����ĽǶ�
	static float last_angle = 0;           //��������һ���Ƕ�
	u8 data_count = 0;
	u16 Follow_distance=1500;        //������Զ����1500mm
	if(Mode==Lidar_Follow_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			 if((0<Dataprocess[i].distance) && (Dataprocess[i].distance<Follow_distance))//��0~1500mm��ѡ������ĵ�������
			 {
				 follow_num++;
				 if(Dataprocess[i].distance<mini_distance)                  //�жϳ���С����ĵ�
				 {
					 mini_distance = Dataprocess[i].distance;
					 angle = Dataprocess[i].angle;
					 Distance = mini_distance;                                     //��oled����ʾҪ�����ľ���
				 }
			 }
	  }
	if(angle>180)
		  angle -= 360;				//0--360��ת����0--180��-180--0��˳ʱ�룩
	if(angle-last_angle>13 ||angle-last_angle<-13)	//��һ����������������10�ȵ���Ҫ���ж�
	{
		if(++data_count == 60)		//����60�βɼ�����ֵ(300ms��)���ϴεıȴ���10�ȣ���ʱ������Ϊ����Чֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//����С��10�ȵĿ���ֱ����Ϊ����Чֵ
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(follow_num>5) 	
	{
		Move_X=Lidar_follow_PID(mini_distance,300);//����ľ���pidʱֱ���������ٶȻ�������Ҫ��Сһ��(Move�ķ�Χ��0~800)
		Move_Z=Follow_Turn_PID(angle,0);//ת��PIDֱ��������ת��
	}
	else
	{
		Move_X = 0;
		Move_Z = 0;
	}
	if(Move_X>600)    Move_X=600;
 }
}
/**************************************************************************
Function: Lidar_Straight
Input   : none
Output  : none
�������ܣ��״���ֱ��ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Straight(void) 
{
	static u16 target_distance=0;
	u8 i;
	u16 current_distance=target_distance;
	static u16 Limit_distance=0;   //�״�����̽�����
	if(Mode==Lidar_Straight_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		 Move_X=Initial_speed;//��С��һ����ʼ�ٶ�
		 for(i=0;i<225;i++)
	  {
		  if((Dataprocess[i].angle>71)&&(Dataprocess[i].angle<74))//ȡ�״��70��75�ȷ�Χ�ĵ����Ƚϵ�
		 {
			 if(determine<Limit_time) //��ģʽת����Straightģʽ3���ȷ��������Ҫ�ľ���
			 {
				 target_distance=Dataprocess[i].distance;
				 Limit_distance=target_distance+200;//��Ŀ������200mm,��Ҫ������������ʧ����С������ת��
				 determine++;
				 if(determine==(Limit_time-1)) determine=Limit_time;
			 }
			 if(Dataprocess[i].distance<Limit_distance)//����һ���״��̽�����
			 {
				 current_distance=Dataprocess[i].distance;//ȷ������
			   Distance=Dataprocess[i].distance;
			 }
		 }
	 }
	 Move_Z=Distance_Adjust_PID(current_distance,target_distance);//�״����pid
	}
}

/**************************************************************************
Function: Select_Zhongzhi
Input   : none
Output  : none
�������ܣ�С����е��ֵ��ѡ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Select_Zhongzhi(void)                             //��е��ֵѡ�񣬱��ⰲװ�ϵ��Ѳ�ߡ�CCDѲ��װ��ʱС����ǰ�������
{
	if(Mode == ELE_Line_Patrol_Mode)
		Middle_angle = -9 + ANGLE_ZERO_OFFSET;   //���Ѳ��
	else if(Mode == CCD_Line_Patrol_Mode)
		Middle_angle = -4 + ANGLE_ZERO_OFFSET;   //CCDѲ��
	else if(Mode == Track_Line_Patrol_Mode)
		Middle_angle = 0 + ANGLE_ZERO_OFFSET;    //����Ѳ��: ģ�����,��ֵ�ӽ�0,�ɰ�ʵ�ʻ�е���΢��
	else   Middle_angle = 2 + ANGLE_ZERO_OFFSET;     //��ͨģʽ
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=200;
    int Threshold=100;

	  //limiter //�޷�
    int LX,RY; 
//	  static float Target_LX,Target_LY,Target_RY,Target_RX;
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);

		// Front and back direction of left rocker. Control forward and backward.
	  //��ҡ��ǰ���򡣿���ǰ�����ˡ�
       LX=Remoter_Ch2-1500;
	
//		//Left joystick left and right. Control left and right movement.
//	  //��ҡ�����ҷ��򡣿��������ƶ�����
//      LY=Remoter_Ch2-1500;
	
		 //Right stick left and right. To control the rotation. 
		//��ҡ�����ҷ��򡣿�����ת��
	  RY=Remoter_Ch1-1500;		//
	
   

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
		
		
//		if(LX==0) Target_LX=Target_LX/1.2f;
//		if(LY==0) Target_LY=Target_LY/1.2f;
//		if(RY==0) Target_RY=Target_RY/1.2f;
		
		
//		//Throttle related //�������
//		Remote_RCvelocity=RC_Velocity+RX;
//	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//�Ժ�ģң�ؿ���������д���
        Move_X= LX; 
		Move_Z= RY; 
        Move_X= Move_X*1.8;//*1.3��Ϊ�������ٶ�
        Move_Z=Move_Z*2.3;
			 		
		//Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
      if(thrice>0) Move_X=0,Move_Z=0,thrice--;

}


