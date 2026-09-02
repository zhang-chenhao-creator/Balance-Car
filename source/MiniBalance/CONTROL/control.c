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
#include "control.h"	//控制函数
#include "TrackModule.h"	//红外巡线模块(4路数字量寻迹,与PS2共用扩展接口)
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         5ms外部中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
入口参数：无
返回  值：无				 
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{ 		
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
	if(GPIO_Pin==GPIO_PIN_9)
	{
		if(Time_count < START_DELAY)   //开机前3秒(600*5ms): MPU6050数据不稳定, 禁用数据与控制
		{
			Time_count++;
			if(delay_flag==1)          //维持主循环50ms节拍(DataScope示波器用),避免死等
			{
				if(++delay_50==10) delay_50=0,delay_flag=0;
			}
			Set_Pwm(0,0);                //稳定期内电机强制停转(差分PWM=7200/7200)
			return;
		}
		Flag_Target=!Flag_Target;
		Encoder_Left=Read_Encoder(4);            					  //读取左轮编码器的值，前进为正，后退为负
		Encoder_Right=-Read_Encoder(8);           					//读取右轮编码器的值，前进为正，后退为负
																												//左轮A相接TIM4_CH1,右轮A相接TIM8_CH1,故这里两个编码器的极性不相同
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		Mode_Choose();                                      //小车模式的选择
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//编码器读数转速度（mm/s）
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
		}
		if(++Ros_count == 10)
		{	
            Lidar_flag=0;			
			Ros_send_flag=1;
			Ros_count=0;
		}
		if(Flag_Target==1)                        					//10ms控制一次
		{		
			Voltage_Temp=Get_battery_volt();		    					//读取电池电压		
			Voltage_Count++;                       						//平均值计数器
			Voltage_All+=Voltage_Temp;              					//多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值			                                               
		}                                         					//10ms控制一次
        if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)		
	       Read_Distane();                                  //超声波读取距离   
		Select_Zhongzhi();                                  //机械中值的选择
		Normal();                                           //普通模式
		Lidar_Avoid();                                      //雷达避障模式
		Lidar_Follow();                                     //雷达跟随模式
		Lidar_Straight();                                   //雷达走直线模式 
		ELE_Mode();                                         //电磁循迹模式
		CCD_Mode();                                         //CCD巡线模式运行
		if(Mode==Track_Line_Patrol_Mode)  IRDM_line_inspection();//红外巡线(4路寻迹模块)
		if(Mode==Normal_Mode)	Led_Flash(100);               //LED闪烁;常规模式 1s改变一次指示灯的状态	
		else Led_Flash(0);                                  //LED常亮;其余模式	
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		if(Mode ==CCD_Line_Patrol_Mode)                     //CCD循迹下的转向环控制 
			Turn_Pwm=CCD_turn(CCD_Zhongzhi,Gyro_Turn);
		else if(Mode==ELE_Line_Patrol_Mode)
			Turn_Pwm=ELE_turn(Encoder_Left,Encoder_Right,Gyro_Turn);
		else
		  Turn_Pwm=Turn(Gyro_Turn);													//转向环PID控制     
		

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM
																												//PWM值正数使小车前进，负数使小车后退
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM限幅

		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起
			Pick_up_stop=1;	                           					//如果被拿起就关闭电机
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下
			Pick_up_stop=0;	                           					//如果被放下就启动电机
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器  
	 }       	  
} 

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制		
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM		
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
//修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral;
	  //================遥控前进后退部分====================// 
    if(Mode==Track_Line_Patrol_Mode)
			Movement=base_speed_mm/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //红外巡线: 目标速度=巡线模块base_speed_mm(mm/s)
		else if(Flag_front==1)		Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //收到前进信号
		else if(Flag_back==1)	Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //收到后退信号
	  else  Movement=Move_X/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;//将给定速度转化为电机的编码器读数单位;
          //Movement=Move_X/周长/编码器读取频率*频数*减速比*精度*2;	
	  if(Movement>2400)  Movement=2400;                     //蓝牙遥控速度的限制，避免破坏小车的平衡
	  
	//=============超声波功能（跟随/避障）==================// 
	  if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //跟随
			 Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //收到前进信号
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //收到前进信号
		if(Mode==Ultrasonic_Avoid_Mode&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //超声波避障
			 Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //收到前进信号
		
   //================速度PI控制器=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
		Encoder_bias *= 0.84;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.16;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，控制前进后退
		if(Encoder_Integral>200000)  	Encoder_Integral=200000;            //积分限幅
		if(Encoder_Integral<-200000)	  Encoder_Integral=-200000;         //积分限幅	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制
    if(Mode == ROS_Mode)
		{ 
			if(++Ros_Rate>=100) Ros_Rate=0,Move_X=0;//如果ros端200ms内没有发送数据过来，Move_Z置0；这是为了ros端控制小车的更加顺畅
		}
		else Move_X	=0;	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//电机关闭后清除积分
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制 
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
作    者：轮趣科技（东莞）有限公司
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可
	//===================遥控左右旋转部分=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else if(Mode==Track_Line_Patrol_Mode)  Turn_Target=turn_diff*Track_Turn_Scale; //红外巡线: 转向差速(mm/s)*系数
		 else Turn_Target=0;
		 if(1==Flag_front||1==Flag_back||Mode==Track_Line_Patrol_Mode)  Kd=Turn_Kd; //红外巡线: 保留陀螺仪阻尼
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//结合Z轴陀螺仪进行PD控制
   if(Mode == ROS_Mode)
	 {
		 if(++Ros_Rate>=40) Ros_Rate=0,Move_Z=0;//如果ros端200ms没有发送数据过来，Move_Z置0；这是为了ros端控制小车的更加顺畅
	 }
	 else Move_Z=0;
	 return turn;								 				 //转向环PWM右转为正，左转为负
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)		
	{
		PWMA_IN1=7200;
		PWMA_IN2=7200-motor_left;//左轮前进
	}		
	else 
	{
		PWMA_IN1=7200+motor_left;
		PWMA_IN2=7200;
	} //左轮后退
  if(motor_right>0)			
	{
		PWMB_IN1=7200-motor_right;
		PWMB_IN2=7200;
	}		//右轮前进
	else 	        			  
	{
		PWMB_IN1=7200;
		PWMB_IN2=7200+motor_right;

	}//右轮后退
	
}
/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
函数功能：限制PWM赋值 
入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
返回  值：限幅后的值
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
Input   : angle：Car inclination；voltage：Voltage
Output  : 1：abnormal；0：normal
函数功能：异常关闭电机		
入口参数：angle：小车倾角；voltage：电压
返回  值：1：异常  0：正常
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	Flag_Stop = KEY2_STATE;
	if(KEY2_STATE==1) Pick_up_stop=0;                  //key2关闭，Pick_up_stop恢复为0
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000||Pick_up_stop==1)//电池电压低于11.1V关闭电机
	{	                                                 //倾角大于40度关闭电机
		temp=1;                                          //Flag_Stop置1，即单击控制关闭电机
		PWMA_IN1=0;                                      //Pick_up_stop置1，即小车基本静止，在0度左右拿起小车 
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
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
  float gyro_x,gyro_y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
//		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
//		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
//		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
		gyro_x=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
		gyro_y=Gyro_Y/16.4;                              //陀螺仪量程转换	
		if(Way_Angle==2)		  	
		{
			 Pitch = -Kalman_Filter_x(Accel_Angle_x,gyro_x);//卡尔曼滤波
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
	}
	Angle_Balance += ANGLE_ZERO_OFFSET;            //零点补偿: 平衡姿态读数归零(宏见control.h, 例如-22°->0°)
}
/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
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
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<300)               //条件1，小车接近静止
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //进入第二步
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时不再等待2000ms，返回第一步
			if(Acceleration>30000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2，小车是在0度附近被拿起
				flag=2;			
	 } 
	  if(flag == 2)
	 {
		  if(++count2>100)       count2=0,flag=0;                    //超时不再等待1000ms
		  if(myabs(encoder_left+encoder_right)>3000)                   //条件3，小车的轮胎因为正反馈达到最大的转速
			{
				flag=0;
				return 1;                                                //检测到小车被拿起
			}
	 }		
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Pick_up_stop==0)                     //防止误检      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是在0度附近的
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<100&&encoder_right<100) //条件2，小车的轮胎在未上电的时候被人为转动  
			{ 
				flag=0;
				flag=0;
				return 1;                        //检测到小车被放下 
			}				                        
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}

/**************************************************************************
Function: Normal
Input   : none
Output  : none
函数功能：雷达前进避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Normal(void)
{
	u8 j;
		if(Mode == Normal_Mode)									  //普通的控制模式可进行手柄控制
	{				
		for(j=0;j<225;j++) 	
	 {
		 Distance = Dataprocess[j].distance;           //正常模式下在OLED显示距离
	 }
	 if(PS2_ON_Flag == RC_ON)		  					 //开启手柄控制时，需先按start按键，然后上拉左摇杆直到出现 PS2 字样
			PS2_Control();
	 else if(Remote_ON_Flag==RC_ON)
		 Remote_Control();
	}
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
函数功能：雷达前进避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i;
	u8 avoid_Num=0;//需要避障的点
	float Angle_Sum=0;//确认障碍物在哪一方向的变量
	u8 too_close = 0;//判断障碍物是否太近的变量
	if(Mode==Lidar_Avoid_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			if((Dataprocess[i].angle<avoid_Angle1)||(Dataprocess[i].angle>avoid_Angle2))//小车前进方向100度范围
			{		
				if((Dataprocess[i].distance>0)&&(Dataprocess[i].distance<avoid_Distance))//距离小于300mm需要避障
				{
					Distance=Dataprocess[i].distance;
					avoid_Num++;
					if(Dataprocess[i].angle>310) Angle_Sum += (Dataprocess[i].angle-360);
					else if(Dataprocess[i].angle<50) Angle_Sum+= Dataprocess[i].angle;
					if(Dataprocess[i].distance<150)			too_close++;//靠得太近，需要后退
				}
			}
		}
		if(avoid_Num<8)
		{
		  Move_X=avoid_speed;                                           //给小车一个200mm/s的速度，不要大于800
			Move_Z=0;
		}
		else if(avoid_Num>8)
		{
			 Move_X=0;
	    	if(too_close>10) Move_X=-avoid_speed,Move_Z=0;      //靠的太近，后退一点
				else
				{
					if(Angle_Sum>0)      
					{
						Move_Z=-turn_speed;//障碍物靠右，左转
					}
					else   Move_Z=turn_speed; //障碍物靠左，右转
				}					
		}
 }
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
函数功能：雷达跟随模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Follow(void)
{
	u8 i;
	u8 follow_num=0;                //判断跟随的点
	u16 mini_distance = 65535;      //要跟随的距离，就是最小距离点的距离
	static float angle =0;                 //跟随点的角度
	static float last_angle = 0;           //跟随点的上一个角度
	u8 data_count = 0;
	u16 Follow_distance=1500;        //跟随最远距离1500mm
	if(Mode==Lidar_Follow_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			 if((0<Dataprocess[i].distance) && (Dataprocess[i].distance<Follow_distance))//在0~1500mm中选择最近的点来跟随
			 {
				 follow_num++;
				 if(Dataprocess[i].distance<mini_distance)                  //判断出最小距离的点
				 {
					 mini_distance = Dataprocess[i].distance;
					 angle = Dataprocess[i].angle;
					 Distance = mini_distance;                                     //在oled上显示要跟随点的距离
				 }
			 }
	  }
	if(angle>180)
		  angle -= 360;				//0--360度转换成0--180；-180--0（顺时针）
	if(angle-last_angle>13 ||angle-last_angle<-13)	//做一定消抖，波动大于10度的需要做判断
	{
		if(++data_count == 60)		//连续60次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//波动小于10度的可以直接认为是有效值
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(follow_num>5) 	
	{
		Move_X=Lidar_follow_PID(mini_distance,300);//这个的距离pid时直接作用在速度环，所以要变小一点(Move的范围在0~800)
		Move_Z=Follow_Turn_PID(angle,0);//转向PID直接作用在转向环
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
函数功能：雷达走直线模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Straight(void) 
{
	static u16 target_distance=0;
	u8 i;
	u16 current_distance=target_distance;
	static u16 Limit_distance=0;   //雷达最大的探测距离
	if(Mode==Lidar_Straight_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		 Move_X=Initial_speed;//给小车一个初始速度
		 for(i=0;i<225;i++)
	  {
		  if((Dataprocess[i].angle>71)&&(Dataprocess[i].angle<74))//取雷达的70到75度范围的点做比较点
		 {
			 if(determine<Limit_time) //在模式转换到Straight模式3秒后确定我们想要的距离
			 {
				 target_distance=Dataprocess[i].distance;
				 Limit_distance=target_distance+200;//比目标距离大200mm,主要避免参照物的消失导致小车快速转向
				 determine++;
				 if(determine==(Limit_time-1)) determine=Limit_time;
			 }
			 if(Dataprocess[i].distance<Limit_distance)//限制一下雷达的探测距离
			 {
				 current_distance=Dataprocess[i].distance;//确定距离
			   Distance=Dataprocess[i].distance;
			 }
		 }
	 }
	 Move_Z=Distance_Adjust_PID(current_distance,target_distance);//雷达距离pid
	}
}

/**************************************************************************
Function: Select_Zhongzhi
Input   : none
Output  : none
函数功能：小车机械中值的选择
入口参数：无
返回  值：无
**************************************************************************/
void Select_Zhongzhi(void)                             //机械中值选择，避免安装上电磁巡线、CCD巡线装备时小车往前冲的现象
{
	if(Mode == ELE_Line_Patrol_Mode)
		Middle_angle = -9 + ANGLE_ZERO_OFFSET;   //电磁巡线
	else if(Mode == CCD_Line_Patrol_Mode)
		Middle_angle = -4 + ANGLE_ZERO_OFFSET;   //CCD巡线
	else if(Mode == Track_Line_Patrol_Mode)
		Middle_angle = 0 + ANGLE_ZERO_OFFSET;    //红外巡线: 模块较轻,中值接近0,可按实际机械情况微调
	else   Middle_angle = 2 + ANGLE_ZERO_OFFSET;     //普通模式
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=200;
    int Threshold=100;

	  //limiter //限幅
    int LX,RY; 
//	  static float Target_LX,Target_LY,Target_RY,Target_RX;
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);

		// Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
       LX=Remoter_Ch2-1500;
	
//		//Left joystick left and right. Control left and right movement.
//	  //左摇杆左右方向。控制左右移动。。
//      LY=Remoter_Ch2-1500;
	
		 //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
	  RY=Remoter_Ch1-1500;		//
	
   

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
		
		
//		if(LX==0) Target_LX=Target_LX/1.2f;
//		if(LY==0) Target_LY=Target_LY/1.2f;
//		if(RY==0) Target_RY=Target_RY/1.2f;
		
		
//		//Throttle related //油门相关
//		Remote_RCvelocity=RC_Velocity+RX;
//	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
        Move_X= LX; 
		Move_Z= RY; 
        Move_X= Move_X*1.8;//*1.3是为了阔大速度
        Move_Z=Move_Z*2.3;
			 		
		//Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
      if(thrice>0) Move_X=0,Move_Z=0,thrice--;

}


