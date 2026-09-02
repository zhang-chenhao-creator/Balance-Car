/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "usart3.h"

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
#define CONTROL_DELAY		1000
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

u8 Usart1_Receive_buf[1];           //串口1接收中断数据存放的缓存区
u8 Usart1_Receive;                  //从串口1读取的数据
u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                 //从串口3读取的数据
u8 Uart5_Receive_buf[1];          //串口5接收中断数据存放的缓冲区
u8 Uart5_Receive;                 //从串口5读取的数据
/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //接收回调函数
{	
	if(UartHandle->Instance == USART3) //接收到数据
	{	  
	  static	int uart_receive=0;//蓝牙接收相关变量
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=Usart3_Receive_buf[0]; 
		Usart3_Receive=uart_receive;
		if(uart_receive==0x59)  Target_Velocity-=100,Flag_velocity=2;  //减速
		if(uart_receive==0x58)  Target_Velocity+=100,Flag_velocity=1;  //加速
		
	  if(uart_receive>10)  //默认使用
    {			
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}
		if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
		{			
			Flag_velocity=1;//转向速度切换至高速档
			if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
														Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}	

		
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //分析数据
		 {
			  if(Receive[3]==0x50) 				 PID_Send=1;
			  else if(Receive[1]!=0x23) 
				{								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					{
						case 0x30:  Target_Velocity=Data;break;
						case 0x31:  Balance_Kp=Data;break;
						case 0x32:  Balance_Kd=Data;break;
						case 0x33:  Velocity_Kp=Data;break;
						case 0x34:  Velocity_Ki=Data;break;
						case 0x35:  Turn_Kp=Data;break; 
					  case 0x36:  Turn_Kd=Data;break; 
						case 0x37:  Distance_KP=Data;break;
						case 0x38:  Distance_KD=Data;break;
						default: break;
					}
				}				 
			    Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//数组清零
		 } 
		HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
	}  											 
	else if(UartHandle->Instance == UART5) //接收到数据
	{
		static u8 state = 0;//状态位	
		static u8 crc_sum = 0;//校验和
		static u8 cnt = 0;//用于一帧16个点的计数
		u8 temp_data;
		temp_data=Uart5_Receive_buf[0]; 
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//头固定
				{
					Pack_Data.header_0= temp_data;
					state++;
					//校验
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//头固定
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//字长固定
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//速度高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//速度低八位
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//开始角度高八位
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//开始角度低八位
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				Pack_Data.point[cnt].distance_h = temp_data;//16个点的距离数据，高字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				Pack_Data.point[cnt].distance_l = temp_data;//16个点的距离数据，低字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				Pack_Data.point[cnt].Strong = temp_data;//16个点的强度数据
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			
			case 55:
				Pack_Data.end_angle_h = temp_data;//结束角度的高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 56:
				Pack_Data.end_angle_l = temp_data;//结束角度的低八位
				state++;
				crc_sum += temp_data;
				break;
			case 57:
				Pack_Data.crc = temp_data;//校验
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//数据处理，校验正确不断刷新存储的数据
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//清零
				}
				crc_sum = 0;//校验和清零
				break;
			default: break;
	  }	
   HAL_UART_Receive_IT(&huart5,Uart5_Receive_buf,sizeof(Uart5_Receive_buf));//开启串口5接收中断		
	}
	else if(UartHandle->Instance == USART1)//串口一接收数据
	{
		static u8 Count=0;
		Usart1_Receive = Usart1_Receive_buf[0];//读取数据
		Mode=ROS_Mode;//ros控制时，将小车模式设为ROS模式
////		if(Time_count < CONTROL_DELAY)
////		{
////			//Data is not processed until 10 seconds after startup
////			//开机十秒前不允许接收数据
////		}
		Receive_Data.buffer[Count] = Usart1_Receive;//Fill the array with serial data  串口数据填入数组
		
		//Ensure that the first data is the array is FRAME_HEADER
		//确保第一个数据时帧头
		if(Usart1_Receive == FRAME_HEADER ||Count>0)
			Count++;
		else Count=0;
		if(Count == 11) //Verify the length of the packet  验证数据包的长度
		{
			Count = 0;
			if(Receive_Data.buffer[10] == FRAME_TAIL)
			{
				if(Receive_Data.buffer[9] == Check_Sum(9,0))
				{
					
					//Calculate the 3-axis target velocity from the serial data,which is divided into 8-bit high and 8-bit low units m/s
					Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);//平衡小车没有运用到运动学正解逆解
					Move_Z=-XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8])/1000*160/2/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision/Perimeter*5.6;//Move_Z是直接作用在转向环
					//Move_Z=ROS给的弧度/1000*轮距/2/周长/编码器读取频率*频数*减速比*精度；
					Ros_Rate=0;  //只要中断进来，就把它置0，防止小车控制的卡顿
				}
			}
		}
		HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf));//开启串口1接收中断
	}
} 

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//根据运动学算法进行运动学正解，从各车轮速度求出三轴速度
	
	Send_Data.Sensor_Str.X_speed = ((Velocity_Left+Velocity_Right)/2); //平衡小车的速度是mm/s
	Send_Data.Sensor_Str.Y_speed = 0;
	Send_Data.Sensor_Str.Z_speed = ((Velocity_Right-Velocity_Left)/1000/0.160*1000);//0.160为轮距
	
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= -Accel_Y; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=Accel_X; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= Accel_Z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= -Gyro_Y*4; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data= Gyro_X*4; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data=Gyro_Z*4;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将整数放大十倍传输，相应的在接收端在接收到数据后会缩小一千倍得到正常显示的电压，如检测Voltage为1140，放大十倍后再缩小一千倍为11.40)
	Send_Data.Sensor_Str.Power_Voltage = (float)Voltage*10; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //小车软件失能标志位
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed>>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed>>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage>>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return  transition; 						
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(mode==1)
	for(k=0;k<Count_Number;k++)
	{
	  check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(mode==0)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}
