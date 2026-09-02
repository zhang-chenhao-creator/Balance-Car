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

#include "pstwo.h"
#define DELAY_TIME  delay_us(5); 
u16 Handkey;	// 按键值读取，临时存储。
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 


/**************************************************************************
Function: Sent_Cmd
Input   : none
Output  : none
函数功能: 向手柄发送命令
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                  	 //输出一位控制位
		}
		else DO_L;

		CLK_H;                        //时钟拉高
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
//判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
//返回值；0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}

//对读出来的PS2的数据进行处理,只处理按键部分  
//只有一个按键按下时按下为0， 未按下为1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
//进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//振动设置
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
//完成并保存配置
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
//手柄配置初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	//PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
}



/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
函数功能：读取PS2手柄的控制量
入口参数：无
返回  值：无
**************************************************************************/
void PS2_Read(void)
{
	static int Strat;
	//Reading key
  //读取按键键值
	PS2_KEY=PS2_DataKey(); 
	//Read the analog of the remote sensing x axis on the left
  //读取左边遥感X轴方向的模拟量	
	PS2_LX=PS2_AnologData(PSS_LX); 
	//Read the analog of the directional direction of remote sensing on the left
  //读取左边遥感Y轴方向的模拟量	
	PS2_LY=PS2_AnologData(PSS_LY);
	//Read the analog of the remote sensing x axis
  //读取右边遥感X轴方向的模拟量  
	PS2_RX=PS2_AnologData(PSS_RX);
	//Read the analog of the directional direction of the remote sensing y axis
  //读取右边遥感Y轴方向的模拟量  
	PS2_RY=PS2_AnologData(PSS_RY);  

	if(PS2_KEY==4&&PS2_ON_Flag==0) 
		//The start button on the // handle is pressed
		//手柄上的Start按键被按下
		Strat=1; 
	//When the button is pressed, you need to push the right side forward to the formal ps2 control car
	//Start按键被按下后，需要推下左边前进杆，才可以正式PS2控制小车
	if(Strat&&(PS2_LY<118)&&PS2_ON_Flag==0)
	{
		PS2_ON_Flag=RC_ON;
		Remote_ON_Flag = RC_OFF;
		RC_Velocity = Default_Velocity;
	  RC_Turn_Velocity = Default_Turn_Bias;
	}
	
}

/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
函数功能：PS2手柄控制
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void PS2_Control(void)
{
	int LY,RX;									//手柄ADC的值
	int Threshold=20; 							//阈值，忽略摇杆小幅度动作
	static u8 Key1_Count = 0,Key2_Count = 0;	//用于控制读取摇杆的速度
	//转化为128到-128的数值
	LY=-(PS2_LY-128);//左边Y轴控制前进后退
	RX=-(PS2_RX-128);//右边X轴控制转向

	if(LY>-Threshold&&LY<Threshold)	LY=0;
	if(RX>-Threshold&&RX<Threshold)	RX=0;		//忽略摇杆小幅度动作
	
	Move_X = (RC_Velocity/128)*LY;				//速度控制，力度表示速度大小

	if(Move_X>=0)
		Move_Z = -(RC_Turn_Velocity/128)*RX;	//转向控制，力度表示转向速度
	else
		Move_Z = (RC_Turn_Velocity/128)*RX;
	if (PS2_KEY == PSB_L1) 					 	//按下左1键加速（按键在顶上）
	{	
		if((++Key1_Count) == 20)				//调节按键反应速度
		{
			PS2_KEY = 0;
			Key1_Count = 0;
			if((RC_Velocity += X_Step)>MAX_RC_Velocity)				//前进后退最大速度800,大概为600mm/s
				RC_Velocity = MAX_RC_Velocity;

				if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//转向最大速度2000
					RC_Turn_Velocity = MAX_RC_Turn_Bias;
			}
	}
	else if(PS2_KEY == PSB_R1) 					//按下右1键减速
	{
		if((++Key2_Count) == 20)
		{
			PS2_KEY = 0;
			Key2_Count = 0;
			if((RC_Velocity -= X_Step)<MINI_RC_Velocity)			//前进后退最小速度100
				RC_Velocity = MINI_RC_Velocity;
			
			if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//转向最小速度800
			   RC_Turn_Velocity = MINI_RC_Turn_Velocity;
		}
	}
	else
		Key2_Count = 0,Key2_Count = 0;			//读取到其他按键重新计数
}


