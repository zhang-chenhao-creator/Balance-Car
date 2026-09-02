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
#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
extern int Time_count;

//============================================================
// 角度零点补偿: 把小车扶到平衡(直立)姿态, 记下OLED显示的度数X°(例: -22°),
// 此处设为 -X (即 22)。补偿后平衡姿态显示为 0°, OLED/手机APP/PC控制端
// 同步归零; 控制目标Middle_angle会按相同量自动平移, 小车控制行为不变。
//============================================================
#define ANGLE_ZERO_OFFSET   22

//============================================================
// 开机稳定期: MPU6050上电后前几秒数据不稳定。
// 该宏为5ms中断计数次数, 600*5ms=3秒。期间禁用MPU6050数据
// 与控制(电机保持关闭), 避免启动瞬间轮子转速异常。
//============================================================
#define START_DELAY         600


#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  500.0 	//编码器精度 500线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm


#define DIFFERENCE 100
#define INT PBin(9)   //PB9连接到MPU6050的中断引脚


//小车各模式定义
#define Normal_Mode							0
#define Lidar_Avoid_Mode					1
#define Lidar_Follow_Mode					2
#define Lidar_Straight_Mode                 3
#define ELE_Line_Patrol_Mode				4
#define CCD_Line_Patrol_Mode				5
#define ROS_Mode				            6
#define Ultrasonic_Avoid_Mode               7
#define Ultrasonic_Follow_Mode              8
#define Track_Line_Patrol_Mode			9

//避障模式的参数
#define  avoid_Distance 300//避障距离300mm
#define avoid_Angle1 50 //避障的角度，在310~360、0~50°的范围
#define avoid_Angle2 310
#define avoid_speed 200    //避障速度
#define turn_speed 800;    //避障转向速度

//雷达走直线的参数
#define Initial_speed 200//小车的初始速度大概为200mm每秒
#define Limit_time 500   //限制时间，5ms中断*数值=时间 ，这里就是3s
#define refer_angle1  71 //参照物的角度1
#define refer_angle2  74 //参照物的角度2

int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Middle_angle_Check(void);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);
void Read_distance(void);
void Lidar_Avoid(void);
void Lidar_Follow(void);
void Lidar_Straight(void);
void Read_Distane(void);
void Lidar_Avoid(void);
void Select_Zhongzhi(void);
void Normal(void);
void Remote_Control(void);
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;

#endif

