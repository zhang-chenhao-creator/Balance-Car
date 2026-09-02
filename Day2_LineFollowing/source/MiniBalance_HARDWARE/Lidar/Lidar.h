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

#ifndef __LIDAR_H
#define	__LIDAR_H

#include "sys.h"



#define Lidar_Follow_Mode					2
#define Lidar_Avoid_Mode					1

#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_ 0x3A

#define POINT_PER_PACK 16






typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

typedef struct PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;


extern PointDataProcessDef PointDataProcess[225];//更新225个数据
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef Dataprocess[225];//用于小车避障、跟随、走直线、ELE雷达避障的雷达数据

void LIDAR_USART_Init(void);
void  UART5_IRQHandler(void);
void data_process(void);
float Distance_Adjust_PID(float Current_Distance,float Target_Distance);
void Get_Target_Encoder(float Vx,float Vz);
int Incremental_PI_Left (int Encoder,int Target);
int Incremental_PI_Right (int Encoder,int Target);
float Follow_Turn_PID(float Current_Angle,float Target_Angle);
float Lidar_follow_PID(float Current_Distance,float Target_Distance);//距离调整PID
#endif


