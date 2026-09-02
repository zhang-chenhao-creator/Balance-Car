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

#include "Lidar.h"
#include <string.h>
#include "sys.h"



float Follow_KP =10,Follow_KD =1,Follow_KI = 0.001;	

PointDataProcessDef PointDataProcess[225] ;//更新225个数据
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[225];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据


/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
//完成一帧接收后进行处理
void data_process(void) //数据处理
{
	static u8 data_cnt = 0;
	u8 i,m,n;
	u32 distance_sum[8]={0};//2个点的距离和的数组
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//计算16个点的开始角度
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//计算16个点的结束角度
	float area_angle[8]={0};//一帧数据的8个平均角度
	Lidar_flag=1;
	if((start_angle>350)&&(end_angle<13))//因为一帧数据是13度，避免350到10这段范围相加，最后angle反而变成180范围
		end_angle +=360;
	for(m=0;m<8;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/8*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	for(i=0;i<16;i++)
	{
		switch(i)
		{
			case 0:case 1:
				distance_sum[0] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//0~1点的距离和
			  break;
			case 2:case 3:
				distance_sum[1] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//2~3点的距离和
			  break;
			case 4:case 5:
				distance_sum[2] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//4~5点的距离和
			  break;
			case 6:case 7:
				distance_sum[3] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//6~7点的距离和
			  break;
			case 8:case 9:
				distance_sum[4] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//8~9点的距离和
			  break;
			case 10:case 11:
				distance_sum[5] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//10~11点的距离和
			  break;
			case 12:case 13:
				distance_sum[6] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//12~13点的距离和
			  break;
			case 14:case 15:
				distance_sum[7] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//14~15点的距离和
			  break;
			default:break;
								
		}
		  
	}
	for(n=0;n<8;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	  PointDataProcess[data_cnt+n].distance = distance_sum[n]/2;
	}
	data_cnt +=8;
	if(data_cnt>=225)
	{
		for(i=0;i<225;i++)
		{
			Dataprocess[i].angle=PointDataProcess[i].angle;
			Dataprocess[i].distance=PointDataProcess[i].distance;
		}
		data_cnt = 0;
	}
		
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//雷达距离调整pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	OutPut=-Distance_KP*Bias/100-Distance_KD*(Bias-Last_Bias)/100;//位置式PID控制器  //
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	return OutPut;                                          	
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：雷达跟随距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	

float Lidar_follow_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	OutPut=-1.5*Bias-1*(Bias-Last_Bias);//位置式PID控制器  //
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/	 	
//雷达转向pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)		                                 				 //求出偏差的积分

{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //计算偏差
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=-Follow_KP*Bias-Follow_KI*Integral_bias-Follow_KD*(Bias-Last_Bias);	//位置式PID控制器
	Last_Bias=Bias;                                       					 		//保存上一次偏差
	if(Turn_Off(Angle_Balance,Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                           					 	//输出
	
}




