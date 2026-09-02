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
#include "key.h"
#include "TrackModule.h"


/*************************************************************************
Function:Mode Choose
Iput    :None
Output  :Noce
函数功能：小车模式选择
入口参数：无
返回  值：无
*************************************************************************/
void Mode_Choose(void)
{
	switch(User_Key_Scan())
		{
			//单击按键可以切换到
			//1.普通遥控模式
			//2.雷达巡航模式
			//3.雷达跟随模式
			//4.电磁巡线模式
			//5.CCD巡线模式
			//长按按键进入上位机
			case Click:
				/* The short-term project is fixed to IRDM line patrol. */
				Mode = Track_Line_Patrol_Mode;
				break;
			case Long_Press:
				Flag_Show = !Flag_Show;								//长按 进入/退出 上位机模式
				break;
			case Double_Click:										
				if((Mode == ELE_Line_Patrol_Mode)||(Mode==CCD_Line_Patrol_Mode))					//巡线状态时，双击可以打开/关闭雷达检测障碍物，默认打开
				{
					Lidar_Detect = !Lidar_Detect;
					if(Lidar_Detect == Lidar_Detect_OFF)
						memset(PointDataProcess,0, sizeof(PointDataProcessDef)*40);		//用于雷达检测障碍物的数组清零
				}
				break;
			default:break;
		}
}

/*************************************************************************
Function:User_Key_Scan
Input:None
Output:Key_status
函数功能：用户按键检测
入口参数：无
返回值  ：按键状态
**************************************************************************/
//放在5ms中断中调用
uint8_t User_Key_Scan(void)
{
	static u16 count_time = 0;					//计算按下的时间，每5ms加1
	static u8 key_step = 0;						//记录此时的步骤
	switch(key_step)
	{
		case 0:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == KEY_ON )
				key_step++;						//检测到有按键按下，进入下一步
			break;
		case 1:
			if((++count_time) == 5)				//延时消抖
			{
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == KEY_ON )//按键确实按下了
					key_step++,count_time = 0;	//进入下一步
				else
					count_time = 0,key_step = 0;//否则复位
			}
			break;
		case 2:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == KEY_ON )
				count_time++;					//计算按下的时间
			else 								//此时已松开手
				key_step++;						//进入下一步
			break;
		case 3:									//此时看按下的时间，来判断是长按还是短按
			if(count_time > 400)				//在5ms中断中调用，故按下时间若大于400*5 = 2000ms（大概值）
			{							
				key_step = 0;					//标志位复位
				count_time = 0;
				return Long_Press;				//返回 长按 的状态 
 			}
			else if(count_time > 5)				//此时是单击了一次
			{
				key_step++;						//此时进入下一步，判断是否是双击
				count_time = 0;					//按下的时间清零
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//判断是否是双击或单击
			if(++count_time >20&&count_time<70)				//5*50= 250ms内判断按键是否按下
			{
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == KEY_ON )	//按键确实按下了
				{																	//这里双击不能按太快，会识别成单击
					key_step++;														//进入下一步，需要等松手才能释放状态
					count_time = 0;
				}
				else																//190ms内无按键按下，此时是单击的状态
				{
					if(count_time>65)
					{
						key_step = 0;				//标志位复位
						count_time = 0;					
						return Click;				//返回单击的状态
					}
				}
			}
			break;
		case 5:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == KEY_ON )//按键还在按着
			{
				count_time++;
			}
			else								//按键已经松手
			{
//				if(count_time>400)				//这里第二次的单击也可以判断时间的，默认不判断时间，全部都返回双击
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//无动作
}
