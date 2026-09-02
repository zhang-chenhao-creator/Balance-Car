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
#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PAin(0)
#define KEY_ON	1
#define KEY_OFF	0
//用户按键返回值状态
#define No_Action 					0
#define Click 						1
#define Long_Press 					2
#define Double_Click				3
#define KEY2_STATE  		 PCin(13)

uint8_t User_Key_Scan(void);
void Mode_Choose(void);
#endif  
