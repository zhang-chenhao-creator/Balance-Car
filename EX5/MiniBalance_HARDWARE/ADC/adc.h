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
// ADC 通道宏定义
#define    ELE_ADC_L_CHANNEL					 ADC_Channel_4
#define    ELE_ADC_M_CHANNEL					 ADC_Channel_5
#define    ELE_ADC_R_CHANNEL					 ADC_Channel_15
#define    CCD_ADC_CHANNEL					 	 ADC_Channel_15

#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define Battery_Ch 11
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
u16 Get_Adc1(u8 ch);
#endif 















