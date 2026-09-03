/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

说明：本文件移植自
  STM32F103RCT6_通用巡线例程_C10B / STM32F103RCT6_椭圆巡线例程_C10B
  (MiniBalance_HARDWARE/TrackModule)，两例程算法一致，仅参数不同：
    通用例程: BaseSpeed=250 TurnMaxAngle=65 TurnMidAngle=40 ForwardLimit=50
    椭圆例程: BaseSpeed=400 TurnMaxAngle=45 TurnMidAngle=25 ForwardLimit=80
  已适配本工程(平衡小车 HAL库cube版)：
    * GPIO 初始化改为 HAL 库风格
    * 输出由"左右电机目标速度"改为"前方目标速度+转向差速"，供平衡小车的
      速度环/转向环使用
    * 巡线模块与 PS2 手柄共用扩展接口(PB8/PC8/PC4/PC9)，两者不能同时使用，
      进入/退出巡线模式时切换引脚功能
All rights reserved
***********************************************/
#ifndef __TRACKMODULE_H
#define __TRACKMODULE_H
#include "sys.h"

/*============================================================================*
 * 传感器引脚定义（4路数字量红外巡线模块，接扩展接口，与PS2手柄共用）		 *
 * 物理从左到右：DH4(左外) DH3(左中) DH2(右中) DH1(右外)					 *
 * 白底为1，黑线为0（四路全白=1111，居中=白黑黑白）						 *
 *============================================================================*/
#define DH4 PBin(8)
#define DH3 PCin(9)
#define DH2 PCin(4)
#define DH1 PCin(8)

/*==============================可调参数(全局变量,可在线调参)=================*/
extern float Turn90Angle ;   // 直角弯转向参数
extern float TurnMaxAngle;   // 大弯道转向参数
extern float TurnMidAngle;   // 中等转向参数（丢线时使用）
extern float TurnMinAngle;   // 微调转向参数
extern float BaseSpeed;      // 基础巡线速度（直行时的速度，单位mm/s）
extern float FineSpeed;       // 微调状态目标速度（单位mm/s）
extern float CurveSpeed;      // 普通弯道目标速度（单位mm/s）
extern float BigCurveSpeed;   // 大弯道目标速度（单位mm/s）
extern float LostSpeed;       // 丢线/未定义状态安全速度（单位mm/s）
extern float JunctionSpeed;   // 路口锁存转向时的前进速度（单位mm/s）
extern float FinishSpeed;     // 最后一段接近终点的巡线速度（单位mm/s）
extern float ForwardLimit;   // 前行限制(转向差速大于该值则前进速度降为0)
extern float Track_Turn_Scale; // 转向差速(mm/s) 换算到转向环目标幅值的系数
extern float Track_Speed_RiseStep;  // 每个5ms周期允许的加速步长（mm/s）
extern float Track_Speed_FallStep;  // 每个5ms周期允许的减速步长（mm/s）
extern u8 Track_CenterConfirmCycles; // 恢复直行前需要连续确认的周期数

/*==============================传感器状态定义==============================*
 * 白底为1、黑线为0。sensor_state = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4		   *
 * 物理从左到右为 DH4 DH3 DH2 DH1。下列枚举名沿用原厂命名，和车体左右相反； *
 * 转向符号才是车体方向：turn_diff>0 右转，turn_diff<0 左转。			   *
 *============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 四路全黑，丁字口/终点
    STATE_LEFT_90_A     = 1,    // 0001 - 物理右岔口（右三路压线）
	STATE_LEFT_90_B		= 3,	// 0011 - 物理右偏/右岔口（右两路压线）
    STATE_RIGHT_90_A    = 8,  	// 1000 - 物理左岔口（左三路压线）
	STATE_RIGHT_90_B    = 12,	// 1100 - 物理左偏/左岔口（左两路压线）
    STATE_LEFT_BIG      = 7,    // 0111 - 只有右外压线
    STATE_RIGHT_BIG     = 14,   // 1110 - 只有左外压线
    STATE_LEFT_SMALL    = 11,   // 1011 - 只有右中压线
    STATE_RIGHT_SMALL   = 13,   // 1101 - 只有左中压线
    STATE_STRAIGHT      = 9,    // 1001 - 居中（白黑黑白）
    STATE_LOST          = 15    // 1111 - 四路全白
} SensorState_t;

/*==============================巡线结果(输出给控制环)========================*/
extern float base_speed_mm ; // 当前巡线目标前进速度（mm/s）
extern float turn_diff ;     // 当前转向差速（左+右-，单位：mm/s）
extern u8 Track_state ;      // 最新识别的传感器状态(SensorState_t, 供OLED显示)

void TrackModule_Init(void);   // 进入巡线模式：配置PB8/PC8/PC4/PC9为输入下拉
void TrackModule_DeInit(void); // 退出巡线模式：恢复扩展接口为PS2默认状态
void Track_ResetLogic(void);   // 抬车/重新落地时复位路线状态，恢复普通巡线
void IRDM_line_inspection(void);// 巡线算法：读4路传感器->计算base_speed_mm/turn_diff
#endif
