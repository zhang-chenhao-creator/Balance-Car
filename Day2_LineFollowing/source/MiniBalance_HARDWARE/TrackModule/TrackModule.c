/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

说明：移植自 STM32F103RCT6_通用巡线例程_C10B / 椭圆巡线例程_C10B
  算法与参考例程一致（仅去掉了"赋值给两个电机目标速度"的语句，因为平衡
  小车的速度环/转向环独立，见 control.c 的 Velocity()/Turn()）
All rights reserved
***********************************************/
#include "TrackModule.h"

/*=============================================================================*
 * 可调参数区域																   *
 * 默认使用"通用巡线"参数；"椭圆巡线"参数见注释，可自行切换					   *
 *=============================================================================*/
float Turn90Angle  = 80;   // 直角弯转向参数
float TurnMaxAngle = 65;   // 大弯道转向参数（椭圆例程为45）
float TurnMidAngle = 40;   // 中等转向参数（丢线时使用）（椭圆例程为25）
float TurnMinAngle = 15;   // 微调转向参数
float BaseSpeed = 500;     // 基础巡线速度（直行时的速度，mm/s），为原通用参数约2倍
float ForwardLimit = 50;   // 前行限制(转向差速大于该值时前进速度降为0)（椭圆例程为80）
float Track_Turn_Scale = 0.7f; // turn_diff(mm/s) → 转向环目标幅值 的换算系数
                              // 参考:遥控全速转向时 Turn_Target≈54(即turn_diff≈77时)

/* 传感器状态定义见 TrackModule.h 中的 SensorState_t */

float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速
u8 Track_state = 0;     // 最新识别的传感器状态(供OLED显示)

/*=============================================================================*
 * 巡线功能函数（计算 base_speed_mm / turn_diff 两个输出量）					   *
 * 在5ms中断中调用															   *
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;// 记录上一次的状态

    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
    Track_state = sensor_state;   // 供OLED显示当前识别状态
    /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)
    {
       case STATE_CROSS:// 交叉路口处理
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://右大弯
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://左微调
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://右微调
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            break;
        case STATE_LOST://丢线处理
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//继续左转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//继续右转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//继续左转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//继续右转
			//其余情况维持上一次的转向(与参考例程一致)
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            break;
    }
	//保存传感器状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
	// 转向差速越大，基础速度越低：保证转弯时减速
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
}

/**************************************************************************
Function: 进入巡线模式时的引脚配置
Input   : none
Output  : none
函数功能：把扩展接口(PB8/PC8/PC4/PC9)配置为4路巡线传感器输入(下拉)。
          PS2手柄与巡线模块共用该接口，两者不能同时使用。
入口参数：无
返回  值：无
**************************************************************************/
void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();      // 使能 GPIOC 时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();      // 使能 GPIOB 时钟

	// 配置 GPIOC 的 8、4、9 引脚为输入下拉模式 (DH1/DH2/DH3)
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 配置 GPIOB 的 8 引脚为输入下拉模式 (DH4)
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************************************************************
Function: 退出巡线模式时的引脚恢复
Input   : none
Output  : none
函数功能：恢复扩展接口为PS2手柄默认配置(与gpio.c中MX_GPIO_Init一致)：
          PC4/PC8/PC9 推挽输出高、PB8 输入上拉。
入口参数：无
返回  值：无
**************************************************************************/
void TrackModule_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// PS2: DO=PC9  CS=PC4  CLK=PC8  (推挽输出, 初始高)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9, GPIO_PIN_SET);
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PS2: DI=PB8 (输入上拉)
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}
